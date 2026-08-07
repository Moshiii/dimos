// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// RTAB-Map RGB-D SLAM as a dimos native module. No ROS: this drives the rtabmap
// C++ library directly.
//
// in:  camera_info, plus one pair chosen by input_mode:
//        rgbd      -> color_image (rgb8) + depth_image (16UC1, mm)
//        stereo_ir -> image_left + image_right (mono8, rectified)
// out: odometry, corrected_odometry, map_tf, cloud_map
//
// Two rtabmap objects do two different jobs, and the split is why there are two
// pose outputs. rtabmap::Odometry is frame-to-map visual odometry: it drifts but
// never jumps, so it owns odom->base_link. rtabmap::Rtabmap is the appearance-based
// loop closure detector and pose graph on top of it: its correction jumps at a
// closure, so it owns map->odom. Consumers that want a smooth pose read odometry;
// consumers that want a globally consistent one read corrected_odometry.
//
// cloud_map is assembled from the *optimized* graph rather than accumulated as
// frames arrive, so a loop closure visibly pulls the whole map back together
// instead of leaving two smeared copies of the same wall.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>

#include "dimos/native.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

namespace {

constexpr std::int64_t kNsPerSec = 1000000000LL;
constexpr std::int32_t kPointFieldFloat32 = 7;

double stamp_to_sec(const std_msgs::Header& header) {
    return static_cast<double>(header.stamp.sec) +
           static_cast<double>(header.stamp.nsec) / static_cast<double>(kNsPerSec);
}

/// Wrap the LCM payload without copying. The Mat borrows the message's buffer, so
/// it stays valid only while that message is alive.
cv::Mat borrow_mat(const sensor_msgs::Image& img, int cv_type) {
    return cv::Mat(img.height, img.width, cv_type, const_cast<std::uint8_t*>(img.data.data()),
                   static_cast<std::size_t>(img.step));
}

}  // namespace

struct RtabmapConfig {
    /// "rgbd"      colour + aligned depth, metric odometry and a dense map.
    /// "stereo_ir" the infrared pair; rtabmap computes disparity itself.
    /// "mono"      colour only. A single camera cannot give metric odometry, so this
    ///             is rtabmap's classic appearance-only loop closure detection and it
    ///             publishes no pose and no cloud. Useful as a place-recognition
    ///             throughput baseline.
    std::string input_mode;
    /// Run the pose graph and loop closure detector at all. With it off only the
    /// visual odometry runs, which is the cheapest this module gets: no graph, no
    /// map, no map->odom correction. rgbd and stereo_ir only -- mono *is* the loop
    /// closure detector, so turning it off there would leave nothing to run.
    bool enable_loop_closure;
    /// Log processing time (mean and worst) every this many seconds, so a run
    /// reports the rate it could sustain rather than the rate it was fed at. 0 is off.
    double timing_report_period_s;
    /// Distance between the two infrared imagers, metres. Only read in stereo_ir
    /// mode -- in rgbd mode the camera has already done the triangulation.
    double baseline_m;
    std::string database_path;
    bool delete_db_on_start;
    /// 0 = frame-to-map, 1 = frame-to-frame. See rtabmap's Odom/Strategy.
    int odom_strategy;
    /// Consecutive odometry failures before the odometry resets itself. 0 never resets.
    int odom_reset_countdown;
    /// How often rtabmap adds a node and looks for a loop closure, Hz. Enforced by
    /// this module (see should_detect) because Rtabmap itself does not read the
    /// equivalent parameter -- only RtabmapThread does, and this drives Rtabmap
    /// directly. Odometry still runs on every frame regardless.
    double detection_rate_hz;
    /// Rtabmap/LoopThr. Confidence a hypothesis needs before it counts as a loop
    /// closure. Lower finds more, and accepts more false ones.
    double loop_closure_threshold;
    /// Rtabmap/TimeThr, milliseconds, 0 = no limit. rtabmap's real-time mechanism:
    /// when an update takes longer than this, nodes are moved out of working memory
    /// until it fits. This is the knob that trades map size for a rate guarantee.
    double time_budget_ms;
    /// Rtabmap/MemoryThr, 0 = unlimited. Hard cap on working-memory nodes.
    int memory_threshold_nodes;
    /// RGBD/LinearUpdate and RGBD/AngularUpdate: how far the robot must move before
    /// a frame is allowed to become a node. This, not detection_rate_hz, is why a
    /// stationary camera never grows the map. 0 disables the gate.
    double linear_update_m;
    double angular_update_rad;
    /// Kp/MaxFeatures for the loop closure dictionary, Vis/MaxFeatures for
    /// registration. Both are first-order speed knobs.
    int max_features;
    int vis_max_features;
    /// Vis/MinInliers. Correspondences needed to accept a transform.
    int min_inliers;
    /// Kp/DetectorStrategy. 8=GFTT/ORB (rtabmap's default), 2=ORB, 1=SIFT.
    int feature_type;
    /// Optimizer/Strategy: 0=TORO, 1=g2o, 2=GTSAM, 3=Ceres. Only the ones this
    /// build was linked against will work; g2o and TORO are always available.
    int optimizer_strategy;
    double max_depth_m;
    double min_depth_m;
    /// Pixel stride when turning a depth frame into points. 1 keeps every pixel.
    int cloud_decimation;
    double cloud_voxel_size_m;
    double cloud_publish_period_s;
    bool publish_cloud_map;
    /// Upper bound on the published cloud. Exceeding it is logged, never silent.
    int cloud_max_points;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;
    /// Colour and depth further apart than this are not the same instant.
    double max_pair_skew_s;
    // Owned by the python half, which turns the pose streams into tf and into
    // nav_msgs/Path for the viewer. They cross the boundary because config for one
    // module lives in one struct; nothing below reads them.
    bool publish_map_to_odom;
    int path_max_poses;
    double path_publish_period_s;
    double path_min_step_m;
    /// Raw rtabmap Parameters (e.g. {"Vis/MinInliers": "15"}), applied last so they
    /// win over everything above. The escape hatch for the ~600 parameters that are
    /// not worth a config field each.
    std::map<std::string, std::string> extra_parameters;

    bool stereo() const { return input_mode == "stereo_ir"; }
    bool mono() const { return input_mode == "mono"; }
    /// mono has no odometry to correct, so its only job is loop closure.
    bool loop_closure() const { return enable_loop_closure || mono(); }

    void validate() const {
        if (input_mode != "rgbd" && input_mode != "stereo_ir" && input_mode != "mono") {
            throw std::runtime_error("input_mode must be 'rgbd', 'stereo_ir' or 'mono', got '" +
                                     input_mode + "'");
        }
        if (stereo()) {
            dimos::native::require_positive(baseline_m, "baseline_m");
        }
        dimos::native::require_positive(detection_rate_hz, "detection_rate_hz");
        dimos::native::require_positive(cloud_publish_period_s, "cloud_publish_period_s");
        dimos::native::require_positive(max_pair_skew_s, "max_pair_skew_s");
        if (cloud_decimation < 1) {
            throw std::runtime_error("cloud_decimation must be at least 1");
        }
        if (cloud_max_points < 1) {
            throw std::runtime_error("cloud_max_points must be at least 1");
        }
    }
};

class RtabmapSlam : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<RtabmapConfig>();

        builder.input<sensor_msgs::CameraInfo>("camera_info", &RtabmapSlam::on_camera_info, this);
        // Only the active mode's ports are subscribed, so the other pair may be left
        // unwired in the blueprint. camera_info must describe whichever pair is used:
        // the colour intrinsics in rgbd mode, the left infrared's in stereo_ir.
        if (cfg_.stereo()) {
            builder.input<sensor_msgs::Image>("image_left", &RtabmapSlam::on_primary, this);
            builder.input<sensor_msgs::Image>("image_right", &RtabmapSlam::on_secondary, this);
        } else {
            builder.input<sensor_msgs::Image>("color_image", &RtabmapSlam::on_primary, this);
            // mono has no second image; it never pairs.
            if (!cfg_.mono()) {
                builder.input<sensor_msgs::Image>("depth_image", &RtabmapSlam::on_secondary, this);
            }
        }

        odometry_ = builder.output<nav_msgs::Odometry>("odometry");
        corrected_odometry_ = builder.output<nav_msgs::Odometry>("corrected_odometry");
        map_tf_ = builder.output<nav_msgs::Odometry>("map_tf");
        cloud_map_ = builder.output<sensor_msgs::PointCloud2>("cloud_map");
    }

    void setup() override {
        // rtabmap's own logger writes free-form text; the coordinator expects one
        // JSON object per line. Warnings and errors are worth the format break,
        // routine info is not.
        ULogger::setType(ULogger::kTypeConsole);
        ULogger::setLevel(ULogger::kWarning);
    }

    void teardown() override {
        if (initialized_ && cfg_.loop_closure()) {
            rtabmap_.close();
        }
        logging::info("rtabmap shutting down",
                      {logging::Field("frames", static_cast<std::int64_t>(frames_)),
                       logging::Field("tracked", static_cast<std::int64_t>(tracked_)),
                       logging::Field("nodes", static_cast<std::int64_t>(cloud_cache_.size())),
                       logging::Field("loop_closures", static_cast<std::int64_t>(loop_closures_)),
                       logging::Field("odom_losses", static_cast<std::int64_t>(odom_losses_))});
    }

private:
    void on_camera_info(const sensor_msgs::CameraInfo& info) {
        if (initialized_) {
            return;  // the rig is fixed once rtabmap has been initialized against it
        }
        width_ = info.width;
        height_ = info.height;
        fx_ = info.K[0];
        fy_ = info.K[4];
        cx_ = info.K[2];
        cy_ = info.K[5];
        have_info_ = fx_ > 0.0 && fy_ > 0.0;
    }

    /// Colour in rgbd and mono modes, left infrared in stereo_ir mode.
    void on_primary(const sensor_msgs::Image& img) {
        primary_ = img;
        have_primary_ = true;
        if (cfg_.mono()) {
            process_mono();
        } else {
            try_process();
        }
    }

    /// Appearance-only loop closure on a single image. No pose: one camera cannot
    /// resolve scale, so there is nothing metric to publish.
    void process_mono() {
        ensure_initialized();
        if (!initialized_) {
            return;
        }
        const double stamp = stamp_to_sec(primary_.header);
        if (have_last_stamp_ && stamp <= last_stamp_) {
            have_primary_ = false;
            return;
        }
        cv::Mat bgr;
        cv::cvtColor(borrow_mat(primary_, CV_8UC3), bgr, cv::COLOR_RGB2BGR);
        have_primary_ = false;
        last_stamp_ = stamp;
        have_last_stamp_ = true;
        ++frames_;

        if (!should_detect(stamp)) {
            return;
        }
        const auto started = std::chrono::steady_clock::now();
        rtabmap_.process(bgr, ++seq_);
        record_timing(started);
        ++tracked_;
        note_loop_closure();
    }

    /// Wall clock spent inside rtabmap for one frame. Reported as a sustainable rate
    /// so a run says what it *could* have kept up with, not merely what it was fed.
    void record_timing(std::chrono::steady_clock::time_point started) {
        const auto now = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(now - started).count();
        timing_sum_ms_ += ms;
        timing_max_ms_ = std::max(timing_max_ms_, ms);
        ++timing_count_;

        if (cfg_.timing_report_period_s <= 0.0) {
            return;
        }
        if (timing_count_ == 1) {
            timing_window_start_ = started;
            return;
        }
        const double elapsed = std::chrono::duration<double>(now - timing_window_start_).count();
        if (elapsed < cfg_.timing_report_period_s) {
            return;
        }
        const double mean_ms = timing_sum_ms_ / static_cast<double>(timing_count_);
        logging::info(
            "rtabmap timing",
            {logging::Field("mode", cfg_.input_mode),
             logging::Field("loop_closure", cfg_.loop_closure()),
             logging::Field("frames", static_cast<std::int64_t>(timing_count_)),
             logging::Field("mean_ms", mean_ms), logging::Field("max_ms", timing_max_ms_),
             // What the mean per-frame cost implies this build could sustain.
             logging::Field("sustainable_hz", mean_ms > 0.0 ? 1000.0 / mean_ms : 0.0),
             // What it was actually fed, so a fed-limited run is not read as a ceiling.
             logging::Field("observed_hz", static_cast<double>(timing_count_) / elapsed)});
        timing_sum_ms_ = 0.0;
        timing_max_ms_ = 0.0;
        timing_count_ = 0;
        timing_window_start_ = now;
    }

    /// Depth in rgbd mode, right infrared in stereo_ir mode.
    void on_secondary(const sensor_msgs::Image& img) {
        secondary_ = img;
        have_secondary_ = true;
        try_process();
    }

    /// localTransform is the optical -> body rotation, so every pose and point
    /// rtabmap returns is already in the x-forward/z-up convention the rest of the
    /// stack uses. It also means the pose published as base_frame is the *camera's*:
    /// on a robot whose camera is not the body origin, either set base_frame to the
    /// camera's own frame and let the static tf tree carry it to the body, or fold
    /// the mount extrinsic into this transform.
    void ensure_initialized() {
        if (initialized_) {
            return;
        }
        // mono is appearance-only: no geometry, so no intrinsics needed and no waiting
        // on camera_info before the first frame can be processed.
        if (!have_info_ && !cfg_.mono()) {
            return;
        }
        model_ = rtabmap::CameraModel(fx_, fy_, cx_, cy_, rtabmap::CameraModel::opticalRotation(),
                                      0.0, cv::Size(width_, height_));
        if (cfg_.stereo()) {
            // The infrared pair leaves the D4xx already rectified, which is what lets
            // one pinhole model plus a baseline describe the rig.
            stereo_model_ = rtabmap::StereoCameraModel(fx_, fy_, cx_, cy_, cfg_.baseline_m,
                                                       rtabmap::CameraModel::opticalRotation(),
                                                       cv::Size(width_, height_));
        }

        rtabmap::ParametersMap params = rtabmap::Parameters::getDefaultParameters();
        // Metric SLAM needs a pose per node. mono has none, so it falls back to
        // rtabmap's original bag-of-words loop closure detection over images alone.
        params.at(rtabmap::Parameters::kRGBDEnabled()) = cfg_.mono() ? "false" : "true";
        params.at(rtabmap::Parameters::kOdomStrategy()) = std::to_string(cfg_.odom_strategy);
        params.at(rtabmap::Parameters::kOdomResetCountdown()) =
            std::to_string(cfg_.odom_reset_countdown);
        // Deliberately NOT passed to rtabmap: Rtabmap/DetectionRate is read by
        // RtabmapThread, not by Rtabmap itself, and this module drives Rtabmap
        // directly. Setting the parameter here would type-check, be accepted, and do
        // nothing -- so the throttle is enforced below in should_detect() instead.
        params.at(rtabmap::Parameters::kRtabmapLoopThr()) =
            std::to_string(cfg_.loop_closure_threshold);
        params.at(rtabmap::Parameters::kRtabmapTimeThr()) = std::to_string(cfg_.time_budget_ms);
        params.at(rtabmap::Parameters::kRtabmapMemoryThr()) =
            std::to_string(cfg_.memory_threshold_nodes);
        params.at(rtabmap::Parameters::kRGBDLinearUpdate()) = std::to_string(cfg_.linear_update_m);
        params.at(rtabmap::Parameters::kRGBDAngularUpdate()) =
            std::to_string(cfg_.angular_update_rad);
        params.at(rtabmap::Parameters::kKpMaxFeatures()) = std::to_string(cfg_.max_features);
        params.at(rtabmap::Parameters::kVisMaxFeatures()) = std::to_string(cfg_.vis_max_features);
        params.at(rtabmap::Parameters::kVisMinInliers()) = std::to_string(cfg_.min_inliers);
        params.at(rtabmap::Parameters::kKpDetectorStrategy()) = std::to_string(cfg_.feature_type);
        params.at(rtabmap::Parameters::kOptimizerStrategy()) =
            std::to_string(cfg_.optimizer_strategy);
        // Publishing the last signature is what makes getStatistics() carry the
        // loop closure id, which is the only way to count closures from here.
        params.at(rtabmap::Parameters::kRtabmapPublishStats()) = "true";

        for (const auto& [key, value] : cfg_.extra_parameters) {
            if (params.find(key) == params.end()) {
                throw std::runtime_error("extra_parameters: '" + key +
                                         "' is not an rtabmap parameter");
            }
            params.at(key) = value;
        }

        // Odometry gets the same parameter map: it reads the Odom/* and Vis/* keys
        // out of it and ignores the rest. mono has no metric odometry to run.
        if (!cfg_.mono()) {
            odometry_engine_.reset(rtabmap::Odometry::create(params));
        }

        if (cfg_.loop_closure()) {
            // sqlite will not create the directory itself, and rtabmap reports the
            // failure once per frame rather than at startup, so a missing parent shows
            // up as an unbounded stream of "unable to open database file".
            const std::filesystem::path database(cfg_.database_path);
            if (database.has_parent_path()) {
                std::error_code ec;
                std::filesystem::create_directories(database.parent_path(), ec);
                if (ec) {
                    throw std::runtime_error("cannot create database directory '" +
                                             database.parent_path().string() + "': " +
                                             ec.message());
                }
            }
            if (cfg_.delete_db_on_start) {
                UFile::erase(cfg_.database_path);
            }
            rtabmap_.init(params, cfg_.database_path);
        }
        initialized_ = true;

        logging::info("rtabmap initialized",
                      {logging::Field("mode", cfg_.input_mode),
                       logging::Field("loop_closure", cfg_.loop_closure()),
                       logging::Field("width", static_cast<std::int64_t>(width_)),
                       logging::Field("height", static_cast<std::int64_t>(height_)),
                       logging::Field("fx", fx_), logging::Field("fy", fy_),
                       logging::Field("odom_strategy",
                                      static_cast<std::int64_t>(cfg_.odom_strategy))});
    }

    void try_process() {
        if (!have_primary_ || !have_secondary_) {
            return;
        }
        ensure_initialized();
        if (!initialized_) {
            return;  // no camera_info yet
        }

        const double t_primary = stamp_to_sec(primary_.header);
        const double t_secondary = stamp_to_sec(secondary_.header);
        if (std::fabs(t_primary - t_secondary) > cfg_.max_pair_skew_s) {
            return;  // wait for the matching frame rather than pairing across motion
        }
        if (have_last_stamp_ && t_primary <= last_stamp_) {
            have_primary_ = have_secondary_ = false;
            return;  // rtabmap requires strictly increasing stamps
        }
        if (primary_.width != secondary_.width || primary_.height != secondary_.height) {
            DIMOS_ERROR_THROTTLED(logging::from_secs(5), "paired image sizes differ",
                                  logging::Field("primary_width",
                                                 static_cast<std::int64_t>(primary_.width)),
                                  logging::Field("secondary_width",
                                                 static_cast<std::int64_t>(secondary_.width)));
            have_primary_ = have_secondary_ = false;
            return;
        }

        // Every Mat handed to rtabmap is freshly allocated and never touched again.
        // cv::Mat assignment is a shallow, non-owning share when it was built over
        // someone else's buffer, and rtabmap keeps the SensorData alive past the
        // process() call -- Rtabmap/PublishLastSignature parks it in the statistics
        // until the next frame. Borrowing the LCM payload directly would leave that
        // copy pointing at a message which is freed on the next callback, and reusing
        // one scratch Mat would rewrite it underneath rtabmap in place.
        rtabmap::SensorData data;
        if (cfg_.stereo()) {
            data.setStereoImage(borrow_mat(primary_, CV_8UC1).clone(),
                                borrow_mat(secondary_, CV_8UC1).clone(), stereo_model_);
            data.setId(++seq_);
            data.setStamp(t_primary);
        } else {
            // rtabmap reads colour as BGR, both for the greyscale it tracks on and
            // for the colours it would put on points. cvtColor allocates its output,
            // so this is already a fresh buffer.
            cv::Mat bgr;
            cv::cvtColor(borrow_mat(primary_, CV_8UC3), bgr, cv::COLOR_RGB2BGR);
            data = rtabmap::SensorData(bgr, borrow_mat(secondary_, CV_16UC1).clone(), model_,
                                       ++seq_, t_primary);
        }

        have_primary_ = have_secondary_ = false;
        last_stamp_ = t_primary;
        have_last_stamp_ = true;
        ++frames_;

        // Timed from here: everything below is rtabmap's work on this frame, and the
        // mean of it is what caps the rate this module can sustain.
        const auto started = std::chrono::steady_clock::now();

        rtabmap::OdometryInfo info;
        const rtabmap::Transform pose = odometry_engine_->process(data, &info);
        if (pose.isNull()) {
            ++odom_losses_;
            record_timing(started);
            DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(1), "odometry lost",
                                 logging::Field("losses",
                                                static_cast<std::int64_t>(odom_losses_)));
            return;
        }
        ++tracked_;
        odom_pose_ = pose;
        publish_odometry(pose, t_primary);

        if (cfg_.loop_closure() && should_detect(t_primary)) {
            // Returns true when this frame became a node in the graph. The pose graph
            // only moves on those frames, so that is when the cache and map change.
            if (rtabmap_.process(data, pose)) {
                cache_node_cloud(data);
                note_loop_closure();
            }
            publish_correction(t_primary);
            maybe_publish_cloud_map(t_primary);
        }
        record_timing(started);
    }

    /// Rate-limit graph updates, the job RtabmapThread would do if this module used
    /// it. Without this every frame becomes a candidate node: at 30 fps that is 30x
    /// the intended graph growth, and the bag-of-words search runs 30x as often.
    bool should_detect(double stamp) {
        if (!have_last_detection_) {
            have_last_detection_ = true;
            last_detection_stamp_ = stamp;
            return true;
        }
        if (stamp - last_detection_stamp_ < 1.0 / cfg_.detection_rate_hz) {
            return false;
        }
        last_detection_stamp_ = stamp;
        return true;
    }

    void note_loop_closure() {
        const int loop_id = rtabmap_.getLoopClosureId();
        if (loop_id > 0) {
            ++loop_closures_;
            logging::info("rtabmap loop closure",
                          {logging::Field("count", static_cast<std::int64_t>(loop_closures_)),
                           logging::Field("from", static_cast<std::int64_t>(
                                                      rtabmap_.getLastLocationId())),
                           logging::Field("to", static_cast<std::int64_t>(loop_id))});
        }
    }

    /// Cache the node's points in the *base* frame, not the map frame. The graph
    /// re-optimizes every pose on a loop closure, so anything stored already
    /// positioned would be stale the moment a closure lands.
    void cache_node_cloud(const rtabmap::SensorData& data) {
        if (!cfg_.publish_cloud_map) {
            return;
        }
        const int id = rtabmap_.getLastLocationId();
        if (id <= 0 || cloud_cache_.count(id) != 0) {
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::cloudFromSensorData(
            data, cfg_.cloud_decimation, static_cast<float>(cfg_.max_depth_m),
            static_cast<float>(cfg_.min_depth_m));
        if (cloud->empty()) {
            return;
        }
        if (cfg_.cloud_voxel_size_m > 0.0) {
            cloud = rtabmap::util3d::voxelize(cloud, static_cast<float>(cfg_.cloud_voxel_size_m));
        }
        cloud_cache_[id] = cloud;
    }

    void publish_odometry(const rtabmap::Transform& pose, double stamp) {
        nav_msgs::Odometry msg{};
        fill_pose(msg, pose, stamp, cfg_.odom_frame, cfg_.base_frame);
        odometry_.publish(msg);
    }

    /// map->odom is rtabmap's correction, and map->base_link the corrected pose.
    /// The correction is identity until the graph has moved, which is the honest
    /// answer rather than a missing stream.
    void publish_correction(double stamp) {
        const rtabmap::Transform correction = rtabmap_.getMapCorrection();
        if (correction.isNull()) {
            return;
        }
        nav_msgs::Odometry map_to_odom{};
        fill_pose(map_to_odom, correction, stamp, cfg_.map_frame, cfg_.odom_frame);
        map_tf_.publish(map_to_odom);

        nav_msgs::Odometry corrected{};
        fill_pose(corrected, correction * odom_pose_, stamp, cfg_.map_frame, cfg_.base_frame);
        corrected_odometry_.publish(corrected);
    }

    /// Re-assemble from the optimized graph rather than appending, so a loop
    /// closure moves the whole map instead of adding a second copy of it.
    void maybe_publish_cloud_map(double stamp) {
        if (!cfg_.publish_cloud_map || cloud_cache_.empty()) {
            return;
        }
        if (have_last_cloud_stamp_ && stamp - last_cloud_stamp_ < cfg_.cloud_publish_period_s) {
            return;
        }
        last_cloud_stamp_ = stamp;
        have_last_cloud_stamp_ = true;

        const std::map<int, rtabmap::Transform>& poses = rtabmap_.getLocalOptimizedPoses();
        pcl::PointCloud<pcl::PointXYZ>::Ptr assembled(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& [id, pose] : poses) {
            auto it = cloud_cache_.find(id);
            if (it == cloud_cache_.end() || pose.isNull()) {
                continue;
            }
            *assembled += *rtabmap::util3d::transformPointCloud(it->second, pose);
        }
        if (assembled->empty()) {
            return;
        }
        if (cfg_.cloud_voxel_size_m > 0.0) {
            assembled =
                rtabmap::util3d::voxelize(assembled, static_cast<float>(cfg_.cloud_voxel_size_m));
        }

        std::size_t published = assembled->size();
        const auto limit = static_cast<std::size_t>(cfg_.cloud_max_points);
        if (published > limit) {
            published = limit;
            DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(10), "cloud_map truncated",
                                 logging::Field("points",
                                                static_cast<std::int64_t>(assembled->size())),
                                 logging::Field("cloud_max_points",
                                                static_cast<std::int64_t>(limit)));
        }
        publish_cloud(*assembled, published, stamp);
    }

    void publish_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::size_t count,
                       double stamp) {
        sensor_msgs::PointCloud2 msg{};
        fill_stamp(msg.header, stamp);
        msg.header.frame_id = cfg_.map_frame;
        msg.height = 1;
        msg.width = static_cast<std::int32_t>(count);
        msg.is_bigendian = 0;
        msg.is_dense = 1;
        msg.point_step = 3 * static_cast<std::int32_t>(sizeof(float));
        msg.row_step = msg.point_step * msg.width;

        const char* names[3] = {"x", "y", "z"};
        msg.fields.resize(3);
        for (int i = 0; i < 3; ++i) {
            msg.fields[i].name = names[i];
            msg.fields[i].offset = i * static_cast<std::int32_t>(sizeof(float));
            msg.fields[i].datatype = kPointFieldFloat32;
            msg.fields[i].count = 1;
        }
        msg.fields_length = static_cast<std::int32_t>(msg.fields.size());

        msg.data.resize(static_cast<std::size_t>(msg.row_step));
        auto* out = reinterpret_cast<float*>(msg.data.data());
        for (std::size_t i = 0; i < count; ++i) {
            out[3 * i + 0] = cloud[i].x;
            out[3 * i + 1] = cloud[i].y;
            out[3 * i + 2] = cloud[i].z;
        }
        msg.data_length = static_cast<std::int32_t>(msg.data.size());
        cloud_map_.publish(msg);
    }

    static void fill_stamp(std_msgs::Header& header, double stamp) {
        const auto ns = static_cast<std::int64_t>(stamp * static_cast<double>(kNsPerSec));
        header.stamp.sec = static_cast<std::int32_t>(ns / kNsPerSec);
        header.stamp.nsec = static_cast<std::int32_t>(ns % kNsPerSec);
    }

    static void fill_pose(nav_msgs::Odometry& msg, const rtabmap::Transform& pose, double stamp,
                          const std::string& frame, const std::string& child) {
        fill_stamp(msg.header, stamp);
        msg.header.frame_id = frame;
        msg.child_frame_id = child;
        msg.pose.pose.position.x = pose.x();
        msg.pose.pose.position.y = pose.y();
        msg.pose.pose.position.z = pose.z();
        const Eigen::Quaternionf q = pose.getQuaternionf();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
    }

    RtabmapConfig cfg_{};

    rtabmap::Rtabmap rtabmap_;
    std::unique_ptr<rtabmap::Odometry> odometry_engine_;
    rtabmap::CameraModel model_;
    rtabmap::StereoCameraModel stereo_model_;
    rtabmap::Transform odom_pose_;
    bool initialized_{false};

    bool have_info_{false};
    std::int32_t width_{0};
    std::int32_t height_{0};
    double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

    sensor_msgs::Image primary_{}, secondary_{};
    bool have_primary_{false}, have_secondary_{false};
    double last_stamp_{0.0};
    bool have_last_stamp_{false};

    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cache_;
    double last_detection_stamp_{0.0};
    bool have_last_detection_{false};
    double last_cloud_stamp_{0.0};
    bool have_last_cloud_stamp_{false};

    double timing_sum_ms_{0.0};
    double timing_max_ms_{0.0};
    std::uint64_t timing_count_{0};
    std::chrono::steady_clock::time_point timing_window_start_{};

    int seq_{0};
    std::uint64_t frames_{0};
    std::uint64_t tracked_{0};
    std::uint64_t odom_losses_{0};
    std::uint64_t loop_closures_{0};

    Output<nav_msgs::Odometry> odometry_;
    Output<nav_msgs::Odometry> corrected_odometry_;
    Output<nav_msgs::Odometry> map_tf_;
    Output<sensor_msgs::PointCloud2> cloud_map_;
};

int main() {
    dimos::native::run_with_transport<RtabmapSlam>();
    return 0;
}
