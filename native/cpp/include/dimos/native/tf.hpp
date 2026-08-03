// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Transform graph for C++ native modules. Each edge is buffered per
// (parent, child) pair, and lookups compose the shortest path through the
// graph. Samples are nearest-in-time within a tolerance, not interpolated.

#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include "dimos/native/log.hpp"

namespace dimos::native {

/// How many seconds of history each edge keeps.
inline constexpr double kDefaultTfWindowSecs = 10.0;

/// Seconds since the epoch on the wall clock, the stamp domain transforms use.
inline double now_secs() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/// A rigid transform from `parent` to `child` at a point in time.
///
/// It maps a point expressed in `child` coordinates into `parent` coordinates.
struct Transform {
    std::string parent;
    std::string child;
    double ts = 0.0;
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();

    Eigen::Vector3d translation() const { return iso.translation(); }
    Eigen::Quaterniond rotation() const { return Eigen::Quaterniond(iso.rotation()); }

    Transform inverse() const { return Transform{child, parent, ts, iso.inverse()}; }

    /// Chain `other` onto this one, keeping this transform's parent and stamp.
    Transform compose(const Transform& other) const {
        return Transform{parent, other.child, ts, iso * other.iso};
    }
};

/// One edge's time-sorted history, capped to a fixed-duration window.
class TBuffer {
public:
    struct Sample {
        double ts;
        Eigen::Isometry3d iso;
    };

    explicit TBuffer(double window_secs) : window_secs_(window_secs) {}

    void add(double ts, const Eigen::Isometry3d& iso) {
        // A stamp a whole window behind the newest is a clock reset, not jitter.
        if (!samples_.empty() && ts < samples_.back().ts - window_secs_) {
            samples_.clear();
        }
        auto pos = std::partition_point(samples_.begin(), samples_.end(),
                                        [ts](const Sample& s) { return s.ts <= ts; });
        samples_.insert(pos, Sample{ts, iso});
        // Anchored to the newest sample so a late message cannot widen the window.
        prune(samples_.back().ts - window_secs_);
    }

    std::size_t size() const { return samples_.size(); }

    const Sample* last() const { return samples_.empty() ? nullptr : &samples_.back(); }

    /// Nearest sample in time, preferring the later one on a tie. Null when the
    /// closest sample is further than `tolerance` from `ts`.
    const Sample* find_closest(double ts, std::optional<double> tolerance) const {
        auto pos = std::partition_point(samples_.begin(), samples_.end(),
                                        [ts](const Sample& s) { return s.ts < ts; });
        const Sample* prev = pos == samples_.begin() ? nullptr : &*(pos - 1);
        const Sample* next = pos == samples_.end() ? nullptr : &*pos;
        const Sample* best = nullptr;
        if (prev != nullptr && next != nullptr) {
            best = std::abs(next->ts - ts) <= std::abs(ts - prev->ts) ? next : prev;
        } else {
            best = prev != nullptr ? prev : next;
        }
        if (best == nullptr) {
            return nullptr;
        }
        if (tolerance.has_value() && std::abs(best->ts - ts) > *tolerance) {
            return nullptr;
        }
        return best;
    }

    /// The sample for `time`, or the latest one when no time is given. Without a
    /// tolerance a timed query falls back to the whole window.
    std::optional<Transform> sample(const std::string& parent, const std::string& child,
                                    std::optional<double> time,
                                    std::optional<double> tolerance) const {
        const Sample* s = time.has_value()
                              ? find_closest(*time, tolerance.value_or(window_secs_))
                              : last();
        if (s == nullptr) {
            return std::nullopt;
        }
        return Transform{parent, child, s->ts, s->iso};
    }

private:
    void prune(double min_ts) {
        auto drop_to = std::partition_point(samples_.begin(), samples_.end(),
                                            [min_ts](const Sample& s) { return s.ts < min_ts; });
        samples_.erase(samples_.begin(), drop_to);
    }

    double window_secs_;
    std::deque<Sample> samples_;
};

/// The transform graph: one TBuffer per (parent, child) edge.
class MultiTBuffer {
public:
    explicit MultiTBuffer(double window_secs) : window_secs_(window_secs) {}

    void receive(const std::string& parent, const std::string& child, double ts,
                 const Eigen::Isometry3d& iso) {
        buffers_.try_emplace(Key{parent, child}, window_secs_).first->second.add(ts, iso);
    }

    /// Every frame sharing an edge with `frame`, in either direction.
    std::vector<std::string> connections(const std::string& frame) const {
        std::vector<std::string> out;
        for (const auto& entry : buffers_) {
            const auto& [parent, child] = entry.first;
            if (parent == frame) {
                out.push_back(child);
            }
            if (child == frame) {
                out.push_back(parent);
            }
        }
        return out;
    }

    /// One edge: the forward buffer, else the reverse buffer inverted.
    std::optional<Transform> edge(const std::string& parent, const std::string& child,
                                  std::optional<double> time,
                                  std::optional<double> tolerance) const {
        if (parent == child) {
            return Transform{parent, child, time.value_or(now_secs()),
                             Eigen::Isometry3d::Identity()};
        }
        auto forward = buffers_.find(Key{parent, child});
        if (forward != buffers_.end()) {
            return forward->second.sample(parent, child, time, tolerance);
        }
        auto reverse = buffers_.find(Key{child, parent});
        if (reverse != buffers_.end()) {
            std::optional<Transform> t = reverse->second.sample(child, parent, time, tolerance);
            return t.has_value() ? std::optional<Transform>(t->inverse()) : std::nullopt;
        }
        return std::nullopt;
    }

    /// The transform from `parent` to `child`, direct or composed over the
    /// fewest hops. Null when no path connects them within the tolerance.
    std::optional<Transform> get(const std::string& parent, const std::string& child,
                                 std::optional<double> time,
                                 std::optional<double> tolerance) const {
        std::optional<Transform> direct = edge(parent, child, time, tolerance);
        if (direct.has_value()) {
            return direct;
        }
        std::optional<std::vector<Transform>> path = bfs(parent, child, time, tolerance);
        if (!path.has_value() || path->empty()) {
            return std::nullopt;
        }
        // A composition is only as fresh as its stalest edge.
        double oldest = std::numeric_limits<double>::infinity();
        for (const Transform& step : *path) {
            oldest = std::min(oldest, step.ts);
        }
        Transform composed = path->front();
        for (std::size_t i = 1; i < path->size(); ++i) {
            composed = composed.compose((*path)[i]);
        }
        composed.ts = oldest;
        return composed;
    }

private:
    using Key = std::pair<std::string, std::string>;

    // Edges are resolved during expansion, so one failing its tolerance is not
    // traversed and the search keeps looking for a path that holds.
    std::optional<std::vector<Transform>> bfs(const std::string& parent, const std::string& child,
                                              std::optional<double> time,
                                              std::optional<double> tolerance) const {
        std::deque<std::pair<std::string, std::vector<Transform>>> queue;
        queue.emplace_back(parent, std::vector<Transform>{});
        std::set<std::string> visited{parent};

        while (!queue.empty()) {
            auto [frame, path] = std::move(queue.front());
            queue.pop_front();
            if (frame == child) {
                return path;
            }
            for (const std::string& next : connections(frame)) {
                if (visited.count(next) != 0) {
                    continue;
                }
                std::optional<Transform> step = edge(frame, next, time, tolerance);
                if (!step.has_value()) {
                    continue;
                }
                visited.insert(next);
                std::vector<Transform> extended = path;
                extended.push_back(*step);
                queue.emplace_back(next, std::move(extended));
            }
        }
        return std::nullopt;
    }

    double window_secs_;
    std::map<Key, TBuffer> buffers_;
};

/// The shared graph behind a Tf handle, guarding it for concurrent access.
///
/// tf arrives on the transport receive thread while the module's own thread
/// reads, so every read takes a shared lock and every write an exclusive one.
class Graph {
public:
    explicit Graph(double window_secs) : buffer_(window_secs) {}

    template <class F>
    void update(F&& edits) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        edits(buffer_);
    }

    std::optional<Transform> get(const std::string& parent, const std::string& child,
                                 std::optional<double> time,
                                 std::optional<double> tolerance) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return buffer_.get(parent, child, time, tolerance);
    }

    /// True at most once per second for this frame pair.
    ///
    /// Keyed per pair rather than per call site, because one call site serves
    /// every lookup and would otherwise mute every pair but the first to miss.
    bool should_warn(const std::string& parent, const std::string& child) const {
        std::lock_guard<std::mutex> lock(warn_mutex_);
        return log::check_and_record(warned_[Key{parent, child}], log::from_secs(1));
    }

private:
    using Key = std::pair<std::string, std::string>;

    mutable std::shared_mutex mutex_;
    MultiTBuffer buffer_;
    mutable std::mutex warn_mutex_;
    mutable std::map<Key, std::atomic<std::uint64_t>> warned_;
};

/// A transform lookup being built. Created by Tf::lookup.
class Lookup {
public:
    Lookup(const Graph* graph, std::string parent, std::string child)
        : graph_(graph), parent_(std::move(parent)), child_(std::move(child)) {}

    /// Take the sample nearest `time` rather than the latest one.
    Lookup& at(double time) {
        time_ = time;
        return *this;
    }

    /// Bound how far, in seconds, the chosen sample may sit from at().
    Lookup& tolerance(double tolerance) {
        tolerance_ = tolerance;
        return *this;
    }

    /// Resolve the lookup against the transforms buffered so far.
    ///
    /// Null when no path connects the frames, or when the nearest sample is
    /// outside the tolerance.
    std::optional<Transform> get() const {
        std::optional<Transform> found = graph_->get(parent_, child_, time_, tolerance_);
        if (!found.has_value()) {
            warn_unresolved();
        }
        return found;
    }

private:
    // A lookup that resolves to nothing is otherwise invisible: the caller sees
    // an empty optional, and the buffer says nothing about which frames missed.
    void warn_unresolved() const {
        if (!graph_->should_warn(parent_, child_)) {
            return;
        }
        log::warn("No transform found between frames",
                  {log::Field("parent", parent_), log::Field("child", child_),
                   log::Field("at", time_.value_or(now_secs())),
                   log::Field("tolerance",
                              tolerance_.value_or(std::numeric_limits<double>::quiet_NaN()))});
    }

    const Graph* graph_;
    std::string parent_;
    std::string child_;
    std::optional<double> time_;
    std::optional<double> tolerance_;
};

/// Where published transforms go after they have fed the local graph.
using TransformSink = std::function<void(const std::vector<Transform>&)>;

/// A cheap-to-copy handle for querying and publishing transforms.
///
/// Copies share one graph, which fills in the background as tf messages arrive.
class Tf {
public:
    Tf(std::shared_ptr<Graph> graph, TransformSink sink)
        : graph_(std::move(graph)), sink_(std::move(sink)) {}

    /// Start a lookup of the transform from `parent` to `child`. Refine it with
    /// at() and tolerance(), then finish with get(). Use get_latest() when no
    /// refinement is needed.
    ///
    ///     auto at_scan = tf.lookup("map", "base_link").at(scan_ts).tolerance(0.1).get();
    Lookup lookup(const std::string& parent, const std::string& child) const {
        return Lookup(graph_.get(), parent, child);
    }

    /// The latest transform from `parent` to `child`.
    std::optional<Transform> get_latest(const std::string& parent,
                                        const std::string& child) const {
        return lookup(parent, child).get();
    }

    /// Publish transforms on the tf topic.
    ///
    /// They feed the local graph first, so a lookup right after sees them
    /// without waiting for the transport round trip.
    void publish(const std::vector<Transform>& transforms) const {
        graph_->update([&transforms](MultiTBuffer& buffer) {
            for (const Transform& t : transforms) {
                buffer.receive(t.parent, t.child, t.ts, t.iso);
            }
        });
        sink_(transforms);
    }

private:
    std::shared_ptr<Graph> graph_;
    TransformSink sink_;
};

}  // namespace dimos::native
