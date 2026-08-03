// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Transform graph for C++ native modules. Each edge is buffered per
// (parent, child) pair, and lookups compose the shortest path through the
// graph. Samples are nearest-in-time within a tolerance, not interpolated.

#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

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

}  // namespace dimos::native
