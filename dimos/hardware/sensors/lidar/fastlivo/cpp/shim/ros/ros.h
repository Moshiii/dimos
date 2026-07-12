// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Minimal fake-ROS layer that lets unmodified FAST-LIVO2 sources compile
// outside ROS. Parameters come from a string map filled by main.cpp
// (CLI args + first CameraInfo message); publishes dispatch to type-erased
// hooks keyed by topic so the glue can forward selected topics onto LCM.

#ifndef DIMOS_ROS_SHIM_H
#define DIMOS_ROS_SHIM_H

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace dimos_shim {

// name → string value ("1"/"0" for bools, CSV for vectors)
inline std::map<std::string, std::string>& params() {
    static std::map<std::string, std::string> p;
    return p;
}

// topic → publish hook. The hook receives a pointer to the message that was
// passed to Publisher::publish; the registrar knows the concrete type from
// the topic it registered for.
inline std::map<std::string, std::function<void(const void*)>>& publish_hooks() {
    static std::map<std::string, std::function<void(const void*)>> h;
    return h;
}

inline std::function<void()>& spin_once_fn() {
    static std::function<void()> fn;
    return fn;
}

inline std::atomic<bool>& running() {
    static std::atomic<bool> r{true};
    return r;
}

inline bool& debug_logging() {
    static bool d = false;
    return d;
}

template <typename T> bool parse_value(const std::string& s, T& out);

template <> inline bool parse_value<bool>(const std::string& s, bool& out) {
    out = (s == "1" || s == "true" || s == "True");
    return true;
}
template <> inline bool parse_value<int>(const std::string& s, int& out) {
    try { out = std::stoi(s); } catch (...) { return false; }
    return true;
}
template <> inline bool parse_value<double>(const std::string& s, double& out) {
    try { out = std::stod(s); } catch (...) { return false; }
    return true;
}
template <> inline bool parse_value<float>(const std::string& s, float& out) {
    try { out = std::stof(s); } catch (...) { return false; }
    return true;
}
template <> inline bool parse_value<std::string>(const std::string& s, std::string& out) {
    out = s;
    return true;
}
template <> inline bool parse_value<std::vector<double>>(const std::string& s, std::vector<double>& out) {
    out.clear();
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        try { out.push_back(std::stod(item)); } catch (...) { return false; }
    }
    return !out.empty();
}
template <> inline bool parse_value<std::vector<int>>(const std::string& s, std::vector<int>& out) {
    out.clear();
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        try { out.push_back(std::stoi(item)); } catch (...) { return false; }
    }
    return !out.empty();
}

}  // namespace dimos_shim

namespace ros {

class Time {
public:
    uint32_t sec = 0;
    uint32_t nsec = 0;

    Time() = default;
    Time(uint32_t s, uint32_t ns) : sec(s), nsec(ns) {}

    double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); }

    Time& fromSec(double t) {
        sec = static_cast<uint32_t>(std::floor(t));
        nsec = static_cast<uint32_t>(std::round((t - sec) * 1e9));
        if (nsec >= 1000000000u) { sec += nsec / 1000000000u; nsec %= 1000000000u; }
        return *this;
    }

    uint64_t toNSec() const { return static_cast<uint64_t>(sec) * 1000000000ull + nsec; }

    static Time now() {
        double t = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        return Time().fromSec(t);
    }
};

class Duration {
public:
    double d = 0.0;
    Duration() = default;
    explicit Duration(double seconds) : d(seconds) {}
    double toSec() const { return d; }
};

class Rate {
public:
    explicit Rate(double hz) : period_(hz > 0 ? 1.0 / hz : 0.0) {}
    void sleep() const {
        if (period_ > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(period_));
        }
    }

private:
    double period_;
};

struct TimerEvent {
    Time current_real;
};

// Timers never fire in the shim (FAST-LIVO2 only uses one for the optional
// imu-rate propagation, disabled by default; the glue can call the callback
// itself if that mode is ever needed).
class Timer {};

class Subscriber {};

class Publisher {
public:
    Publisher() = default;
    explicit Publisher(std::string topic) : topic_(std::move(topic)) {}

    template <typename M>
    void publish(const M& msg) const {
        auto& hooks = dimos_shim::publish_hooks();
        auto it = hooks.find(topic_);
        if (it != hooks.end()) { it->second(static_cast<const void*>(&msg)); }
    }

    const std::string& getTopic() const { return topic_; }

private:
    std::string topic_;
};

class NodeHandle {
public:
    template <typename T>
    void param(const std::string& name, T& out, const T& default_value) const {
        const auto& p = dimos_shim::params();
        auto it = p.find(name);
        if (it == p.end() || !dimos_shim::parse_value<T>(it->second, out)) {
            out = default_value;
        }
    }

    // Callbacks are invoked directly by the LCM glue; subscriptions are inert.
    template <typename M, typename T>
    Subscriber subscribe(const std::string& /*topic*/, uint32_t /*queue*/, void (T::* /*cb*/)(M), T* /*obj*/) {
        return Subscriber();
    }

    template <typename M>
    Publisher advertise(const std::string& topic, uint32_t /*queue*/) {
        return Publisher(topic);
    }

    template <typename T>
    Timer createTimer(Duration /*period*/, void (T::* /*cb*/)(const TimerEvent&), T* /*obj*/) {
        return Timer();
    }
};

inline void init(int /*argc*/, char** /*argv*/, const std::string& /*name*/) {}

inline bool ok() { return dimos_shim::running().load(); }

inline void spinOnce() {
    auto& fn = dimos_shim::spin_once_fn();
    if (fn) { fn(); }
}

}  // namespace ros

#define ROS_INFO(...)                                              \
    do {                                                           \
        if (dimos_shim::debug_logging()) {                         \
            fprintf(stderr, "[fastlivo info] " __VA_ARGS__);       \
            fprintf(stderr, "\n");                                 \
        }                                                          \
    } while (0)
#define ROS_WARN(...)                                          \
    do {                                                       \
        fprintf(stderr, "[fastlivo warn] " __VA_ARGS__);       \
        fprintf(stderr, "\n");                                 \
    } while (0)
#define ROS_ERROR(...)                                          \
    do {                                                        \
        fprintf(stderr, "[fastlivo error] " __VA_ARGS__);       \
        fprintf(stderr, "\n");                                  \
    } while (0)
#define ROS_ASSERT(cond)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            fprintf(stderr, "[fastlivo] assertion failed: %s (%s:%d)\n", #cond, \
                    __FILE__, __LINE__);                                        \
            abort();                                                            \
        }                                                                       \
    } while (0)

#endif  // DIMOS_ROS_SHIM_H
