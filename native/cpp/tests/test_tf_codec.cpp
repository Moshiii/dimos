// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dimos/native/tf_codec.hpp"

using namespace dimos::native;
using Bytes = std::vector<uint8_t>;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct MockTransport : Transport {
    std::mutex m;
    std::vector<std::pair<std::string, Bytes>> published;
    std::unordered_map<std::string, Dispatch> subs;

    void publish(const std::string& channel, Bytes data) override {
        std::lock_guard<std::mutex> lock(m);
        published.emplace_back(channel, std::move(data));
    }
    void subscribe(const std::string& channel, Dispatch on_msg) override {
        std::lock_guard<std::mutex> lock(m);
        subs[channel] = std::move(on_msg);
    }

    void deliver(const std::string& channel, const Bytes& bytes) {
        Dispatch cb;
        {
            std::lock_guard<std::mutex> lock(m);
            cb = subs.at(channel);
        }
        cb(bytes.data(), bytes.size());
    }
    std::size_t publish_count() {
        std::lock_guard<std::mutex> lock(m);
        return published.size();
    }
};

Eigen::Isometry3d pose(double x, double y, double z, double yaw) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    iso.translation() = Eigen::Vector3d(x, y, z);
    return iso;
}

double yaw_of(const Transform& t) {
    Eigen::Matrix3d r = t.iso.rotation();
    return std::atan2(r(1, 0), r(0, 0));
}

// One transform on the wire, with the rotation given as a raw quaternion so a
// test can put an invalid one there.
Bytes wire_message(const std::string& parent, const std::string& child, std::int32_t sec,
                   std::int32_t nsec, double x, double qx, double qy, double qz, double qw) {
    geometry_msgs::TransformStamped st;
    st.header.seq = 0;
    st.header.stamp.sec = sec;
    st.header.stamp.nsec = nsec;
    st.header.frame_id = parent;
    st.child_frame_id = child;
    st.transform.translation.x = x;
    st.transform.translation.y = 0.0;
    st.transform.translation.z = 0.0;
    st.transform.rotation.x = qx;
    st.transform.rotation.y = qy;
    st.transform.rotation.z = qz;
    st.transform.rotation.w = qw;

    tf2_msgs::TFMessage msg;
    msg.transforms.push_back(st);
    msg.transforms_length = 1;
    return lcm_encode(msg);
}

template <class F>
bool wait_until(F cond, std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!cond()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

// RAII publish workers, so a failing CHECK cannot strand a spinning thread.
struct WorkerGuard {
    Builder& builder;
    std::vector<std::thread> workers;

    WorkerGuard(Builder& b, Transport& t) : builder(b) {
        for (const auto& queue : builder.publish_queues()) {
            workers.emplace_back(publish_worker_loop, queue.get(), &t);
        }
    }
    ~WorkerGuard() {
        for (const auto& queue : builder.publish_queues()) {
            queue->stop();
        }
        for (std::thread& w : workers) {
            w.join();
        }
    }
};

}  // namespace

TEST_CASE("a tf message decodes into the graph") {
    auto graph = std::make_shared<Graph>(kDefaultTfWindowSecs);
    Dispatch dispatch = tf_dispatch("/tf", graph);

    Bytes bytes = wire_message("base_link", "mid360_link", 5, 500000000, 0.1, 0.0, 0.0, 0.0, 1.0);
    dispatch(bytes.data(), bytes.size());

    std::optional<Transform> t =
        graph->get("base_link", "mid360_link", std::nullopt, std::nullopt);
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(0.1));
    CHECK(t->ts == doctest::Approx(5.5));
}

TEST_CASE("a zero rotation on the wire is dropped rather than stored as NaN") {
    auto graph = std::make_shared<Graph>(kDefaultTfWindowSecs);
    Dispatch dispatch = tf_dispatch("/tf", graph);

    Bytes invalid = wire_message("a", "b", 5, 0, 1.0, 0.0, 0.0, 0.0, 0.0);
    dispatch(invalid.data(), invalid.size());
    CHECK_FALSE(graph->get("a", "b", std::nullopt, std::nullopt).has_value());

    Bytes valid = wire_message("a", "b", 6, 0, 1.0, 0.0, 0.0, 0.0, 1.0);
    dispatch(valid.data(), valid.size());
    std::optional<Transform> t = graph->get("a", "b", std::nullopt, std::nullopt);
    REQUIRE(t.has_value());
    CHECK(t->rotation().coeffs().allFinite());
}

TEST_CASE("undecodable bytes leave the graph empty") {
    auto graph = std::make_shared<Graph>(kDefaultTfWindowSecs);
    Dispatch dispatch = tf_dispatch("/tf", graph);

    Bytes garbage{'g', 'a', 'r', 'b', 'a', 'g', 'e'};
    dispatch(garbage.data(), garbage.size());

    CHECK_FALSE(graph->get("a", "b", std::nullopt, std::nullopt).has_value());
}

TEST_CASE("a published transform round-trips through the wire") {
    auto out_graph = std::make_shared<Graph>(kDefaultTfWindowSecs);
    Bytes encoded;
    Tf tf(out_graph, [&encoded](const std::vector<Transform>& transforms) {
        encoded = lcm_encode(to_tf_message(transforms));
    });

    tf.publish({Transform{"a", "b", 3.25, pose(0.5, -0.5, 0.25, kPi / 6.0)}});

    auto in_graph = std::make_shared<Graph>(kDefaultTfWindowSecs);
    Dispatch dispatch = tf_dispatch("/tf", in_graph);
    dispatch(encoded.data(), encoded.size());

    std::optional<Transform> t = in_graph->get("a", "b", std::nullopt, std::nullopt);
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(0.5));
    CHECK(t->translation().y() == doctest::Approx(-0.5));
    CHECK(t->translation().z() == doctest::Approx(0.25));
    CHECK(t->ts == doctest::Approx(3.25));
    CHECK(yaw_of(*t) == doctest::Approx(kPi / 6.0));
}

TEST_CASE("stamp rounding does not overflow nsec") {
    geometry_msgs::TransformStamped st =
        to_stamped(Transform{"a", "b", 1.9999999999, Eigen::Isometry3d::Identity()});

    CHECK(st.header.stamp.sec == 2);
    CHECK(st.header.stamp.nsec == 0);
}

TEST_CASE("make_tf publishes and receives on the mapped tf topic") {
    MockTransport transport;
    Notifier notifier;
    Builder builder({{"tf", "/robot/tf"}}, &notifier);
    Tf tf = make_tf(builder);

    for (const auto& route : builder.routes()) {
        transport.subscribe(route.first, route.second);
    }
    WorkerGuard workers(builder, transport);

    tf.publish({Transform{"map", "base_link", 4.0, pose(1.0, 0.0, 0.0, 0.0)}});
    REQUIRE(wait_until([&] { return transport.publish_count() == 1; }));
    CHECK(transport.published[0].first == "/robot/tf");

    // An edge only on the wire reaches the graph through the raw dispatch.
    transport.deliver("/robot/tf", wire_message("odom", "map", 9, 0, 2.0, 0.0, 0.0, 0.0, 1.0));
    std::optional<Transform> t = tf.get_latest("odom", "map");
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(2.0));
    CHECK(t->ts == doctest::Approx(9.0));
}

TEST_CASE("repeated make_tf calls share one graph and wire the topic once") {
    MockTransport transport;
    Notifier notifier;
    Builder builder({{"tf", "/tf"}}, &notifier);

    Tf first = make_tf(builder);
    Tf second = make_tf(builder);

    CHECK(builder.routes().size() == 1);
    CHECK(builder.publish_queues().size() == 1);

    // One graph: an edge delivered once is visible from either handle.
    for (const auto& route : builder.routes()) {
        transport.subscribe(route.first, route.second);
    }
    transport.deliver("/tf", wire_message("a", "b", 3, 0, 1.0, 0.0, 0.0, 0.0, 1.0));

    REQUIRE(first.get_latest("a", "b").has_value());
    REQUIRE(second.get_latest("a", "b").has_value());
    CHECK(second.get_latest("a", "b")->translation().x() == doctest::Approx(1.0));
}

TEST_CASE("tf dispatch bypasses the input queue") {
    MockTransport transport;
    Notifier notifier;
    Builder builder({{"tf", "/tf"}}, &notifier);
    Tf tf = make_tf(builder);

    // Nothing to drain: tf is not an InputChannel, so a busy dispatch loop
    // cannot delay it and a flood cannot overflow kInputQueueCapacity.
    CHECK(builder.input_ports().empty());

    for (const auto& route : builder.routes()) {
        transport.subscribe(route.first, route.second);
    }
    for (int i = 0; i < static_cast<int>(kInputQueueCapacity) + 10; ++i) {
        transport.deliver("/tf", wire_message("a", "b", i, 0, static_cast<double>(i), 0.0, 0.0,
                                              0.0, 1.0));
    }

    std::optional<Transform> t = tf.get_latest("a", "b");
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(kInputQueueCapacity + 9));
}
