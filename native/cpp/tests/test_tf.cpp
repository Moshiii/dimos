// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <cmath>
#include <optional>
#include <string>

#include "dimos/native/tf.hpp"

using namespace dimos::native;

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d pose(double x, double y, double z, double yaw) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    iso.translation() = Eigen::Vector3d(x, y, z);
    return iso;
}

// atan2 of the rotated x axis rather than eulerAngles, which wraps a negative
// yaw into an equivalent triple with a flipped pitch.
double yaw_of(const Transform& t) {
    Eigen::Matrix3d r = t.iso.rotation();
    return std::atan2(r(1, 0), r(0, 0));
}

void add(MultiTBuffer& buffer, const std::string& parent, const std::string& child, double ts,
         double x, double y, double z, double yaw) {
    buffer.receive(parent, child, ts, pose(x, y, z, yaw));
}

std::optional<Transform> latest(const MultiTBuffer& buffer, const std::string& parent,
                                const std::string& child) {
    return buffer.get(parent, child, std::nullopt, std::nullopt);
}

}  // namespace

TEST_CASE("a direct edge resolves") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "base_link", "arm", 1.0, 1.0, -1.0, 0.0, 0.0);

    std::optional<Transform> t = latest(buffer, "base_link", "arm");
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(1.0));
    CHECK(t->translation().y() == doctest::Approx(-1.0));
    CHECK(t->parent == "base_link");
    CHECK(t->child == "arm");
}

// Inverse of a rotated edge is t' = -R^T t, the classic sign/order trap.
TEST_CASE("a reverse edge inverts rotation and translation") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "base_link", "arm", 1.0, 1.0, 2.0, 3.0, kPi / 2.0);

    std::optional<Transform> inv = latest(buffer, "arm", "base_link");
    REQUIRE(inv.has_value());
    CHECK(inv->translation().x() == doctest::Approx(-2.0));
    CHECK(inv->translation().y() == doctest::Approx(1.0));
    CHECK(inv->translation().z() == doctest::Approx(-3.0));
    CHECK(yaw_of(*inv) == doctest::Approx(-kPi / 2.0));
    CHECK(inv->parent == "arm");
    CHECK(inv->child == "base_link");
}

TEST_CASE("composes the ROS example chain") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "base_link", "arm", 1.0, 1.0, -1.0, 0.0, kPi / 6.0);
    add(buffer, "arm", "end_effector", 1.0, 1.0, 1.0, 0.0, 0.0);

    std::optional<Transform> t = latest(buffer, "base_link", "end_effector");
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(1.366).epsilon(0.001));
    CHECK(t->translation().y() == doctest::Approx(0.366).epsilon(0.001));
    CHECK(t->parent == "base_link");
    CHECK(t->child == "end_effector");
}

TEST_CASE("composes a multi-hop chain") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "world", "robot", 1.0, 1.0, 2.0, 3.0, 0.0);
    add(buffer, "robot", "sensor", 1.0, 0.5, 0.0, 0.2, kPi / 2.0);

    std::optional<Transform> t = latest(buffer, "world", "sensor");
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(1.5));
    CHECK(t->translation().y() == doctest::Approx(2.0));
    CHECK(t->translation().z() == doctest::Approx(3.2));
}

TEST_CASE("a composed chain accumulates rotation") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "a", "b", 1.0, 0.0, 0.0, 0.0, kPi / 6.0);
    add(buffer, "b", "c", 1.0, 0.0, 0.0, 0.0, kPi / 6.0);

    std::optional<Transform> t = latest(buffer, "a", "c");
    REQUIRE(t.has_value());
    CHECK(yaw_of(*t) == doctest::Approx(kPi / 3.0));
}

TEST_CASE("a composed stamp is the stalest edge") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "world", "robot", 700.0, 1.0, 0.0, 0.0, 0.0);
    add(buffer, "robot", "sensor", 1000.0, 0.5, 0.0, 0.0, 0.0);

    REQUIRE(latest(buffer, "world", "sensor").has_value());
    CHECK(latest(buffer, "world", "sensor")->ts == doctest::Approx(700.0));
    // Both directions, so the answer does not depend on which end is queried.
    REQUIRE(latest(buffer, "sensor", "world").has_value());
    CHECK(latest(buffer, "sensor", "world")->ts == doctest::Approx(700.0));
}

// Two routes to d: three hops through b, c and two through x. BFS must compose
// the two-hop route.
TEST_CASE("BFS takes the fewest hops on a branching graph") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "a", "b", 1.0, 1.0, 0.0, 0.0, 0.0);
    add(buffer, "b", "c", 1.0, 1.0, 0.0, 0.0, 0.0);
    add(buffer, "c", "d", 1.0, 1.0, 0.0, 0.0, 0.0);
    add(buffer, "a", "x", 1.0, 10.0, 0.0, 0.0, 0.0);
    add(buffer, "x", "d", 1.0, 1.0, 0.0, 0.0, 0.0);

    std::optional<Transform> t = latest(buffer, "a", "d");
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(11.0));
}

TEST_CASE("a missing path returns nothing") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "world", "robot", 1.0, 1.0, 0.0, 0.0, 0.0);

    CHECK_FALSE(latest(buffer, "world", "unconnected").has_value());
}

TEST_CASE("the same frame resolves to identity") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);

    // No query time: identity is stamped now, not the epoch.
    std::optional<Transform> t = latest(buffer, "base_link", "base_link");
    REQUIRE(t.has_value());
    CHECK(t->translation().norm() == doctest::Approx(0.0));
    CHECK(t->ts > 0.0);

    // Explicit query time is echoed back.
    std::optional<Transform> at = buffer.get("base_link", "base_link", 42.0, std::nullopt);
    REQUIRE(at.has_value());
    CHECK(at->ts == doctest::Approx(42.0));
}

TEST_CASE("a time query picks the nearest sample") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "a", "b", 10.0, 1.0, 0.0, 0.0, 0.0);
    add(buffer, "a", "b", 20.0, 2.0, 0.0, 0.0, 0.0);

    std::optional<Transform> near_10 = buffer.get("a", "b", 11.0, std::nullopt);
    REQUIRE(near_10.has_value());
    CHECK(near_10->translation().x() == doctest::Approx(1.0));

    std::optional<Transform> near_20 = buffer.get("a", "b", 18.0, std::nullopt);
    REQUIRE(near_20.has_value());
    CHECK(near_20->translation().x() == doctest::Approx(2.0));
}

TEST_CASE("a tie prefers the later sample") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "a", "b", 10.0, 1.0, 0.0, 0.0, 0.0);
    add(buffer, "a", "b", 12.0, 2.0, 0.0, 0.0, 0.0);

    std::optional<Transform> t = buffer.get("a", "b", 11.0, std::nullopt);
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(2.0));
}

TEST_CASE("a time query outside the tolerance returns nothing") {
    MultiTBuffer buffer(kDefaultTfWindowSecs);
    add(buffer, "a", "b", 10.0, 1.0, 0.0, 0.0, 0.0);

    CHECK_FALSE(buffer.get("a", "b", 50.0, 1.0).has_value());
    CHECK(buffer.get("a", "b", 10.5, 1.0).has_value());
}

TEST_CASE("a time query without a tolerance falls back to the window") {
    MultiTBuffer buffer(10.0);
    add(buffer, "a", "b", 100.0, 1.0, 0.0, 0.0, 0.0);

    CHECK_FALSE(buffer.get("a", "b", 50.0, std::nullopt).has_value());

    std::optional<Transform> t = buffer.get("a", "b", 95.0, std::nullopt);
    REQUIRE(t.has_value());
    CHECK(t->translation().x() == doctest::Approx(1.0));
}

// An explicit tolerance is the caller opting into staleness, so it widens past
// the window rather than being clamped by it.
TEST_CASE("an explicit tolerance reaches past the window") {
    MultiTBuffer buffer(10.0);
    add(buffer, "a", "b", 100.0, 1.0, 0.0, 0.0, 0.0);

    CHECK(buffer.get("a", "b", 50.0, 60.0).has_value());
}

// The window bounds queries against a stamp, not the latest sample. With no
// requested time, the newest edge is returned however old it is.
TEST_CASE("latest is not bounded by the window") {
    MultiTBuffer buffer(10.0);
    add(buffer, "a", "b", 100.0, 1.0, 0.0, 0.0, 0.0);

    CHECK(latest(buffer, "a", "b").has_value());
}

TEST_CASE("samples outside the window are pruned") {
    TBuffer buffer(5.0);
    buffer.add(1.0, Eigen::Isometry3d::Identity());
    buffer.add(2.0, Eigen::Isometry3d::Identity());
    buffer.add(10.0, Eigen::Isometry3d::Identity());

    CHECK(buffer.size() == 1);
    REQUIRE(buffer.last() != nullptr);
    CHECK(buffer.last()->ts == doctest::Approx(10.0));
}

TEST_CASE("a late sample does not spare ones the window has aged out") {
    TBuffer buffer(5.0);
    buffer.add(10.0, Eigen::Isometry3d::Identity());
    buffer.add(11.0, Eigen::Isometry3d::Identity());
    // Late, but still inside the window.
    buffer.add(7.0, Eigen::Isometry3d::Identity());
    CHECK(buffer.size() == 3);

    buffer.add(20.0, Eigen::Isometry3d::Identity());
    CHECK(buffer.size() == 1);
    REQUIRE(buffer.last() != nullptr);
    CHECK(buffer.last()->ts == doctest::Approx(20.0));
}

TEST_CASE("a clock reset drops the pre-jump samples") {
    TBuffer buffer(5.0);
    for (int i = 0; i < 20; ++i) {
        buffer.add(1000.0 + i, Eigen::Isometry3d::Identity());
    }
    for (int i = 0; i < 20; ++i) {
        buffer.add(100.0 + i, Eigen::Isometry3d::Identity());
    }

    CHECK(buffer.size() <= 6);
    REQUIRE(buffer.last() != nullptr);
    CHECK(buffer.last()->ts == doctest::Approx(119.0));
}

TEST_CASE("adding out of order keeps samples sorted") {
    TBuffer buffer(kDefaultTfWindowSecs);
    buffer.add(3.0, Eigen::Isometry3d::Identity());
    buffer.add(1.0, Eigen::Isometry3d::Identity());
    buffer.add(2.0, Eigen::Isometry3d::Identity());

    REQUIRE(buffer.last() != nullptr);
    CHECK(buffer.last()->ts == doctest::Approx(3.0));
    const TBuffer::Sample* s = buffer.find_closest(1.9, std::nullopt);
    REQUIRE(s != nullptr);
    CHECK(s->ts == doctest::Approx(2.0));
}
