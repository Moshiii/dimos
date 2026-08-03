// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Wire codec between the transform graph and tf2_msgs.TFMessage, plus the port
// wiring for it. Schema, not transport: any Transport carries these bytes.
// Split from tf.hpp because naming TFMessage pulls in the dimos-lcm generated
// headers and liblcm, which the rest of the SDK core does not need.

#pragma once

#include <geometry_msgs/TransformStamped.hpp>
#include <tf2_msgs/TFMessage.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dimos/native/lcm_codec.hpp"
#include "dimos/native/log.hpp"
#include "dimos/native/module.hpp"
#include "dimos/native/tf.hpp"
#include "dimos/native/transport.hpp"

namespace dimos::native {

/// Encode one transform as a stamped LCM transform.
inline geometry_msgs::TransformStamped to_stamped(const Transform& t) {
    double sec = std::floor(t.ts);
    double nsec = std::round((t.ts - sec) * 1e9);
    // Rounding 0.9999999999 up lands a full second in the nsec field.
    if (nsec >= 1e9) {
        sec += 1.0;
        nsec -= 1e9;
    }

    Eigen::Vector3d p = t.translation();
    Eigen::Quaterniond q = t.rotation();

    geometry_msgs::TransformStamped st;
    st.header.seq = 0;
    st.header.stamp.sec = static_cast<std::int32_t>(sec);
    st.header.stamp.nsec = static_cast<std::int32_t>(nsec);
    st.header.frame_id = t.parent;
    st.child_frame_id = t.child;
    st.transform.translation.x = p.x();
    st.transform.translation.y = p.y();
    st.transform.translation.z = p.z();
    st.transform.rotation.x = q.x();
    st.transform.rotation.y = q.y();
    st.transform.rotation.z = q.z();
    st.transform.rotation.w = q.w();
    return st;
}

/// Encode transforms as one TFMessage.
inline tf2_msgs::TFMessage to_tf_message(const std::vector<Transform>& transforms) {
    tf2_msgs::TFMessage msg;
    msg.transforms.reserve(transforms.size());
    for (const Transform& t : transforms) {
        msg.transforms.push_back(to_stamped(t));
    }
    // lcm-gen encodes the array from this field, not from the vector's size.
    msg.transforms_length = static_cast<std::int32_t>(msg.transforms.size());
    return msg;
}

/// A dispatch that decodes TFMessage traffic into `graph`.
///
/// Meant for Builder::raw_input, so it runs on the receive thread and the graph
/// stays current while the module's dispatch loop is busy.
inline Dispatch tf_dispatch(std::string topic, std::shared_ptr<Graph> graph) {
    return [topic = std::move(topic), graph = std::move(graph)](const uint8_t* data,
                                                                std::size_t len) {
        tf2_msgs::TFMessage msg;
        try {
            msg = lcm_decode<tf2_msgs::TFMessage>(data, len);
        } catch (const std::exception& e) {
            DIMOS_ERROR_THROTTLED(log::from_secs(1), "tf decode error",
                                  log::Field("topic", topic),
                                  log::Field("error", std::string(e.what())));
            return;
        }
        graph->update([&msg, &topic](MultiTBuffer& buffer) {
            for (const geometry_msgs::TransformStamped& st : msg.transforms) {
                const geometry_msgs::Quaternion& q = st.transform.rotation;
                Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
                // Normalizing a zero-norm quaternion yields a NaN rotation.
                if (rotation.norm() < 1e-9) {
                    DIMOS_ERROR_THROTTLED(log::from_secs(1),
                                          "tf rotation is not a valid quaternion",
                                          log::Field("topic", topic),
                                          log::Field("parent", st.header.frame_id),
                                          log::Field("child", st.child_frame_id));
                    continue;
                }
                rotation.normalize();

                const geometry_msgs::Vector3& p = st.transform.translation;
                Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
                iso.linear() = rotation.toRotationMatrix();
                iso.translation() = Eigen::Vector3d(p.x, p.y, p.z);
                double ts = static_cast<double>(st.header.stamp.sec) +
                            static_cast<double>(st.header.stamp.nsec) * 1e-9;
                buffer.receive(st.header.frame_id, st.child_frame_id, ts, iso);
            }
        });
    };
}

/// A handle that answers transform queries and publishes on the `tf` port.
///
/// The graph fills in the background as tf messages arrive. Repeated calls
/// share one graph and subscribe the topic once.
inline Tf make_tf(Builder& builder) {
    std::optional<Tf>& cached = builder.tf_handle();
    if (cached.has_value()) {
        return *cached;
    }
    auto graph = std::make_shared<Graph>(kDefaultTfWindowSecs);
    Output<tf2_msgs::TFMessage> out = builder.output<tf2_msgs::TFMessage>("tf");
    builder.raw_input("tf", tf_dispatch(builder.topic_for("tf"), graph));
    cached = Tf(std::move(graph), [out](const std::vector<Transform>& transforms) {
        out.publish(to_tf_message(transforms));
    });
    return *cached;
}

}  // namespace dimos::native
