#!/bin/bash
# In-image DDS discovery self-test: a pub/echo pair inside this container
# must discover each other and deliver one message. Proves the image's
# own DDS machinery independent of the vendor stack; the install wizard
# runs one instance in each of two containers for the cross-container
# variant. Exit 0 on delivery, 1 on silence.
# No `set -u`: the ROS setup file reads variables it may not have set.
source /opt/ros/humble/setup.bash
topic="/dimos_dds_selftest_$$"
timeout 25 ros2 topic pub "$topic" std_msgs/msg/String '{data: selftest}' -r 5 >/dev/null 2>&1 &
pub=$!
trap 'kill "$pub" 2>/dev/null' EXIT
if timeout 20 ros2 topic echo "$topic" std_msgs/msg/String --once >/dev/null 2>&1; then
    echo "dds-selftest: PASS (pub/echo discovered and delivered)"
    exit 0
fi
echo "dds-selftest: FAIL (no discovery/delivery inside the image)" >&2
exit 1
