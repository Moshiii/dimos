#!/bin/bash
# Source ROS, gate command-capable invocations, then run the dimos CLI.
#
# The gate lives in dimos.robot.galaxea.r1lite.entrypoint_gate (shipped
# in the wheel, deterministically tested): if any argument names a
# command-capable blueprint, regardless of option placement, every
# feedback stream in the arming contract must deliver a message within
# the wait window or the container exits nonzero. There is no
# launch-anyway path. Inert invocations (the sim blueprint, plain CLI
# commands) pass through ungated.
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"

if ! python3 -m dimos.robot.galaxea.r1lite.entrypoint_gate "$@"; then
    echo "[entrypoint] vendor stack gate failed; refusing to launch" >&2
    exit 1
fi

exec dimos "$@"
