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
#
# The gate runs as a background child with signals forwarded: bash as
# pid 1 does not deliver signals to a foreground child, so a docker stop
# during the readiness wait would otherwise hang until the force kill.
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"

python3 -m dimos.robot.galaxea.r1lite.entrypoint_gate "$@" &
gate_pid=$!
trap 'kill -INT "$gate_pid" 2>/dev/null' INT TERM
gate_status=0
wait "$gate_pid" || gate_status=$?
trap - INT TERM
if [ "$gate_status" -ne 0 ]; then
    echo "[entrypoint] vendor stack gate failed; refusing to launch" >&2
    exit 1
fi

exec dimos "$@"
