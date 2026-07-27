#!/bin/bash
# Source ROS, then run the dimos CLI.
#
# Command-capable blueprints (anything that can publish actuator
# commands) fail closed: the vendor stack's /hdas/* feedback topics must
# be present and publishing within the wait window, or the container
# exits nonzero. There is no launch-anyway path for them. Blueprints not
# on the list (the sim variant, plain CLI commands) run ungated because
# they carry no actuator streams toward hardware.
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"

COMMAND_CAPABLE="r1lite-coordinator r1lite-keyboard-teleop r1lite-quest-teleop"

requires_vendor_stack() {
    local name
    for name in $COMMAND_CAPABLE; do
        [ "$1" = "$name" ] && return 0
    done
    return 1
}

if [ "$1" = "run" ] && requires_vendor_stack "$2"; then
    echo "[entrypoint] $2 is command-capable: requiring Galaxea /hdas/* feedback (domain $ROS_DOMAIN_ID)"
    found=""
    for i in $(seq 1 60); do
        if ros2 topic list 2>/dev/null | grep -q '^/hdas/'; then
            found=1
            break
        fi
        sleep 2
    done
    if [ -z "$found" ]; then
        echo "[entrypoint] FAIL: no /hdas topics after 120s; refusing to launch $2" >&2
        exit 1
    fi
    # Present is not enough: require at least one message on a required
    # feedback topic so a topic ghost from a dead node cannot pass.
    if ! timeout 10 ros2 topic echo --once /hdas/feedback_arm_left > /dev/null 2>&1; then
        echo "[entrypoint] FAIL: /hdas/feedback_arm_left present but silent; refusing to launch $2" >&2
        exit 1
    fi
fi

exec dimos "$@"
