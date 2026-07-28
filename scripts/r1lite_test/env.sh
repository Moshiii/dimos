# Source this in every dev-container shell before running dimos or the
# preflight tool on the R1 Lite:
#
#     source scripts/r1lite_test/env.sh
#
# Sets up the venv, ROS, the vendor DDS domain, and the UDP-only FastDDS
# profile a root container needs to hear the uid-1000 vendor stack.
_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_repo="$(cd "$_here/../.." && pwd)"

if [ -f "$_repo/.venv/bin/activate" ]; then
    source "$_repo/.venv/bin/activate"
else
    echo "WARNING: $_repo/.venv missing; run: UV_PYTHON=3.10 uv sync --no-default-groups" >&2
fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export FASTRTPS_DEFAULT_PROFILES_FILE="$_here/fastdds_udp_only.xml"
echo "r1lite env ready: domain=$ROS_DOMAIN_ID profile=$FASTRTPS_DEFAULT_PROFILES_FILE"
