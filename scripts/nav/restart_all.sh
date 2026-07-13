#!/usr/bin/env bash
# Compatibility entry point for a full paired restart.
#
# Keep exactly one restart implementation. bringup.sh owns the validated order:
# tear down navstack and Isaac, start Isaac, wait for the settled IMU marker,
# then start navstack/RViz and require unique, continuous ROS streams. All
# GO2W_* overrides remain in this process environment and are consumed there.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"

bash "$HERE/bringup.sh" teardown
exec bash "$HERE/bringup.sh" up
