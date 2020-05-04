#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# SETUP_SH="$DIR/../../../devel/setup.bash"

# source $SETUP_SH
# echo "Sourced $SETUP_SH"

. "$DIR/world_settings.sh" ${1:-0}
roslaunch bringup bringup.launch rviz:=true