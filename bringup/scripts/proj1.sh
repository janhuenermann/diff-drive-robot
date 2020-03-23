#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
SETUP_SH="$DIR/../../../devel/setup.bash"

source $SETUP_SH
echo "Sourced $SETUP_SH"

# all environment variables MUST NOT contain spaces
# export GOALS="-4.0,-6.0|-2.5,-1.0|-9.0,1.5"
# export X_POS="-8.9"
# export Y_POS="-8.7"
# export WORLD="world1"
# export MIN_POS="-11.0,-11.0"
# export MAX_POS="11.0,11.0"

# ====== TURTLEBOT3_WORLD =========
# export GOALS="0.5,0.5|1.5,0.5|-1.5,-1.5|-2.0,-0.5"
# export X_POS="-2.0"
# export Y_POS="-0.5"
# export WORLD="turtlebot3_world"
# export MIN_POS="-4.0,-4.0"
# export MAX_POS="4.0,4.0"

# ====== WORLD1 =========
export GOALS="-4.0,-6.0|-2.5,-1.0|-9.0,1.5"
export X_POS="-8.9"
export Y_POS="-8.7"
export WORLD="world1"
export MIN_POS="-11.0,-11.0"
export MAX_POS="11.0,11.0"

# ====== WORLD2 =========
# export GOALS="2.0,0.0|3.0,-4.0|-4.0,-6.0"
# export X_POS="-2.0"
# export Y_POS="-0.5"
# export WORLD="world2"
# export MIN_POS="-7.0,-11.0"
# export MAX_POS="7.0,4.0"

# ====== WORLD3 =========
# export GOALS="3.0,-4.0|-1.0,-5.0|-0.5,-9.5"
# export X_POS="-2.0"
# export Y_POS="-0.5"
# export WORLD="world3"
# export MIN_POS="-6.0,-12.0"
# export MAX_POS="6.0,6.0"

export CELL_SIZE="0.05"
roslaunch bringup robot_bringup.launch
