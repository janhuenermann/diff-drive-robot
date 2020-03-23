ps aux | grep gazebo | awk '{print $2}' | while read line ; do
    echo "Killed Gazebo $line"
    kill -9 $line
done

export PROJ_GROUND_CELL_SIZE="0.05"

case $1 in
    1)
        export PROJ_WORLD="world1"
        export PROJ_GROUND_GOALS="-4.0,-6.0|-2.5,-1.0|-9.0,1.5"
        export PROJ_GROUND_X_POS="-8.9"
        export PROJ_GROUND_Y_POS="-8.7"
        export PROJ_GROUND_MIN_POS="-11.0,-11.0"
        export PROJ_GROUND_MAX_POS="11.0,11.0"
        ;;
    2)
        export PROJ_WORLD="world2"
        export PROJ_GROUND_GOALS="2.0,0.0|3.0,-4.0|-4.0,-6.0"
        export PROJ_GROUND_X_POS="-2.0"
        export PROJ_GROUND_Y_POS="-0.5"
        export PROJ_GROUND_MIN_POS="-7.0,-11.0"
        export PROJ_GROUND_MAX_POS="7.0,4.0"
        ;;
    3)
        export PROJ_WORLD="world3"
        export PROJ_GROUND_GOALS="3.0,-4.0|-1.0,-5.0|-0.5,-9.5"
        export PROJ_GROUND_X_POS="-2.0"
        export PROJ_GROUND_Y_POS="-0.5"
        export PROJ_GROUND_MIN_POS="-6.0,-12.0"
        export PROJ_GROUND_MAX_POS="6.0,6.0"
        ;;
    0)
        export PROJ_WORLD="turtlebot3_world"
        export PROJ_GROUND_GOALS="0.5,0.5|1.5,0.5|-1.5,-1.5|-2.0,-0.5"
        export PROJ_GROUND_X_POS="-2.0"
        export PROJ_GROUND_Y_POS="-0.5"
        export PROJ_GROUND_MIN_POS="-4.0,-4.0"
        export PROJ_GROUND_MAX_POS="4.0,4.0"
        ;;
esac