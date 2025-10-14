#!/bin/bash

case $1 in
  start)
    gnome-terminal \
      --tab --title="livox_ros_driver2" --command="bash -c 'roslaunch livox_ros_driver2 msg_MID360_group.launch; exec bash'" \
      --tab --title="livox_merge"       --command="bash -c 'roslaunch livox_merge livox_merge.launch; exec bash'" \
      --tab --title="vanjee_lidar_sdk"  --command="bash -c 'roslaunch vanjee_lidar_sdk dual_716mini.launch; exec bash'"\
      --tab --title="fast_lio"          --command="bash -c 'roslaunch fast_lio localization_avia.launch; exec bash'" \
      --tab --title="dispatch_control"  --command="bash -c 'rosrun dispatch_control dispatch_node.py; exec bash'" \
      --tab --title="ntu_planner"       --command="bash -c 'roslaunch ntu_planner move_base.launch; exec bash'"
    ;;
  stop)
    echo "Killing all related ROS processes..."
    pkill -f "roslaunch livox_ros_driver2"
    pkill -f "roslaunch livox_merge"
    pkill -f "vanjee_lidar_sdk"
    pkill -f "roslaunch fast_lio"
    pkill -f "rosrun dispatch_control"
    pkill -f "roslaunch ntu_planner"
    ;;
  *)
    echo "Usage: $0 {start|stop}"
    ;;
esac

