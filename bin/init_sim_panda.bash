#!/bin/bash

SESSION=simulator

tmux -2 new-session -d -s $SESSION

tmux rename-window -t $SESSION:0 'robot'
tmux send-keys -t $SESSION:0 "roslaunch arm_gazebo arm_panda.launch world:=`rospack find arm_gazebo`/worlds/arm_empty.world  x:=-0.10 y:=0 z:=0.615"  C-m

sleep 15

# tmux new-window -t $SESSION:1 -n 'moveit'
# tmux send-keys -t $SESSION:1 "roslaunch panda_moveit_config demo.launch load_robot_description:=false  moveit_controller_manager:=simple use_rviz:=false"  C-m

# sleep 5

# tmux new-window -t $SESSION:2 -n 'rviz'
# tmux send-keys -t $SESSION:2 "roscd panda_moveit_config/launch" C-m
# tmux send-keys -t $SESSION:2 "rosrun rviz rviz -d moveit.rviz"


tmux new-window -t $SESSION:3 -n 'scripts'
tmux send-keys -t $SESSION:3 "roscd arm_gazebo/scripts" C-m
tmux send-keys -t $SESSION:3 "python gazebo_objects.py -a ../config/ARM0.txt" C-m

while [ 1 ]; do
  sleep 60
done

