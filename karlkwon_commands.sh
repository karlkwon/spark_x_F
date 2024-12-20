#!/bin/bash

# https://shanelonergan.github.io/streamline-your-workflow-with-custom-bash-commands/
function tb3() {
  tmp_path=~/turtlebot3_ws/install/setup.bash
  echo $tmp_path
  source $tmp_path
}

function tb3_bringup() {
  tb3
  echo 'hardware.launch !!'
  ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
}

function tb3_servo() {
  tb3
  echo 'servo.launch !!'
  ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
}
