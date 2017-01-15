#!/bin/bash
ROS_WS=~/ros_catkin_ws
PATCH=build.patch

cp $PATCH $ROS_WS/
cd $ROS_WS
git apply $ROS_WS
rm -f $PATCH
