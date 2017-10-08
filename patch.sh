#!/bin/bash

ROS_WS=~/ros_catkin_ws
PATCH_DIR=patches

cp $PATCH_DIR $ROS_WS/
cd $ROS_WS

for PATCH_FILE in $PATCH_DIR/* ; do
  echo "Applying $PATCH_FILE"
  patch -p0 < $PATCH_FILE
done

rm -rf $PATCH_DIR
cd -