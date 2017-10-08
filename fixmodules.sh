#!/bin/bash
SOURCE_PATH=~/ros_catkin_ws/install_isolated/lib/python2.7/site-packages
DEST_PATH=~/ros_catkin_ws/install_isolated/lib64/python2.7/site-packages

for dir in $SOURCE_PATH/*/ ; do

  dir_base="$(basename $dir)"

  echo "Copying $dir to $DEST_PATH"
  cp -r $dir $DEST_PATH
  echo "Removing $dir"
  rm -rf $dir
done


