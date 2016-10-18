#!/bin/bash
SOURCE_PATH=~/ros_catkin_ws/install_isolated/lib/python2.7/site-packages
DEST_PATH=~/ros_catkin_ws/install_isolated/lib64/python2.7/site-packages

for dir in $SOURCE_PATH/*/ ; do

    dir_base="$(basename $dir)"

    if [ -d $DEST_PATH/$dir_base ]; then
    	mv $dir/* $DEST_PATH/$dir_base
    	rm -rf $dir
    else
    	mv $dir $DEST_PATH/
    fi
done


