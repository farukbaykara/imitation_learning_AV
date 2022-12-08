#!/bin/bash


source /opt/ros/noetic/setup.bash


cd ~
cd /media/aydin/data_64gb

rosbag record /zed/zed_node/left_raw/image_raw_color /zed/zed_node/left_raw/image_raw_color/compressed /zed/zed_node/odom /zed/zed_node/path_odom /zed/zed_node/pose /zed/zed_node/point_cloud/cloud_registered /vehicle_cmd


