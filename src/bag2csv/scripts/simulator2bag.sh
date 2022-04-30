#!/bin/bash
cd ~
cd baykara_ws/bags

echo image_transport package is working
echo Please press Ctrl+C to start rosbag 
rosrun image_transport republish compressed in:=/camera raw out:=/camera

echo rosbag starting
rosbag record /camera /CanBusData -O sag1.bag