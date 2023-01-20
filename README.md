# Imitation Driving Learning Autonomous Vehicle Project Repo
İmitation Learning Autonomous Driving Project repo

In this project, Lane following established with imitation learning (behavioral cloning). 
Obstacle detection made with KNN clustering algorithm on 2D Lidar point cloud. 
Object detection made with YOLOv4 algorithm. 
Pure Pursuit is used as control method. 

All packages created using ROS1 middleware and implemented in SVL simulator. 

roslaunch zed_wrapper zedm.launch

roslaunch rplidar_ros rplidar.launch

## Starting Vehicle

Go to your /home directory to install repo in it. It is important to be in **/home** directory.

```
cd /home
cd /imitation_learning_AV
source /devel/setup.bash
```
 Connect ZED camera and IMU to the Jetson TX2 and run below commands. 


```
sudo chmod 777 /dev/ttyUSB0
roslaunch ros2vehicle ros2vehicle.launch
```


