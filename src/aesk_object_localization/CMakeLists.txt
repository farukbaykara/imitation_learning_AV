cmake_minimum_required(VERSION 3.0.2)
project(aesk_object_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-Wall -Wextra)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
set(Open3D_DIR /usr/local/lib/cmake/Open3D/)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        #  pcl_conversions
        #  pcl_ros
        roscpp
        rospy
        sensor_msgs
        std_msgs
        darknet_ros_msgs
        OpenCV REQUIRED
        cv_bridge
        message_generation
        actionlib_msgs
        image_transport
        visualization_msgs
        actionlib
        Open3D
        dynamic_reconfigure
        vision_msgs
        tf2
        tf2_ros
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        PointCloud2WithId.msg
        ArrayOfPointCloud2s.msg
        #   Message1.msg
        #   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

add_action_files(
        DIRECTORY action
        FILES ProjectedCloud.action
        Park.action
        SignTracker.action
        DeleteSign.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        visualization_msgs
        actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        config/playground_params.cfg
        config/extrinsics.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES utils_core lidar_tracker_core
        #  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
        CATKIN_DEPENDS message_runtime
        actionlib_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/aesk_object_localization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/aesk_object_localization_node.cpp)

### CloudPainter
## Declare a cpp library
add_library(cloud_painter_core
        src/CloudPainter.cpp
        )
target_compile_features(cloud_painter_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(cloud_painter
        src/cloud_painter_node.cpp
        src/CloudPainter.cpp
        )
target_compile_features(cloud_painter INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(cloud_painter_core
        ${catkin_EXPORTED_TARGETS}
        )

add_dependencies(cloud_painter
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(cloud_painter_core
        ${catkin_LIBRARIES}
        )

target_link_libraries(cloud_painter
        cloud_painter_core
        ${catkin_LIBRARIES}
        )


### CloudFilter
## Declare a cpp library
#add_library(cloud_filter_core
#   src/CloudFilter.cpp
#)
#target_compile_features(cloud_filter_core INTERFACE cxx_std_11)
#
### Declare cpp executables
#add_executable(cloud_filter
#  src/cloud_filter_node.cpp
#  src/CloudFilter.cpp
#)
#target_compile_features(cloud_filter INTERFACE cxx_std_11)
#
### Add dependencies to exported targets, like ROS msgs or srvs
#add_dependencies(cloud_filter_core
#  ${catkin_EXPORTED_TARGETS}
#)
#
#add_dependencies(cloud_filter
#  ${catkin_EXPORTED_TARGETS}
#)
#
### Specify libraries to link executable targets against
#target_link_libraries(cloud_filter_core
#  ${catkin_LIBRARIES}
#)
#
#target_link_libraries(cloud_filter
#  cloud_filter_core
#  ${catkin_LIBRARIES}
#)

### ParkTracker
## Declare a cpp library
#add_library(park_tracker_core
#   src/ParkTracker.cpp
#)
#target_compile_features(park_tracker_core INTERFACE cxx_std_11)
#
### Declare cpp executables
#add_executable(park_tracker
#  src/park_tracker_node.cpp
#  src/ParkTracker.cpp
#)
#target_compile_features(park_tracker INTERFACE cxx_std_11)
#
### Add dependencies to exported targets, like ROS msgs or srvs
#add_dependencies(park_tracker_core
#  ${catkin_EXPORTED_TARGETS}
#)
#
#add_dependencies(park_tracker
#  ${catkin_EXPORTED_TARGETS}
#)
#
### Specify libraries to link executable targets against
#target_link_libraries(park_tracker_core
#  ${catkin_LIBRARIES}
#)
#
#target_link_libraries(park_tracker
#  park_tracker_core
#  ${catkin_LIBRARIES}
#)


### LidarTracker
## Declare a cpp library
add_library(lidar_tracker_core
        src/lidar_tracker/LidarTracker.cpp
        src/lidar_tracker/Object.cpp
        src/lidar_tracker/ObjectTracker.cpp)
target_compile_features(lidar_tracker_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(lidar_tracker
        src/lidar_tracker/lidar_tracker_node.cpp
        src/lidar_tracker/LidarTracker.cpp
        src/lidar_tracker/Object.cpp
        src/lidar_tracker/ObjectTracker.cpp
        )
target_compile_features(lidar_tracker INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(lidar_tracker_core
        ${catkin_EXPORTED_TARGETS}
        )

add_dependencies(lidar_tracker
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(lidar_tracker_core
        ${catkin_LIBRARIES}
        Open3D::Open3D
        utils_core
        )

target_link_libraries(lidar_tracker
        lidar_tracker_core
        ${catkin_LIBRARIES}
        )


### DarknetRepublish
## Declare a cpp library
#add_library(darknet_republish_core
#   src/DarknetRepublish.cpp
#)
#target_compile_features(darknet_republish_core INTERFACE cxx_std_11)
#
### Declare cpp executables
#add_executable(darknet_republish
#  src/darknet_republish_node.cpp
#  src/DarknetRepublish.cpp
#)
#target_compile_features(darknet_republish INTERFACE cxx_std_11)
#
### Add dependencies to exported targets, like ROS msgs or srvs
#add_dependencies(darknet_republish_core
#  ${catkin_EXPORTED_TARGETS}
#)
#
#add_dependencies(darknet_republish
#  ${catkin_EXPORTED_TARGETS}
#)
#
### Specify libraries to link executable targets against
#target_link_libraries(darknet_republish_core
#  ${catkin_LIBRARIES}
#)
#
#target_link_libraries(darknet_republish
#  darknet_republish_core
#  ${catkin_LIBRARIES}
#)


### DarknetRepublish
## Declare cpp executables
#add_executable(cloud_filter_mf
#  src/cloud_filter_node_mf.cpp
#)
#target_compile_features(cloud_filter_mf INTERFACE cxx_std_11)
#
#add_dependencies(cloud_filter_mf
#  ${catkin_EXPORTED_TARGETS}
#)
#
#target_link_libraries(cloud_filter_mf
#  ${catkin_LIBRARIES}
#)


### Playground
## Declare cpp executables
add_executable(playground
        src/playground/playground.cpp
        )
target_compile_features(playground INTERFACE cxx_std_11)

add_dependencies(playground
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(playground
        ${catkin_LIBRARIES}
        Open3D::Open3D
        )


### CameraDepthEstimator
## Declare a cpp library
add_library(camera_depth_estimator_core
        src/CameraDepthEstimator/CameraDepthEstimator.cpp
        )
target_compile_features(camera_depth_estimator_core INTERFACE cxx_std_11)
## Declare cpp executables
add_executable(camera_depth_estimator
        src/CameraDepthEstimator/camera_depth_estimator_node.cpp
        src/CameraDepthEstimator/CameraDepthEstimator.cpp
        )
target_compile_features(camera_depth_estimator INTERFACE cxx_std_11)
## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(camera_depth_estimator_core
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(camera_depth_estimator
        ${catkin_EXPORTED_TARGETS}
        )
## Specify libraries to link executable targets against
target_link_libraries(camera_depth_estimator_core
        ${catkin_LIBRARIES}
        )
target_link_libraries(camera_depth_estimator
        camera_depth_estimator_core
        ${catkin_LIBRARIES}
        )


## Declare a cpp library
add_library(utils_core
        src/PointCloudOpen3D.cpp
        )
target_compile_features(utils_core INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(utils_core
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(utils_core
        ${catkin_LIBRARIES}
        Open3D::Open3D
        )


## Declare a cpp library
add_library(pointcloud_ros_core
        src/PointCloudRos.cpp
        )
target_compile_features(pointcloud_ros_core INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(pointcloud_ros_core
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(pointcloud_ros_core
        ${catkin_LIBRARIES}
        Open3D::Open3D
        )


### roi_filter
## Declare a cpp library
add_library(roi_filter_core
        src/ROIFilter/ROIFilter.cpp
        )
target_compile_features(roi_filter_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(roi_filter
        src/ROIFilter/roi_filter_node.cpp
        src/ROIFilter/ROIFilter.cpp
        )
target_compile_features(roi_filter INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(roi_filter_core
        ${catkin_EXPORTED_TARGETS}
        )

add_dependencies(roi_filter
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(roi_filter_core
        ${catkin_LIBRARIES}
        Open3D::Open3D
        utils_core
        )

target_link_libraries(roi_filter
        roi_filter_core
        ${catkin_LIBRARIES}
        )


### lidar_cluster
## Declare a cpp library
add_library(lidar_cluster_core
        src/LidarCluster/LidarCluster.cpp
        )
target_compile_features(lidar_cluster_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(lidar_cluster
        src/LidarCluster/lidar_cluster_node.cpp
        src/LidarCluster/LidarCluster.cpp
        )
target_compile_features(lidar_cluster INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(lidar_cluster_core
        ${catkin_EXPORTED_TARGETS}
        )

add_dependencies(lidar_cluster
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(lidar_cluster_core
        ${catkin_LIBRARIES}
        Open3D::Open3D
        utils_core
        )

target_link_libraries(lidar_cluster
        lidar_cluster_core
        ${catkin_LIBRARIES}
        )


### sign_finder
## Declare a cpp library
add_library(sign_finder_core
        src/sign_finder/SignFinder.cpp
        )
target_compile_features(sign_finder_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(sign_finder
        src/sign_finder/sign_finder_node.cpp
        src/sign_finder/SignFinder.cpp
        )
target_compile_features(sign_finder INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(sign_finder_core
        ${catkin_EXPORTED_TARGETS}
        )

add_dependencies(sign_finder
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(sign_finder_core
        ${catkin_LIBRARIES}
        Open3D::Open3D
        utils_core
        lidar_tracker_core
        )

target_link_libraries(sign_finder
        sign_finder_core
        ${catkin_LIBRARIES}
        )


### sign_finder
## Declare a cpp library
add_library(lane3d_combiner_core
        src/lane3d_combiner/Lane3DCombiner.cpp
        )
target_compile_features(lane3d_combiner_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(lane3d_combiner
        src/lane3d_combiner/lane3d_combiner_node.cpp
        src/lane3d_combiner/Lane3DCombiner.cpp
        )
target_compile_features(lane3d_combiner INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(lane3d_combiner_core
        ${catkin_EXPORTED_TARGETS}
        )

add_dependencies(lane3d_combiner
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(lane3d_combiner_core
        ${catkin_LIBRARIES}
        )

target_link_libraries(lane3d_combiner
        lane3d_combiner_core
        ${catkin_LIBRARIES}
        )


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aesk_object_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
