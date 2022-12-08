#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import pyzed.sl as sl
import math
import numpy as np
import sys
import cv2 
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

class ImageProjector:

    def __init__(self):


        self.raw_image = Image()
        self.cloud_Zed = PointCloud2()


        # self.image_sub = rospy.Subscriber('/zed/zed_node/left_raw/image_raw_color',Image,self.imageCallback)
        # self.cloud_sub = rospy.Subscriber('/zed/zed_node/point_cloud/cloud_registered',PointCloud2,self.cloudCallback)
        self.image_raw_pub = rospy.Publisher("/zed/image_raw",Image,queue_size=10)
        self.cloud_zed_pub = rospy.Publisher("/zed/point_cloud",PointCloud2,queue_size=10)
        
        self.wp_sub = rospy.Subscriber('/waypoint_topic_2d', PoseStamped,
                        self.waypointCallback)

        


        

        print("Image to 3D node created")


    def waypointCallback(self,data):
        print("waypoitn callback girdi")

        ref_point = PoseStamped()

        wp_3d = PoseStamped()

        ref_point = data
        # print(ref_point.pose.position.x)
        # print(ref_point.pose.position.y)
    

    def RunZED(self):
         # Create a Camera object
        zed = sl.Camera()

        

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
        init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        init_params.camera_resolution = sl.RESOLUTION.HD720

        # Open the camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Create and set RuntimeParameters after opening the camera
        runtime_parameters = sl.RuntimeParameters()
        runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        # Setting the depth confidence parameters
        runtime_parameters.confidence_threshold = 100
        runtime_parameters.textureness_confidence_threshold = 100

        # Capture 150 images and depth, then stop
        i = 0
        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()

        mirror_ref = sl.Transform()
        mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
        tr_np = mirror_ref.m

        while(1):
            # A new image is available if grab() returns SUCCESS
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image
                zed.retrieve_image(image, sl.VIEW.LEFT)
                # Retrieve depth map. Depth is aligned on the left image
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

                # Get and print distance value in mm at the center of the image
                # We measure the distance camera - object using Euclidean distance
                x = round(image.get_width() / 2)
                y = round(image.get_height() / 2)
                err, point_cloud_value = point_cloud.get_value(x, y)

                # cv2.circle(image,(x,y),radius=1,color=(255,0,255))
                # cv2.imshow(image)

                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                        point_cloud_value[1] * point_cloud_value[1] +
                                        point_cloud_value[2] * point_cloud_value[2])

                point_cloud_np = point_cloud.get_data()
                point_cloud_np.dot(tr_np)

                if not np.isnan(distance) and not np.isinf(distance):
                    print("Distance to Camera at ({}, {}) (image center): {:1.3} m".format(x, y, distance), end="\r")
                    # Increment the loop


                    print(point_cloud_value[0] ,point_cloud_value[1],point_cloud_value[2] )

                else:
                    print("Can't estimate distance at this position.")
                    print("Your camera is probably too close to the scene, please move it backwards.\n")
                sys.stdout.flush()

        # Close the camera
        zed.close()




if __name__ == '__main__':

    rospy.init_node('image2world', anonymous=False)
    obj = ImageProjector()
    obj.RunZED()
    rospy.spin()


