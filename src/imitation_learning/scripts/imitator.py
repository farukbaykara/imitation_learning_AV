#! /usr/bin/env python3

from TestModel import *

from utlis import *
from tensorflow.python.keras.models import load_model



class Car():
    
    def __init__(self):
        
        self.img_path = rospy.get_param('~image_path')
        self.image_folder_name = rospy.get_param('~image_folder_name')
        self.model_name = rospy.get_param('~model_name')

        
        self.bridge = CvBridge()

        self.raw_image = None


        rospy.Subscriber(rospy.get_param('~image_topic'),Image,self.imageCallback,queue_size=1)
        
        self.cmdPublisher = rospy.Publisher('/imitator_cmd',VehicleCmd,queue_size=10)



    def imageCallback(self,msg):
        self.raw_image = self.bridge.imgmsg_to_cv2(msg)



    def preProcess(self,img):
        img = img[640:840, :, :]
        img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
        img = cv2.GaussianBlur(img,  (3, 3), 0)
        img = cv2.resize(img, (200, 66))
        img = img/255
        return img


    def sendCommand(self,steering,speed):
        cmdMsg = VehicleCmd()
        cmdMsg.twist_cmd.twist.angular.z = steering
        cmdMsg.twist_cmd.twist.linear.x = speed        
        
        self.cmdPublisher.publish(cmdMsg)



    def testModel(self):
        print('teste girdi')
        print(self.model_name)
        model = load_model(self.model_name)
        image = np.asarray(self.raw_image)
        image_processed = self.preProcess(image)
        image_pixel_array = np.array(image_processed)
        steering_cmd = float(model.predict(image_pixel_array))
        speed_cmd = 5
        self.sendCommand(steering_cmd,speed_cmd)
        



def main():

    rospy.init_node('imitator', anonymous=False)

    imitator = Car()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        imitator.testModel()
        rate.sleep()
        


if __name__=='__main__':

    main()