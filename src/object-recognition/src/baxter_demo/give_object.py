#!/usr/bin/env python
from grip_node import GripperClient
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState, DigitalIOState, NavigatorState, DigitalOutputCommand
from object_recognition.msg import ObjectInfo
import planning_node as pnode
import baxter_interface
import tf


img_take = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/take_object.png')
msg_take = CvBridge().cv2_to_imgmsg(img_take, encoding="bgr8")

img_confirm = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/confirm.png')
msg_confirm = CvBridge().cv2_to_imgmsg(img_confirm, encoding="bgr8")

img_enjoy = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/enjoy.png')
msg_enjoy = CvBridge().cv2_to_imgmsg(img_enjoy, encoding="bgr8")

img_untuckingArms = cv2.imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/untucking_arms.png')
msg_untuckingArms = CvBridge().cv2_to_imgmsg(img_untuckingArms, encoding="bgr8")

# Init
buttonOK_state = 0
navigatorOK_state = False

neutral_pose = [0.581, 0.182, 0.100, 0.139, 0.989, 0.009, 0.023]


# Callback OK cuff button (WORKING)
def buttonOKPress(data):
    global buttonOK_state
    buttonOK_state = data.state
    # print (buttonOK_state)

# Callback OK wheel button left navigator (WORKING)
def navigatorCallback(data):
    global navigatorOK_state
    navigatorOK_state = data.buttons[0]
    # print (navigatorOK_state)

def giveObject():

    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_untuckingArms)

    # Wait
    rospy.sleep(1)

    pnode.initplannode(neutral_pose, "left") # Node place object
    
    giveToOperator()

    confirmation()

    rospy.sleep(1)


    # gc.command(position=100.0, effort=50.0)
    # gc.wait()

    

    # Publish that Baxter has stopped moving
    is_moving_pub.publish(False)
    # Reset desired_object to None
    desired_object = None

    # rospy.sleep(1)
    # if confirmation() == True:
    rospy.signal_shutdown("Give object OK")

    return

# Action when object picked up
def giveToOperator():
    
    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_take)

    while buttonOK_state != 1:
        rospy.logwarn_throttle(1,"Waiting for you to take the object...")
        

    # rospy.logwarn_throttle(1,"OK Button pressed!")
    
    print "Gripper opened, you can take the object now..."

def confirmation():

    print "Please confirm you took the object!"

    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_confirm)
    
    rospy.logwarn_throttle(1,"Waiting for confirmation!")

    while navigatorOK_state != True:    
        # BLINKING BUTTON LIGHT
        leftInnerLight_pub.publish('left_inner_light', True)
        rospy.sleep(0.5)
        leftInnerLight_pub.publish('left_inner_light',False)
        rospy.sleep(0.5)
        
    print "You took the object. Enjoy!"
    
    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_enjoy)

    return True



if __name__ == '__main__':
    rospy.init_node('give_object', log_level=rospy.INFO)

    # ROS stuff
    rate = rospy.Rate(50)

    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    is_moving_pub = rospy.Publisher("/is_moving", Bool, queue_size=10)

    head_RedLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, queue_size=1)
    head_GreenLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, queue_size=1)

    leftInnerLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)
    # leftOuterLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)

    # Buttons subscribers
    rospy.Subscriber("/robot/digital_io/left_lower_button/state", DigitalIOState, buttonOKPress)
    rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, navigatorCallback)

    is_moving_pub.publish(False)
    
    # Debug terminal
    print "Ready to give to operator!"

    giveObject()

    rospy.spin()
