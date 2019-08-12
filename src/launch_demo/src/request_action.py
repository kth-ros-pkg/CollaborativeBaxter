#!/usr/bin/env python
import sys
import rospy
import roslaunch
import cv2
from std_msgs.msg import String, Bool, Float32, UInt32
from sensor_msgs.msg import Image
from baxter_core_msgs.msg import EndpointState, DigitalIOState, NavigatorState, DigitalOutputCommand
from cv_bridge import CvBridge

is_moving = False

img_waiting = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/waiting.png')
msg_waiting = CvBridge().cv2_to_imgmsg(img_waiting, encoding="bgr8")

img_sleeping = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/sleeping.png')
msg_sleeping = CvBridge().cv2_to_imgmsg(img_sleeping, encoding="bgr8")

img_assemblyTask = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/assembly_task.png')
msg_assemblyTask = CvBridge().cv2_to_imgmsg(img_assemblyTask, encoding="bgr8")

img_inspectionTask = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/inspection_task.png')
msg_inspectionTask = CvBridge().cv2_to_imgmsg(img_inspectionTask, encoding="bgr8")

img_pickupObject = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/pickup_object.png')
msg_pickupObject = CvBridge().cv2_to_imgmsg(img_pickupObject, encoding="bgr8")

img_tuckArms = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/tuck_arms.png')
msg_tuckArms = CvBridge().cv2_to_imgmsg(img_tuckArms, encoding="bgr8")

img_untuckArms = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/untuck_arms.png')
msg_untuckArms = CvBridge().cv2_to_imgmsg(img_untuckArms, encoding="bgr8")

img_goHome = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/gotoHome.png')
msg_goHome = CvBridge().cv2_to_imgmsg(img_goHome, encoding="bgr8")

img_exit = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/exit.png')
msg_exit = CvBridge().cv2_to_imgmsg(img_exit, encoding="bgr8")

img_tuckingArms = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/tucking_arms.png')
msg_tuckingArms = CvBridge().cv2_to_imgmsg(img_tuckingArms, encoding="bgr8")

img_untuckingArms = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/untucking_arms.png')
msg_untuckingArms = CvBridge().cv2_to_imgmsg(img_untuckingArms, encoding="bgr8")

img_readyAssembly = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/ready_assembly_task.png')
msg_readyAssembly = CvBridge().cv2_to_imgmsg(img_readyAssembly, encoding="bgr8")

img_readyPickup = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/ready_pickup_object.png')
msg_readyPickup = CvBridge().cv2_to_imgmsg(img_readyPickup, encoding="bgr8")

def check_moving(data):
    global is_moving
    is_moving = data.data

# Callback OK wheel button left navigator (WORKING)
def navigatorCallback(data):
    global navigatorOK_state
    navigatorOK_state = data.buttons[0]
    # print (navigatorOK_state)

# Callback wheel index left navigator (WORKING)
def wheelIndexCallback(data):
    global wheelIndex_value
    wheelIndex_value = data.data
    # print (wheelIndex_value % 3)

def poll_object_request():
    while True:

        # image_pub.publish(msg_waiting)

        # Baxter's ready, all 'green'
        head_RedLed_pub.publish(0.0)
        head_GreenLed_pub.publish(100.0)

        # Light showing where user should interact with Baxter
        leftInnerLight_pub.publish('left_inner_light', True)

        # Wait
        rospy.sleep(0.25)
 
        # Debug terminal 
        # print wheelIndex_value % 3
           
        if (wheelIndex_value % 6) == 0 :
            # Debug terminal
            # print "Tuck arms"
            desired_object = "tuck"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_tuckArms)

        if (wheelIndex_value % 6) == 1 :
            # Debug terminal
            # print "Untuck arms"
            desired_object = "untuck"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_untuckArms)
        
        elif (wheelIndex_value % 6) == 2 :
            # Debug terminal
            # print "Assembly task"
            desired_object = "assembly"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_assemblyTask)

        elif (wheelIndex_value % 6) == 3 :
            # Debug terminal
            # print "Pickup task"
            desired_object = "pickup"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_pickupObject)

        elif (wheelIndex_value % 6) == 4 :
            # Debug terminal
            # print "Go to home"
            desired_object = "home"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_goHome)
        
        elif (wheelIndex_value % 6) == 5 :
            # Debug terminal
            # print "EXIT!"
            desired_object = "q"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_exit)



        # desired_object = raw_input('Enter the object you would like to pick up (q to quit): ')


        if navigatorOK_state == True:
            if desired_object == 'q':
                print "EXIT requested"
                image_pub.publish(msg_sleeping)
                desired_object_pub.publish(desired_object)
                leftInnerLight_pub.publish('left_inner_light', False)
                rospy.sleep(1)
                break
            
            elif desired_object == 'tuck':
                # Debug terminal
                # print "Finding and picking up ",desired_object
                # Switch off button light
                image_pub.publish(msg_tuckingArms)
                desired_object_pub.publish(desired_object)
                leftInnerLight_pub.publish('left_inner_light', False)
                rospy.sleep(1)
                break

            elif desired_object == 'untuck':
                # Debug terminal
                # print "Finding and picking up ",desired_object
                # Switch off button light
                image_pub.publish(msg_untuckingArms)
                desired_object_pub.publish(desired_object)
                leftInnerLight_pub.publish('left_inner_light', False)
                rospy.sleep(1)
                break          

            elif desired_object == 'home':
                # Debug terminal
                # Switch off button light
                # image_pub.publish(msg_readyPickup)
                leftInnerLight_pub.publish('left_inner_light', False)
                desired_object_pub.publish(desired_object)
                print "GOINGHOME requested"
                rospy.sleep(1)
                break


            elif desired_object == 'assembly':
                # Debug terminal
                # print "Finding and picking up ",desired_object
                # Switch off button light
                image_pub.publish(msg_readyAssembly)
                leftInnerLight_pub.publish('left_inner_light', False)
                desired_object_pub.publish(desired_object)
                print "ASSEMBLY requested"
                rospy.sleep(1)
                break

            elif desired_object == 'pickup':
                # Debug terminal
                # print "Finding and picking up ",desired_object
                # Switch off button light
                image_pub.publish(msg_readyPickup)
                leftInnerLight_pub.publish('left_inner_light', False)
                desired_object_pub.publish(desired_object)
                print "PICKUP requested"
                rospy.sleep(1)
                break

        while is_moving:
            pass

    print "Done!"

if __name__ == '__main__':
    rospy.init_node('request_object', log_level=rospy.INFO)

    rate = rospy.Rate(100)
    desired_object_pub = rospy.Publisher("/desired_object",String,queue_size=10)

    rospy.Subscriber("/is_moving",Bool,check_moving)
    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)

    head_RedLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, queue_size=1)
    head_GreenLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, queue_size=1)

    # Navigator subscribers
    rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, navigatorCallback) # OK Button
    rospy.Subscriber("/robot/analog_io/left_wheel/value_uint32", UInt32, wheelIndexCallback) # Wheel index

    leftInnerLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)

    poll_object_request()
