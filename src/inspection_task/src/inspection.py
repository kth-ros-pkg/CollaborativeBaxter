#!/usr/bin/env python
from grip_node_left import GripperClient as GripperClientLeft
from grip_node_right import GripperClient as GripperClientRight

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
from std_msgs.msg import Bool, String, Float32, UInt32
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState, DigitalIOState, NavigatorState, DigitalOutputCommand
from object_recognition.msg import ObjectInfo
import planning_node_left as pnodeLeft
import planning_node_right as pnodeRight
import baxter_interface
import tf
import sys

# Initialization
camera_state_info = None
pixel_info = None
camera_info = None
object_location = None
object_orientation = None
object_name = None
desired_object = None
screwsDetected = 0

# screwingOK = False
# inspectionOK = False

img_gettingReady = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/msg/ready_inspection.png')
msg_gettingReady = CvBridge().cv2_to_imgmsg(img_gettingReady, encoding="bgr8")

img_inspectionRunning = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/msg/inspection_running.png')
msg_inspectionRunning = CvBridge().cv2_to_imgmsg(img_inspectionRunning, encoding="bgr8")

img_inspectionCompleted = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/msg/inspection_completed.png')
msg_inspectionCompleted = CvBridge().cv2_to_imgmsg(img_inspectionCompleted, encoding="bgr8")

img_movingScrewing = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/msg/moving_screwing.png')
msg_movingScrewing = CvBridge().cv2_to_imgmsg(img_movingScrewing, encoding="bgr8")

img_confirmScrewing = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/msg/confirm_screwing.png')
msg_confirmScrewing = CvBridge().cv2_to_imgmsg(img_confirmScrewing, encoding="bgr8")

img_screwingCompleted = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/msg/screwing_completed.png')
msg_screwingCompleted = CvBridge().cv2_to_imgmsg(img_screwingCompleted, encoding="bgr8")

img_missingScrews = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/msg/missing_screws.png')
msg_missingScrews = CvBridge().cv2_to_imgmsg(img_missingScrews, encoding="bgr8")

img_placingBack = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/msg/placing_back.png')
msg_placingBack = CvBridge().cv2_to_imgmsg(img_placingBack, encoding="bgr8")

img_goingHome = cv2.imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/msg/going_home.png')
msg_goingHome = CvBridge().cv2_to_imgmsg(img_goingHome, encoding="bgr8")

dinspection = [0.631, -0.113, 0.292, -0.353, -0.276, -0.618, 0.646]
dinspectionCamera = [0.619, -0.137, 0.323, 0.634, 0.632, -0.348, 0.277]
dplace = [0.880, -0.170, 0.221, 0.390, 0.118, 0.848, -0.338]
dhome = [0.705, -0.327, 0.125, 0.999, 0.016, 0.008, 0.010]
dleftneutral = [0.580, 0.180, 0.099, 0.141, 0.990, 0.007, 0.022]

# Pre-recorded, can be adjusted by operator

zplaceback = -0.150 # CHANGE HEIGHT FOR PICKING UP

dplaceback = [0.705, -0.327, zplaceback+0.025, 0.999, 0.016, 0.008, 0.010]

neutral_pose = [0.581, -0.147, 0.098, -0.113, 0.992, 0.042, 0.021]

# THIS WORKS


def initCamera(data):
    global camera_info
    camera_info = data

# THIS WORKS


def getCameraState(data):
    global camera_state_info
    camera_state_info = data

# THIS WORKS

# Callback OK wheel button left navigator (WORKING)
def navigatorCallback(data):
    global navigatorOK_state
    navigatorOK_state = data.buttons[0]
    # print (navigatorOK_state)

def screwsDetectedCallback(data):
    global screwsDetected
    screwsDetected = data.data
    # print (screwsDetected)


# Action when object picked up
def inspection():
    screwsDetectedMem = UInt32(0)
    readyForInspection_pub.publish(False)
    gcleft = GripperClientLeft()

    # BAXTER SCREEN OUTPUT
    # image_pub.publish(msg_confirm)
    
    rospy.logwarn_throttle(1,"Getting ready for inspection!")

    # Display action on Baxter's screen
    image_pub.publish(msg_gettingReady)

    # Wait
    rospy.sleep(1)

    pnodeRight.initplannode(dinspection, "right")

    # Wait
    rospy.sleep(1)

    # gcleft.command(position=100.0, effort=0.0) # Open gripper
    # gcleft.wait()
    
    # # Wait
    # rospy.sleep(1)

    pnodeLeft.initplannode(dinspectionCamera, "left")

    gcleft.command(position=0.0, effort=0.0) # Close gripper
    gcleft.wait()

    rospy.logwarn_throttle(1,"Inspection in progress...")

    # Say everything is ok, start OpenCV inspection    
    readyForInspection_pub.publish(True)

    # CHANGE THIS
    # while navigatorOK_state != True:    
    #     # BLINKING BUTTON LIGHT
    #     leftInnerLight_pub.publish('left_inner_light', True)
    #     rospy.sleep(0.5)
    #     leftInnerLight_pub.publish('left_inner_light', False)
    #     rospy.sleep(0.5)
    #     print screwsDetected

    while screwsDetected == 0:
        rospy.logwarn_throttle(1, "Inspection running")
        # Display action on Baxter's screen
        # image_pub.publish(msg_inspectionRunning)
    

    if screwsDetected >= 4:
        #Debug terminal
        print "Inspection completed! 4/4 screws detected."
        
        # Wait
        rospy.sleep(2)
        # BAXTER SCREEN OUTPUT
        image_pub.publish(msg_inspectionCompleted)

        #Wait
        rospy.sleep(2)

        # Debug terminal
        print "Going back home"
        # Call function
        goingHome()

        rospy.signal_shutdown("End inspection") 

    else:
        readyForInspection_pub.publish(False)
        print ("Missing " + str(4 - screwsDetected) + " screw(s)! Inspection to be done again...")
        
        # Wait
        rospy.sleep(2)
        # BAXTER SCREEN OUTPUT
        image_pub.publish(msg_missingScrews)

        #Wait
        rospy.sleep(2)

        assemblyAgain()
    


    return

# THIS WORKS

def goingHome():
    print ("Inside goingHome function")

    gcright = GripperClientRight()
    gcleft = GripperClientLeft()

    # Debug terminal
    print ("Everything fine! Going home (and then placing back the box on the table)")

    # Display action on Baxter's screen
    image_pub.publish(msg_placingBack)   

    # Wait
    rospy.sleep(1)

    gcleft.command(position=100.0, effort=0.0) # Open left gripper
    gcleft.wait()
    
    # Wait
    rospy.sleep(1)


    # Move the left arm first (otherwise, collision)
    pnodeLeft.initplannode(dleftneutral, "left")

    print ("Placing back the box...")
    
    # Wait
    rospy.sleep(1)

    # Going home
    pnodeRight.initplannode(dhome, "right")

    # Wait
    rospy.sleep(1)

    pnodeRight.initplannode(dplaceback, "right")

    # Wait
    rospy.sleep(1)

    print ("Opening RIGHT gripper")
    gcright.command(position=100.0, effort=50.0) # Open gripper
    gcright.wait()
    print ("RIGHT gripper should be open now")

    # Display action on Baxter's screen
    image_pub.publish(msg_goingHome)   

    # Wait
    rospy.sleep(1)

    # Going home
    pnodeRight.initplannode(dhome, "right")

    # Wait
    rospy.sleep(1)

    # Going neutral pose
    pnodeRight.initplannode(neutral_pose, "right")   

    # Debug terminal
    print ("I'm back to home position. Terminating...")

    return

def assemblyAgain():
    print ("Inside assemblyAgain function")
    
    # Debug terminal
    print "Presenting again the object to the operator..."

    # Display action on Baxter's screen
    image_pub.publish(msg_movingScrewing)

    # Wait
    rospy.sleep(1)

    # Present object to operator to be assembled
    pnodeRight.initplannode(dplace, "right")

    # Wait for screwing
    screwing()

    # Wait
    rospy.sleep(1)

    # Reset screwsDetected publisher
    screwsDetected_pub.publish(0)

    # Inspection again
    inspection()

    return

def screwing():
    
    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_confirmScrewing)
    
    rospy.logwarn_throttle(1,"Waiting for the action to be completed...")

    while navigatorOK_state != True:    
        # BLINKING BUTTON LIGHT
        leftInnerLight_pub.publish('left_inner_light', True)
        rospy.sleep(0.5)
        leftInnerLight_pub.publish('left_inner_light', False)
        rospy.sleep(0.5)
        
    print "You completed the action."
    
    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_screwingCompleted)


    return

if __name__ == '__main__':
    rospy.init_node('inspection', log_level=rospy.INFO)

    

    print "Moving arm to correct location"
    # arm_setup()

    

    # ROS stuff
    rospy.Subscriber("/cameras/left_hand_camera/camera_info", CameraInfo, initCamera)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, getCameraState)

    rate = rospy.Rate(50)
    while (camera_info is None) or (camera_state_info is None):
        rate.sleep()

    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    is_moving_pub = rospy.Publisher("is_moving", Bool, queue_size=10)

    head_RedLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, queue_size=1)
    head_GreenLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, queue_size=1)

    leftInnerLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)
    # leftOuterLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)

    # object_location_pub = rospy.Publisher("object_location",ObjectInfo,queue_size=10)

    screwsDetected_pub = rospy.Publisher('/screwsDetected', UInt32, latch=True, queue_size=10) 

    readyForInspection_pub = rospy.Publisher('/inspectionReady', Bool, queue_size=10)

    # Buttons subscribers
    # rospy.Subscriber("/robot/digital_io/left_lower_button/state", DigitalIOState, buttonOKPress)
    rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, navigatorCallback)

    rospy.Subscriber("/screwsDetected", UInt32, screwsDetectedCallback)



    is_moving_pub.publish(False)


    # Debug terminal
    print "Ready to go!"

    inspection()

    rospy.spin()
