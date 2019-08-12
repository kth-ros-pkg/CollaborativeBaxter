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

import planning_node_left as pnodeLeft
import planning_node_right as pnodeRight

import baxter_interface
import tf

# Initialization
camera_state_info = None
pixel_info = None
camera_info = None
object_location = None
object_orientation = None
object_name = None
desired_object = None

img_screwdriver = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/pickup_screwdriver.png')
msg_screwdriver = CvBridge().cv2_to_imgmsg(img_screwdriver, encoding="bgr8")

img_marker = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/pickup_marker.png')
msg_marker = CvBridge().cv2_to_imgmsg(img_marker, encoding="bgr8")

# img_found = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/found_object.png')
# msg_found = CvBridge().cv2_to_imgmsg(img_found, encoding="bgr8")

img_take = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/take_object.png')
msg_take = CvBridge().cv2_to_imgmsg(img_take, encoding="bgr8")

img_confirm = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/confirm.png')
msg_confirm = CvBridge().cv2_to_imgmsg(img_confirm, encoding="bgr8")

img_enjoy = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/enjoy.png')
msg_enjoy = CvBridge().cv2_to_imgmsg(img_enjoy, encoding="bgr8")

img_errorRequest = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/error_request.png')
msg_errorRequest = CvBridge().cv2_to_imgmsg(img_errorRequest, encoding="bgr8")

img_placingArms = cv2.imread('/home/thib/CollaborativeBaxter/src/object-recognition/msg/placing_arms.png')
msg_placingArms = CvBridge().cv2_to_imgmsg(img_placingArms, encoding="bgr8")

# THIS WORKS


def initCamera(data):
    global camera_info
    camera_info = data

# THIS WORKS


def getCameraState(data):
    global camera_state_info
    camera_state_info = data


def getDesiredObject(data):
    global desired_object
    desired_object = data.data

# THIS WORKS


def getObjectLocation(data):
    global object_location
    global object_orientation
    global object_name
    global desired_object
    if desired_object is not None and desired_object in data.names:
        i = data.names.index(desired_object)
        object_name = data.names[i]
        x = data.x[i]
        y = data.y[i]
        object_location = [x, y]
        object_orientation = tf.transformations.quaternion_from_euler(
            0, 0, -data.theta[i])
        pickup()
    # elif desired_object is not None and desired_object not in data.names:
    #     print "That object is not one of the ones on the table. Please pick again."
    #     image_pub.publish(msg_errorRequest)
    #     desired_object = None


def pickup():
    print "------------- Object requested. -------------"
    global desired_object
    global buttonOK_state

    zsafe = 0.00333971663214
    zpick = -0.155  # CHANGE HEIGHT FOR PICKING UP

    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)
    gc = GripperClient()

    camera_x = camera_state_info.pose.position.x
    camera_y = camera_state_info.pose.position.y
    camera_z = camera_state_info.pose.position.z

    print "CAMERA X:", camera_x
    print "CAMERA Y:", camera_y
    print "CAMERA Z:", camera_z

    zoffset = -0.28  # table height in baxter's frame (see setup_notes.txt)
    pixel_size = .0025  # camera calibration (meter/pixels)
    h = camera_z-zoffset  # height from table to camera
    x0 = camera_x  # x camera position
    y0 = camera_y  # y camera position
    x_offset = -0.00  # offsets
    y_offset = -0.03
    height = 400  # image frame dimensions
    width = 640
    cx = object_location[0]
    cy = object_location[1]

    if object_name == 'eraser':
        cx -= 5

    elif object_name == 'screwdriver':
        cx += 0
        cy += 25

    # Convert pixel coordinates to baxter coordinates
    xb = (cy - (height/2))*pixel_size*h + x0 + x_offset
    yb = (cx - (width/2))*pixel_size*h + y0 + y_offset

    print "Object Location (pixels):", (cx, cy)
    print "Object Location (world):", (xb, yb)
    print "Object Orientation:", list(reversed(object_orientation))

    dsafe = [xb, yb, zsafe, 0.99, 0.01, 0.01, 0.01]
    dsafe_rotated = [xb, yb, zsafe, object_orientation[3],
                     object_orientation[2], object_orientation[1], object_orientation[0]]

    # EDIT THIS LINE FOR PLACE
    dplace = [camera_x, camera_y, zpick, object_orientation[3],
              object_orientation[2], object_orientation[1], object_orientation[0]]

    dpick = [xb, yb, zpick, object_orientation[3], object_orientation[2],
             object_orientation[1], object_orientation[0]]
    camera_rotated = [camera_x, camera_y, camera_z, object_orientation[3],
                      object_orientation[2], object_orientation[1], object_orientation[0]]
    initial = [camera_x, camera_y, camera_z, 0.99, 0.01, 0.01, 0.01]

    neutral_pose = [0.581, 0.182, 0.100, 0.139, 0.989, 0.009, 0.023]

    poseLeft_movebase = [-0.396, 0.637, -0.008, 0.101, 0.992, 0.071, 0.001] # Arm holding the screwdriver
    poseRight_movebase = [-0.444, -0.257, 0.115, 0.999, -0.025, -0.011, 0.021] # Free arm

    # Publish that Baxter is about to move
    is_moving_pub.publish(True)

    head_RedLed_pub.publish(100.0)
    head_GreenLed_pub.publish(15.0)

    # Display action on Baxter's screen
    if object_name == 'screwdriver':
        image_pub.publish(msg_screwdriver)
        rospy.sleep(1)
    
    if object_name == 'marker':
        image_pub.publish(msg_marker)
        rospy.sleep(1)

    # Debug terminal
    print "Let's pick up the object"

    pnodeLeft.initplannode(dsafe_rotated, "left")

    gc.command(position=100.0, effort=50.0)
    gc.wait()

    pnodeLeft.initplannode(dpick, "left")

    if object_name == 'eraser':
        gc.command(position=70.0, effort=50.0)
        gc.wait()
    elif object_name == 'marker':
        gc.command(position=5.0, effort=50.0)
        gc.wait()
    elif object_name == 'screwdriver':
        gc.command(position=5.0, effort=70.0)
        gc.wait()

    pnodeLeft.initplannode(dsafe_rotated, "left")

    # Debug terminal
    print "We picked up the object!"

    # BAXTER SCREEN OUTPUT
    # image_pub.publish(msg_found)
    
    # Wait
    rospy.sleep(1)

    # Debug terminal
    print "I got the object required!"

    # pnodeLeft.initplannode(dsafe_rotated, "left")
    # pnodeLeft.initplannode(initial, "left")
   

    # Placing arms correctly in order to move

    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_placingArms)

    pnodeLeft.initplannode(poseLeft_movebase, "left") # Node place object

    # Wait
    rospy.sleep(1.0)

    pnodeRight.initplannode(poseRight_movebase, "right") # Node place object

    # Wait
    rospy.sleep(1.0)   

    # ----- STOP SCRIPT HERE, MOVE TO GIVE THE OBJECT TO THE OPERATOR ------

    # Publish that Baxter has stopped moving
    is_moving_pub.publish(False)
    # Reset desired_object to None
    desired_object = None

    # rospy.sleep(1)
    # if confirmation() == True:
    rospy.signal_shutdown("Pickup OK")

    return

# THIS WORKS

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

# THIS WORKS


def left_arm_setup():
    # Get desired joint values from parameter server
    left_w0 = rospy.get_param('left_w0', default=0)
    left_w1 = rospy.get_param('left_w1', default=0)
    left_w2 = rospy.get_param('left_w2', default=0)
    left_e0 = rospy.get_param('left_e0', default=0)
    left_e1 = rospy.get_param('left_e1', default=0)
    left_s0 = rospy.get_param('left_s0', default=0)
    left_s1 = rospy.get_param('left_s1', default=0)

    # Send the left arm to the desired position
    home_left = {'left_w0': left_w0, 'left_w1': left_w1, 'left_w2': left_w2,
            'left_e0': left_e0, 'left_e1': left_e1, 'left_s0': left_s0, 'left_s1': left_s1}
    limb_left = baxter_interface.Limb('left')
    limb_left.move_to_joint_positions(home_left)


if __name__ == '__main__':
    rospy.init_node('pickup_object', log_level=rospy.INFO)

    print "Moving arm to correct location"
    left_arm_setup()

    # ROS stuff
    rospy.Subscriber("/cameras/left_hand_camera/camera_info", CameraInfo, initCamera)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, getCameraState)

    

    rospy.Subscriber("/desired_object", String, getDesiredObject)
    rospy.Subscriber("/object_location", ObjectInfo, getObjectLocation)

    rate = rospy.Rate(50)
    while (camera_info is None) or (camera_state_info is None):
        rate.sleep()

    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    is_moving_pub = rospy.Publisher("/is_moving", Bool, queue_size=10)

    head_RedLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, queue_size=1)
    head_GreenLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, queue_size=1)

    leftInnerLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)
    # leftOuterLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)

    object_location_pub = rospy.Publisher("/object_location",ObjectInfo,queue_size=10)

    # Buttons subscribers
    rospy.Subscriber("/robot/digital_io/left_lower_button/state", DigitalIOState, buttonOKPress)
    rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, navigatorCallback)

    is_moving_pub.publish(False)


    # Debug terminal
    print "Ready to go!"

    rospy.spin()
