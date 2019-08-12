#!/usr/bin/env python
from grip_node import GripperClient

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

# Initialization
camera_state_info = None
pixel_info = None
camera_info = None
object_location = None
object_orientation = None
object_name = None
desired_object = None
# screwingOK = False
# inspectionOK = False

img_box = cv2.imread('/home/thib/CollaborativeBaxter/src/assembly_task/msg/pickup_box.png')
msg_box = CvBridge().cv2_to_imgmsg(img_box, encoding="bgr8")

img_movingScrewing = cv2.imread('/home/thib/CollaborativeBaxter/src/assembly_task/msg/moving_screwing.png')
msg_movingScrewing = CvBridge().cv2_to_imgmsg(img_movingScrewing, encoding="bgr8")

img_confirmScrewing = cv2.imread('/home/thib/CollaborativeBaxter/src/assembly_task/msg/confirm_screwing.png')
msg_confirmScrewing = CvBridge().cv2_to_imgmsg(img_confirmScrewing, encoding="bgr8")

img_screwingCompleted = cv2.imread('/home/thib/CollaborativeBaxter/src/assembly_task/msg/screwing_completed.png')
msg_screwingCompleted = CvBridge().cv2_to_imgmsg(img_screwingCompleted, encoding="bgr8")

img_errorRequest = cv2.imread('/home/thib/CollaborativeBaxter/src/assembly_task/msg/error_request.png')
msg_errorRequest = CvBridge().cv2_to_imgmsg(img_errorRequest, encoding="bgr8")

dinspection = [0.690, -0.018, 0.167, 0.353, 0.187, 0.781, -0.479]
# dinspectionCamera = [0.529, 0.112, 0.155, 0.529, -0.472, 0.537, 0.457]

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
            0, 0, -((data.theta[i])+1.571)) # Add 90 degrees rotation
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
    zpick = -0.145 # CHANGE HEIGHT FOR PICKING UP

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

    if object_name == 'enclosure':
        cx += 15
        cy -= 7.5

    # Convert pixel coordinates to baxter coordinates
    xb = (cy - (height/2))*pixel_size*h + x0 + x_offset
    yb = (cx - (width/2))*pixel_size*h + y0 + y_offset

    print "Object Location (pixels):", (cx, cy)
    print "Object Location (world):", (xb, yb)
    print "Object Orientation:", list(reversed(object_orientation))

    dsafe = [xb, yb, zsafe, 0.99, 0.01, 0.01, 0.01]
    dsafe_rotated = [xb, yb, zsafe, object_orientation[3],
                     object_orientation[2], object_orientation[1], object_orientation[0]]

    # EDIT THIS LINE FOR PICK (IE GET THE BOX)
    dpick = [xb, yb, zpick, object_orientation[3], object_orientation[2], object_orientation[1], object_orientation[0]]


    # EDIT THIS LINE FOR PLACE (IE HOLD THE BOX TO BE ASSEMBLED)
    # dplace = [camera_x, camera_y, zpick, object_orientation[3],
    #           object_orientation[2], object_orientation[1], object_orientation[0]]

    dplace = [0.880, -0.170, 0.221, 0.390, 0.118, 0.848, -0.338]
    # Pre-recorded, can be adjusted by operator

    


    camera_rotated = [camera_x, camera_y, camera_z, object_orientation[3],
                      object_orientation[2], object_orientation[1], object_orientation[0]]
    initial = [camera_x, camera_y, camera_z, 0.99, 0.01, 0.01, 0.01]

    # Publish that Baxter is about to move
    is_moving_pub.publish(True)

    head_RedLed_pub.publish(100.0)
    head_GreenLed_pub.publish(15.0)

    # Display action on Baxter's screen
    image_pub.publish(msg_box)
    rospy.sleep(1) # USEFULL?
    
    # Debug terminal
    print "Let's pick up the box"

    pnode.initplannode(dsafe_rotated, "right")

    gc.command(position=100.0, effort=50.0) # Open gripper
    gc.wait()

    # EDIT HERE FOR PICKING STUFF
    pnode.initplannode(dpick, "right")

    if object_name == 'enclosure':
        gc.command(position=5.0, effort=50.0)
        gc.wait()

    pnode.initplannode(dsafe_rotated, "right")

    # Debug terminal
    print "We picked up the object!"

    # BAXTER SCREEN OUTPUT
    # image_pub.publish(msg_found)
    
    # Wait
    rospy.sleep(1)

    # Debug terminal
    print "I got the enclosure!"
   
    pnode.initplannode(dsafe_rotated, "right")
    pnode.initplannode(initial, "right")

    # Debug terminal
    print "Presenting the object to the operator..."

    # Display action on Baxter's screen
    image_pub.publish(msg_movingScrewing)

    # Present object to operator to be assembled
    pnode.initplannode(dplace, "right")

    # Wait for screwing
    screwing()

    # Wait
    # rospy.sleep(1)

    # Tell pickup ok



    # inspection()
    
    # Wait
    # rospy.sleep(1)

    # rospy.logwarn_throttle(1,"--- ISSUE AFTER SCREWING ---")

    # Go back home
    # pnode.initplannode(dsafe_rotated, "right")
    # pnode.initplannode(initial, "right")

    # Publish that Baxter has stopped moving
    is_moving_pub.publish(False)
    # Reset desired_object to None
    desired_object = None

    if screwing() == True:
        rospy.signal_shutdown("Assembly OK")
        rospy.on_shutdown(cv2.destroyAllWindows())
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


    return True

# Action when object picked up
# def inspection():
    
#     # BAXTER SCREEN OUTPUT
#     # image_pub.publish(msg_confirm)
    
#     rospy.logwarn_throttle(1,"Getting ready for inspection!")

#     pnode.initplannode(dinspection, "right")
#     # pnode.initplannode(dinspectionCamera, "left")

#     rospy.logwarn_throttle(1,"Inspection in progress...")

#     # CHANGE THIS
#     while navigatorOK_state != True:    
#         # BLINKING BUTTON LIGHT
#         leftInnerLight_pub.publish('left_inner_light', True)
#         rospy.sleep(0.5)
#         leftInnerLight_pub.publish('left_inner_light', False)
#         rospy.sleep(0.5)
        
#     print "Inspection completed!"
    
#     # BAXTER SCREEN OUTPUT
#     # image_pub.publish(msg_enjoy)

#     print "Going back home"

#     return

# THIS WORKS


def right_arm_setup():
    # Get desired joint values from parameter server
    right_w0 = rospy.get_param('right_w0', default=0)
    right_w1 = rospy.get_param('right_w1', default=0)
    right_w2 = rospy.get_param('right_w2', default=0)
    right_e0 = rospy.get_param('right_e0', default=0)
    right_e1 = rospy.get_param('right_e1', default=0)
    right_s0 = rospy.get_param('right_s0', default=0)
    right_s1 = rospy.get_param('right_s1', default=0)

    # Send the right arm to the desired position
    home_right = {'right_w0': right_w0, 'right_w1': right_w1, 'right_w2': right_w2,
            'right_e0': right_e0, 'right_e1': right_e1, 'right_s0': right_s0, 'right_s1': right_s1}
    limb_right = baxter_interface.Limb('right')
    limb_right.move_to_joint_positions(home_right)


if __name__ == '__main__':
    rospy.init_node('pickup_enclosure', log_level=rospy.INFO)

    print "Moving arm to correct location"
    right_arm_setup()

    # ROS stuff
    rospy.Subscriber("/cameras/right_hand_camera/camera_info", CameraInfo, initCamera)
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, getCameraState)

    

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
