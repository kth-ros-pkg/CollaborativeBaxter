#!/usr/bin/env python
import os
import subprocess

from std_msgs.msg import String, Bool

from cv_bridge import CvBridge
from cv2 import imread
from sensor_msgs.msg import Image

import numpy as np
from numpy import linalg as LA

import rospy
import threading

import smach
from smach import State, StateMachine, Concurrence

import smach_ros
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer

import std_srvs.srv

from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose

from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from math import sqrt

from object_recognition.msg import ObjectInfo

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

import tf

# Import scripts
# import request_action

img_untuckingArms = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/untucking_arms.png')
msg_untuckingArms = CvBridge().cv2_to_imgmsg(img_untuckingArms, encoding="bgr8")

img_tuckingArms = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/tucking_arms.png')
msg_tuckingArms = CvBridge().cv2_to_imgmsg(img_tuckingArms, encoding="bgr8")

img_goingHome = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/going_home.png')
msg_goingHome = CvBridge().cv2_to_imgmsg(img_goingHome, encoding="bgr8")

img_readyAssembly = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/ready_assembly_task.png')
msg_readyAssembly = CvBridge().cv2_to_imgmsg(img_readyAssembly, encoding="bgr8")

img_readyPickup = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/ready_pickup_object.png')
msg_readyPickup = CvBridge().cv2_to_imgmsg(img_readyPickup, encoding="bgr8")

img_movingOperator = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/moving_operator.png')
msg_movingOperator = CvBridge().cv2_to_imgmsg(img_movingOperator, encoding="bgr8")

img_movingObject = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/moving_object.png')
msg_movingObject = CvBridge().cv2_to_imgmsg(img_movingObject, encoding="bgr8")

img_sleeping = imread('/home/thib/CollaborativeBaxter/src/launch_demo/msg/sleeping.png')
msg_sleeping = CvBridge().cv2_to_imgmsg(img_sleeping, encoding="bgr8")


class RESET(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        print("Inside init state machine\n")
        print(rospy.get_name())

        # Wait
        rospy.sleep(2)

        rospy.loginfo("State machine initialized!")

        return 'success'

class UNTUCK(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
    def execute(self, userdata):
        print("Inside UNTUCK state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/baxter/baxter_tools/scripts/tuck_arms.py -u")

        # Baxter screen output
        image_pub.publish(msg_untuckingArms)
        
        # Wait
        rospy.sleep(1)
        
        # Call 'untuck_arms' script
        # os.system("/home/thib/CollaborativeBaxter/src/baxter/baxter_tools/scripts/tuck_arms.py -u")
        subprocess.check_call(["/home/thib/CollaborativeBaxter/src/baxter/baxter_tools/scripts/tuck_arms.py", "-u"])

        # Wait for termination
        rospy.sleep(1)
        
        return 'success'

class TUCK(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
    def execute(self, userdata):
        print("Inside TUCK state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/baxter/baxter_tools/scripts/tuck_arms.py -t")

        # Baxter screen output
        image_pub.publish(msg_tuckingArms)

        # Wait
        rospy.sleep(1)

        # Call 'tuck_arms' script
        # os.system("/home/thib/CollaborativeBaxter/src/baxter/baxter_tools/scripts/tuck_arms.py -t")
        subprocess.check_call(["/home/thib/CollaborativeBaxter/src/baxter/baxter_tools/scripts/tuck_arms.py", "-t"])

        # Wait for termination
        rospy.sleep(1)

        return 'success'

class GOINGHOME(State):
    def __init__(self):
        State.__init__(self, outcomes=['positionOK', 'fail'])

        
        rospy.loginfo("Waiting for move_base action server...")

    
    def execute(self, userdata):
        print("Inside GOINGHOME state machine\n")

        os.system("rosservice call /move_base/clear_costmaps '{}'")

        rospy.loginfo("Going to home location")

        # Baxter screen output
        image_pub.publish(msg_goingHome)
        
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(100)):
            rospy.logerr("Could not connect to /move_base_ac action server")
            exit()
        rospy.loginfo("Connected to move_base_ac action server")
        
        goal_pick = MoveBaseGoal()
        goal_pick.target_pose.header.frame_id = "map"
        goal_pick.target_pose.header.stamp = rospy.Time.now()
        goal_pick.target_pose.pose.position.x = 2.095
        goal_pick.target_pose.pose.position.y = -0.210
        goal_pick.target_pose.pose.position.z = 0.000
        goal_pick.target_pose.pose.orientation.x = 0.000
        goal_pick.target_pose.pose.orientation.y = 0.000
        goal_pick.target_pose.pose.orientation.z = 0.988
        goal_pick.target_pose.pose.orientation.w = 0.155

        self.move_base_ac.send_goal(goal_pick)
        wait = self.move_base_ac.wait_for_result(rospy.Duration(1000.0))

        if not wait:
            rospy.logerr("Robot stuck or not able to reach home pose!")
            return 'fail'
        else:
            rospy.loginfo("Home pose reached.")
            return 'positionOK'


class CHOOSEACTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['assemblyRequest', 'pickupRequest', 'tuckRequest', 'untuckRequest', 'goingHomeRequest', 'stop'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside CHOOSEACTION state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/launch_demo/src/request_action.py")
        
        # Call 'request_action' script
        # os.system("/home/thib/CollaborativeBaxter/src/launch_demo/src/request_action.py")
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/launch_demo/src/request_action.py") # Better way to call external python script
        
        # Wait for termination
        rospy.sleep(1)

        if self.actionRequested_str == 'q':
            print ('Stop requested')
            return 'stop'
        elif self.actionRequested_str == 'assembly':
            print ('Assembly requested')
            return 'assemblyRequest'
        elif self.actionRequested_str == 'pickup':
            print ('Pickup requested')
            return 'pickupRequest'
        elif self.actionRequested_str == 'tuck':
            print ('Tuck requested')
            return 'tuckRequest'
        elif self.actionRequested_str == 'untuck':
            print ('Untuck requested')
            return 'untuckRequest'
        elif self.actionRequested_str == 'home':
            print ('Going home requested')
            return 'goingHomeRequest'


# ASSEMBLY TASK
class MENUASSEMBLY(State):
    def __init__(self):
        State.__init__(self, outcomes=['requestOK', 'stop'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)
        
    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside MENUASSEMBLY state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/launch_demo/src/request_action.py")
        # os.system("/home//thib/CollaborativeBaxter/src/smach_baxter/src/request_object_SMACH.py")

        # Baxter screen output
        image_pub.publish(msg_readyAssembly)
        
        # Wait
        rospy.sleep(1)
        
        # Call 'request_object' script
        # os.system("/home//thib/CollaborativeBaxter/src/assembly_task/src/request_object.py")
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/assembly_task/src/request_object.py")

        # Wait for termination
        rospy.sleep(1)

        if self.actionRequested_str == 'q':
            return 'stop'
        elif self.actionRequested_str == 'enclosure':
            return 'requestOK'

class MOVETOBOX(State):
    def __init__(self):
        State.__init__(self, outcomes=['positionOK', 'fail'])


        rospy.loginfo("Waiting for move_base action server...")

    
    def execute(self, userdata):
        print("Inside MOVETOBOX state machine\n")

        rospy.loginfo("Going to pick up location")
        
        # Baxter screen output
        image_pub.publish(msg_movingOperator)

        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(100)):
            rospy.logerr("Could not connect to /move_base_ac action server")
            exit()
        rospy.loginfo("Connected to move_base_ac action server")
        
        goal_pick = MoveBaseGoal()
        goal_pick.target_pose.header.frame_id = "map"
        goal_pick.target_pose.header.stamp = rospy.Time.now()
        goal_pick.target_pose.pose.position.x = -0.489
        goal_pick.target_pose.pose.position.y = 1.965
        goal_pick.target_pose.pose.position.z = 0.000
        goal_pick.target_pose.pose.orientation.x = 0.0
        goal_pick.target_pose.pose.orientation.y = 0.0
        goal_pick.target_pose.pose.orientation.z = 0.895
        goal_pick.target_pose.pose.orientation.w = 0.445

        self.move_base_ac.send_goal(goal_pick)
        wait = self.move_base_ac.wait_for_result(rospy.Duration(1000.0))

        if not wait:
            rospy.logerr("Robot stuck or not able to reach pick up pose!")
            return 'fail'
        else:
            rospy.loginfo("Pick up pose reached.")
            return 'positionOK'

class OBJECTPREDICTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['predictionOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside OBJECTPREDICTION state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/thib/CollaborativeBaxter/src/assembly_task/src/baxter_object_prediction.py")

        # Call 'baxter_object_prediction' script
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/assembly_task/src/baxter_object_prediction.py")

        # Wait for termination
        rospy.sleep(1)

        return 'predictionOK'

class ASSEMBLY(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside ASSEMBLY state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/thib/CollaborativeBaxter/src/assembly_task/src/pickup_box.py")

        # Call 'pickup_box' script
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/assembly_task/src/pickup_box.py")

        # Wait for termination
        rospy.sleep(1)

        return 'success'

# PICKUP TASK
class MENUPICKUP(State):
    def __init__(self):
        State.__init__(self, outcomes=['requestOK', 'stop'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)        

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)

    def execute(self, userdata):
        print("Inside MENUPICKUP state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/launch_demo/src/request_action.py")
        # os.system("/home/thib/CollaborativeBaxter/src/smach_baxter/src/request_object_SMACH.py")

        # Baxter screen output
        image_pub.publish(msg_readyPickup)

        # Wait
        rospy.sleep(1)

        # Call 'request_object' script
        # os.system("/home/thib/CollaborativeBaxter/src/object-recognition/src/baxter_demo/request_object.py")
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/object-recognition/src/baxter_demo/request_object.py") # TRY ROSRUN INSTEAD...

        # Wait for termination
        rospy.sleep(1)

        if self.actionRequested_str == 'q':
            return 'stop'
        elif self.actionRequested_str is not None:
            return 'requestOK'

class MOVETOOBJECT(State):
    def __init__(self):
        State.__init__(self, outcomes=['positionOK', 'fail'])

        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(100)):
            rospy.logerr("Could not connect to /move_base_ac action server")
            exit()
        rospy.loginfo("Connected to move_base_ac action server")
    
    def execute(self, userdata):
        print("Inside MOVETOOBJECT state machine\n")

        os.system("rosservice call /move_base/clear_costmaps '{}'")

        # Baxter screen output
        image_pub.publish(msg_movingObject)

        rospy.loginfo("Going to pick up location")
        goal_pick = MoveBaseGoal()
        goal_pick.target_pose.header.frame_id = "map"
        goal_pick.target_pose.header.stamp = rospy.Time.now()
        goal_pick.target_pose.pose.position.x = 0.502
        goal_pick.target_pose.pose.position.y = -0.638
        goal_pick.target_pose.pose.position.z = 0.000
        goal_pick.target_pose.pose.orientation.x = 0.000
        goal_pick.target_pose.pose.orientation.y = 0.000
        goal_pick.target_pose.pose.orientation.z = 0.936
        goal_pick.target_pose.pose.orientation.w = -0.352

        self.move_base_ac.send_goal(goal_pick)
        wait = self.move_base_ac.wait_for_result(rospy.Duration(1000.0))
        if not wait:
            rospy.logerr("Robot stuck or not able to reach pick up pose!")
            return 'fail'
        else:
            rospy.loginfo("Pick up pose reached.")
            return 'positionOK'

class PICKUPOBJECTPREDICTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['predictionOK', 'stop'])
    
    def execute(self, userdata):
        print("Inside OBJECTPREDICTION state machine\n")

        rospy.sleep(5.0)
        # pwd = os.getcwd()

        # Call 'baxter_object_prediction' script
        # os.system("/home/thib/CollaborativeBaxter/src/object-recognition/src/baxter_demo/baxter_object_prediction.py")
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/object-recognition/src/baxter_demo/baxter_object_prediction.py")

        # Wait for termination
        rospy.sleep(1)

        return 'predictionOK'

class PICKUP(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])

        # rospy.sleep(1)
        repeatDesiredObject_pub.publish('screwdriver') # Change this with previous subscriber as a global variable...

        
        
    def execute(self, userdata):
        print("Inside PICKUP state machine\n")

        # Call 'pickup_object' script
        # pwd = os.getcwd()
        # os.system(pwd + "/src/assembly_task/src/request_action.py")
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/object-recognition/src/baxter_demo/pickup_object.py")
        # os.system("rosrun object_recognition pickup_object.py")



        # rospy.sleep(1)

        os.system("rosnode kill joint_traj_action_srv") # Trick for erasing joint goals, not so clean... (launch file is respawning the server then)

        # Wait for termination
        rospy.sleep(1)

        return 'success'

class MOVETOOPERATOR(State):
    def __init__(self):
        State.__init__(self, outcomes=['positionOK', 'fail'])

        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(100)):
            rospy.logerr("Could not connect to /move_base_ac action server")
            exit()
        rospy.loginfo("Connected to move_base_ac action server")
    
    def execute(self, userdata):
        print("Inside MOVETOOBJECT state machine\n")

        os.system("rosservice call /move_base/clear_costmaps '{}'")

        rospy.loginfo("Going to operator location")

        # Baxter screen output
        image_pub.publish(msg_movingOperator)
        
        goal_pick = MoveBaseGoal()
        goal_pick.target_pose.header.frame_id = "map"
        goal_pick.target_pose.header.stamp = rospy.Time.now()
        goal_pick.target_pose.pose.position.x = -0.489
        goal_pick.target_pose.pose.position.y = 1.965
        goal_pick.target_pose.pose.position.z = 0.000
        goal_pick.target_pose.pose.orientation.x = 0.0
        goal_pick.target_pose.pose.orientation.y = 0.0
        goal_pick.target_pose.pose.orientation.z = 0.895
        goal_pick.target_pose.pose.orientation.w = 0.445

        self.move_base_ac.send_goal(goal_pick)
        wait = self.move_base_ac.wait_for_result(rospy.Duration(1000.0))
        if not wait:
            rospy.logerr("Robot stuck or not able to reach pick up pose!")
            return 'fail'
        else:
            rospy.loginfo("Pick up pose reached.")
            return 'positionOK'

class GIVEOBJECT(State):
    def __init__(self):
        State.__init__(self, outcomes=['objectGiven', 'stop'])
    
    def execute(self, userdata):
        print("Inside GIVEOBJECT state machine\n")

        # pwd = os.getcwd()

        # Call 'baxter_object_prediction' script
        # os.system("/home/thib/CollaborativeBaxter/src/object-recognition/src/baxter_demo/baxter_object_prediction.py")
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/object-recognition/src/baxter_demo/give_object.py")

        # Wait for termination
        rospy.sleep(1)

        return 'objectGiven'

# INSPECTION TASK
class INSPECTIONPREDICTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['predictionOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside INSPECTIONPREDICTION state machine\n")

        # Call 'baxter_screws_prediction' script
        # pwd = os.getcwd()
        # os.system(pwd + "/src/assembly_task/src/request_action.py")
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/inspection_task/src/baxter_screws_prediction.py")

        # Wait for termination
        rospy.sleep(1)

        return 'predictionOK'

class INSPECTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
    def execute(self, userdata):
        print("Inside INSPECTION state machine\n")
        
        # Call 'inspection' script
        # pwd = os.getcwd()
        subprocess.check_call("/home/thib/CollaborativeBaxter/src/inspection_task/src/inspection.py")

        os.system("rosnode kill joint_traj_action_srv") # Trick for erasing joint goals, not so clean... (launch file is respawning the server then)

        # Wait for termination
        rospy.sleep(1)

        return 'success'

if __name__ == "__main__":
    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    repeatDesiredObject_pub = rospy.Publisher('/desired_object', String, latch=True, queue_size=10)

    try:
        rospy.init_node('baxter_SMACH')
        
        # Create a SMACH state machine
        sm = StateMachine(outcomes=['success', 'fail', 'stop'])

        # Open the container (HERE IS DEFINED THE SM)
        with sm:
            # Reset the Baxter
            StateMachine.add('RESET', RESET(), transitions={'success':'CHOOSE_ACTION'})
            StateMachine.add('CHOOSE_ACTION', CHOOSEACTION(), transitions={'assemblyRequest':'MOVE_TOBOX', 'pickupRequest':'MENU_PICKUP', 'tuckRequest':'TUCK', 'untuckRequest':'UNTUCK', 'goingHomeRequest':'TUCK_BEFOREGOINGHOME', 'stop':'stop'})
            StateMachine.add('TUCK', TUCK(), transitions={'success':'CHOOSE_ACTION', 'stop':'stop'})
            StateMachine.add('UNTUCK', UNTUCK(), transitions={'success':'CHOOSE_ACTION', 'stop':'stop'})
            StateMachine.add('GOINGHOME', GOINGHOME(), transitions={'positionOK':'CHOOSE_ACTION', 'fail':'stop'})

            StateMachine.add('MENU_PICKUP', MENUPICKUP(), transitions={'requestOK':'MOVE_TOOBJECT', 'stop':'stop'})
            
           
            StateMachine.add('MOVE_TOOBJECT', MOVETOOBJECT(), transitions={'positionOK':'UNTUCK_AFTERMOVINGOBJECT', 'fail':'stop'})
            StateMachine.add('MOVE_TOBOX', MOVETOBOX(), transitions={'positionOK':'UNTUCK_AFTERMOVINGBOX', 'fail':'stop'})
            StateMachine.add('MOVE_TOOPERATOR', MOVETOOPERATOR(), transitions={'positionOK':'GIVE_OBJECT', 'fail':'stop'})

            StateMachine.add('UNTUCK_AFTERMOVINGOBJECT', UNTUCK(), transitions={'success':'PICKUP_ACTION', 'stop':'stop'})
            StateMachine.add('UNTUCK_AFTERMOVINGBOX', UNTUCK(), transitions={'success':'ASSEMBLY_ACTION', 'stop':'stop'})

            StateMachine.add('GIVE_OBJECT', GIVEOBJECT(), transitions={'objectGiven':'ASSEMBLY_ACTION', 'stop':'stop'})

            StateMachine.add('TUCK_BEFOREGOINGHOME', TUCK(), transitions={'success':'GOINGHOME', 'stop':'stop'})


            # Concurrence allow to execute two parallel states

            # PICKUP TASK
            sm_pickup = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'PICKUPOBJECT_PREDICTION':'predictionOK', 'PICKUP':'success'}, 'fail':{'PICKUPOBJECT_PREDICTION':'stop', 'PICKUP':'stop'}})

            with sm_pickup:
                # Concurrence.add('MENU_PICKUP', MENUPICKUP())
                Concurrence.add('PICKUPOBJECT_PREDICTION', PICKUPOBJECTPREDICTION())
                Concurrence.add('PICKUP', PICKUP())
                
            StateMachine.add('PICKUP_ACTION', sm_pickup, transitions={'success':'MOVE_TOOPERATOR', 'fail':'stop'})  


            # ASSEMBLY TASK
            sm_assembly = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'OBJECT_PREDICTION':'predictionOK', 'ASSEMBLY':'success', 'MENU_ASSEMBLY':'requestOK'}})

            with sm_assembly:
                Concurrence.add('MENU_ASSEMBLY', MENUASSEMBLY())
                Concurrence.add('OBJECT_PREDICTION', OBJECTPREDICTION())
                Concurrence.add('ASSEMBLY', ASSEMBLY())
                
            StateMachine.add('ASSEMBLY_ACTION', sm_assembly, transitions={'success':'INSPECTION_ACTION', 'fail':'stop'})

   

            # INSPECTION TASK
            sm_inspection = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'INSPECTION_PREDICTION':'predictionOK', 'INSPECTION':'success'}})

            with sm_inspection:
                Concurrence.add('INSPECTION_PREDICTION', INSPECTIONPREDICTION())
                Concurrence.add('INSPECTION', INSPECTION())
                
            StateMachine.add('INSPECTION_ACTION', sm_inspection, transitions={'success':'CHOOSE_ACTION', 'fail':'stop'})           
        
        # Attach a SMACH introspection server
        sis = IntrospectionServer('baxter_SMACH_introspection', sm, '/BAXTER_DEMO')
        sis.start()

        # Set preempt handler
        smach_ros.set_preempt_handler(sm)

        # Execute SMACH tree in a separate thread so that we can ctrl-c the script
        smach_thread = threading.Thread(target = sm.execute)
        smach_thread.start()

        # Signal handler (wait for CTRL+C)
        rospy.spin()

    
    except rospy.ROSInterruptException:

        # Baxter screen output
        image_pub.publish(msg_sleeping)

        # Wait
        rospy.sleep(1)

        rospy.signal_shutdown()
        pass
    




