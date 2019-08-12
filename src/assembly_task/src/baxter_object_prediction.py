#!/usr/bin/env python
import time
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn.externals import joblib
from scipy.cluster.vq import vq, kmeans, whiten
from object_recognition.msg import ObjectInfo

class webcam_image:
    def __init__(self):
        self._done = False
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        # baxter camera Subscriber
        self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)
        # Subscriber to determine whether or not to use vision
        self.is_moving_sub = rospy.Subscriber("/is_moving",Bool,self.check_moving)
        # Publisher
        self.object_location_pub = rospy.Publisher("/object_location",ObjectInfo,queue_size=10)


        self.object_info = ObjectInfo()
        self.object_info.names = ['','','']
        self.object_info.x = [0,0,0]
        self.object_info.y = [0,0,0]
        self.object_info.theta = [0,0,0]
        self.is_moving = False

    def check_moving(self,data):
        self.is_moving = data.data
        print (self.is_moving)

    def callback(self,data):
        if not self.is_moving:
            try:
                frame = self.bridge.imgmsg_to_cv2(data,"bgr8")
            except CvBridgeError as e:
                print("==[CAMERA MANAGER]==",e)

            global counter

            if counter > 4:
                counter = 0

            scale = 0.75
            frame = (frame*scale).astype(np.uint8)

            # Split frame into left and right halves
            left_frame = frame[0:frame.shape[0], 0:frame.shape[1]/3]
            middle_frame = frame[0:frame.shape[0], frame.shape[1]/3:2*frame.shape[1]/3]
            right_frame = frame[0:frame.shape[0], 2*frame.shape[1]/3:frame.shape[1]]

            # Create list of left and right images
            frames = [left_frame, middle_frame, right_frame]

            for i,f in enumerate(frames):
                kps, des = sift.detectAndCompute(f, None)
                frames[i] = cv2.drawKeypoints(f, kps, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) # draw SIFT points
                

                # Convert current index to one_hot list representation for color assignment later
                one_hot = [0,0,0]
                one_hot[i] = 1

                # Basic threshold example
                gray = cv2.cvtColor(f, cv2.COLOR_RGBA2GRAY)

                # CHANGE THESE VALUES TO CALIBRATE BOUNDING BOXES
                # th, dst = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
                dst = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 19, 5)
                
                # UNCOMMENT IF YOU WANT TO SEE OUTPUT OF OPENCV THRESHOLDING PROCESS
                # cv2.imshow('test' + str(i), dst)

                # Find contours of each partial frame and put bounding box around contour
                _, contours, hierarchy = cv2.findContours(dst,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                if len(contours) <= 0:
                    # print ("Waiting for something to grasp...")
                    rospy.logwarn_throttle(1, "Waiting for something to grasp...")
                else:
                    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:20]
                    cnt = contours[0]


                    # M = cv2.moments(cnt)
                    # self.object_info.x[i] = int(M["m10"]/M["m00"])
                    # self.object_info.y[i] = int(M["m01"]/M["m00"])

                    # cv2.circle(frames[i],(int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"])), 5, (255*one_hot[0],255*one_hot[1],255*one_hot[2]), -1)

                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    cv2.drawContours(frames[i],[np.int0(box)],0,(255*one_hot[0],255*one_hot[1],255*one_hot[2]),2)

                    # Calculate angle and center of each bounding box
                    if len(cnt) > 5:
                        (x,y),(MA,mA),angle = cv2.fitEllipse(cnt)
                        if x >= 0 and y >= 0:
                            self.object_info.x[i] = int(x) + frame.shape[1]/3 * i
                            self.object_info.y[i] = int(y)
                            self.object_info.theta[i] = np.deg2rad(angle)

                # Check to make sure des has elements and there are at least 15 keypoints
                if des is not None and len(kps) > 15:
                    test_features = np.zeros((1, k), "float32")
                    words, distance = vq(whiten(des), vocabulary)
                    for w in words:
                        if w >= 0 and w < 100:
                            test_features[0][w] += 1

                    # Scale the features
                    test_features = std_slr.transform(test_features)

                    # predictions based on classifier (more than 2)
                    predictions[i][counter] = [class_names[j] for j in classifier.predict(test_features)][0]
                    prediction = max(set(predictions[i]), key=predictions[i].count)

                    self.object_info.names[1] = prediction

                    # Find the point at the top of each bounding box to put label
                    labelY = int(min(box[0][1],box[1][1],box[2][1],box[3][1]))
                    labelX = [int(pt[0]) for pt in box if int(pt[1]) == labelY][0]

                    # Add label to partial frame
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(frames[i], prediction, (labelX,labelY-5), font, 0.5, (255*one_hot[0],255*one_hot[1],255*one_hot[2]), 1)

            # Combine smaller frames into one

            frame[0:frame.shape[0], frame.shape[1]/3:2*frame.shape[1]/3] = frames[1]


            # Add a dividing line down the middle of the frame
            cv2.line(frame, (frame.shape[1]/3,0), (frame.shape[1]/3,frame.shape[0]), (255,255,255), 1)
            cv2.line(frame, (2*frame.shape[1]/3,0), (2*frame.shape[1]/3,frame.shape[0]), (255,255,255), 1)

            # Resize image to fit monitor (if monitor is attached)
            if needResizing:
                frame = cv2.resize(frame, (1920, 1080), interpolation = cv2.INTER_CUBIC)

            counter += 1

            # Display the resulting frame
            cv2.imshow('Object recognition', frame)

            self.object_location_pub.publish(self.object_info)


            cv2.waitKey(1)
            
        if self.is_moving == True:
            # Debug terminal
            # print("---- IT'S MOVING -----")
            self._done = True
            # Debug terminal
            # print(self._done)
            # Calling shutdown
            rospy.signal_shutdown("ismoving")
            return
        
    def clean_shutdown(self):
        """Handles ROS shutdown (Ctrl-C) safely."""
        if not self._done:
            rospy.logwarn('Aborting: Shutting down safely...')


def main(args):
    rospy.init_node('webcam_image', anonymous=True)
    ic = webcam_image()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    rospy.on_shutdown(ic.clean_shutdown)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print "OpenCV Version:",cv2.__version__

    # Load the classifier, class names, scaler, number of clusters and vocabulary
    classifier, class_names, std_slr, k, vocabulary = joblib.load("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/dataset_RPLv2.pkl")

    # Create SIFT object
    sift = cv2.xfeatures2d.SIFT_create()

    predictions = [['','','','',''],['','','','',''],['','','','','']]
    counter = 0
    needResizing = False

    main(sys.argv)
