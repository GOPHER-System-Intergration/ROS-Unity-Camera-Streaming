#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
import numpy as np
from ROS_TCP_Endpoint_msgs.msg import Image # Image is the message type
from ROS_TCP_Endpoint_msgs.msg import CompressedImage
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
  
def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('video_frames', CompressedImage, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('webcam_pub', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(rospy.get_param('/webcam_pub/fps')) # 10hz
  

  
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  device_id = rospy.get_param('/webcam_pub/device_id')
  cap = cv2.VideoCapture(device_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH,rospy.get_param('/webcam_pub/input_width'));
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT,rospy.get_param('/webcam_pub/input_height'));   
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()
         
      if ret == True:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), rospy.get_param('/webcam_pub/jpg_quality')]

        msg.data = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tostring()
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(msg)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
