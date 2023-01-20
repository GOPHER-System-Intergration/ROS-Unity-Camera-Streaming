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
import socket  
import math

def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  #pub = rospy.Publisher('video_frames', CompressedImage, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('udpwebcam_pub', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(rospy.get_param('/udpwebcam_pub/fps')) # 10hz
  

  
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  device_id = rospy.get_param('/udpwebcam_pub/device_id')
  udp_port = rospy.get_param('udpwebcam_pub/udp_port')
  udp_ip = rospy.get_param('udpwebcam_pub/udp_ip')
  chunk_size = rospy.get_param('udpwebcam_pub/chunk_size')

  cap = cv2.VideoCapture(device_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH,rospy.get_param('/udpwebcam_pub/input_width'));
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT,rospy.get_param('/udpwebcam_pub/input_height'));   
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
  sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM)
  # While ROS is still running.
  frame_num = 0
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()
         
      if ret == True:
        # Print debugging information to the terminal
        #rospy.loginfo('publishing video frame')
        #msg = CompressedImage()
        #msg.header.stamp = rospy.Time.now()
        #msg.format = "jpeg"
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), rospy.get_param('/udpwebcam_pub/jpg_quality')]
        
        data_string = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tostring()
        frame_size = len(data_string)
        #chunk_size = 4000
        current_ind = 0
        current_chunk = 0;
        max_chunks = math.ceil(frame_size/chunk_size)
        while current_ind < frame_size:
            payload_size = chunk_size
            if current_ind + chunk_size > frame_size :
                payload_size = frame_size - current_ind
            payload = data_string[current_ind:current_ind+payload_size]
            #print(payload_size)
            header = str(frame_num) + "_" + str(current_chunk) + "_" + str(max_chunks) + "_" + str(current_ind) + "_" + str(payload_size) + "_" +  str(frame_size)
            packet = bytes(header, "utf-8") + bytes(1) + payload
            sock.sendto(packet, (udp_ip, udp_port))
            current_chunk = current_chunk +1
            current_ind = current_ind + chunk_size
            print(header)
        print("")
        #print(data_string[0])
        #print(data_string[len(data_string)-1])
        #print(len(data_string))

        #sock.sendto(data_string, (udp_ip, udp_port))

        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        #pub.publish(msg)
             
      # Sleep just enough to maintain the desired rate
      frame_num = frame_num + 1
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
