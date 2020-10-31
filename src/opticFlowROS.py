#!/usr/bin/env python2
from __future__ import division
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from opticFlow import OpticFlow 
from camera import Camera
import rospy
import sys

try:
   from queue import Queue
except ImportError:
   import Queue as Queue


from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


NODE_NAME = 'optic_flow'


class OpticFlowROS():
   ##### TODO: change camerainfo to whatever it is
   def __init__(self, cam_0_topic='/resize_img/image', 
                cam_0_info="/resize_img/cameraInfo", 
                wait_for_imtopic_s=100):
      self.cam_0 = None
      self.cam_0_topic = cam_0_topic
      self.cam_0_info = cam_0_info
      self.image_queue = Queue()
      
      self.time_this_image = 0.0
      self.this_image = None
      self.initial_camera_time = None  # to avoid loosing information
      
      self.subscribers(wait_for_imtopic_s)
      
      self.OF_cam_0_module = OpticFlow(camera_instance=self.cam_0)
   
   # For now only one cam, add more
   def subscribers(self, wait_for_imtopic_s):
      """Subscribe to topics

      Args:
          wait_for_imtopic_s (int): number of seconds to wait ofr publishers
      """
      # Subscribe to the image topics
      self.cam_subs = rospy.Subscriber(
         self.cam_0_topic, Image, self.camera_0_cb, queue_size=5)
      
      self.cam_0_subs = rospy.Subscriber(
         self.cam_0_info, CameraInfo, self.cam_0_info_cb
      )
      
      try:
         rospy.loginfo('waiting for camera topics to be published')
         image_info_msg = rospy.wait_for_message(self.cam_0_info, CameraInfo, timeout=wait_for_imtopic_s)
         self.cam_frame_h = image_info_msg.height         # 120
         self.cam_frame_w = image_info_msg.width          # 16
         
         image_msg = rospy.wait_for_message(self.cam_0_topic, Image, timeout=wait_for_imtopic_s)

         rospy.loginfo('image encoding is {}'.format(image_msg.encoding))
         
         # If image is in black and white:
         if image_msg.encoding == "mono8":
               self.previous_image = np.zeros((self.cam_frame_h, self.cam_frame_w), dtype=np.uint8)
               
         # If it is in color:      
         else:
               rospy.logwarn('Using {} encoding - it is recommended to use mono8 for best efficiency'.format(image_msg.encoding))
               self.previous_image = np.zeros((self.cam_frame_h, self.cam_frame_w, 3), dtype=np.uint8)
         
      except Exception as e:
         rospy.logerr("{} Timed out waiting for camera topics {} in node {} ".format(e, self.cam_0_topic, rospy.get_name()))
         rospy.signal_shutdown('camera topics not detected shutting down node')
         sys.exit(1)
      
   def camera_0_cb(self, data):
      """Callback for the central camera

      Args:
          data (Image message): the callback data
      """
      self.camera_general_cb('cam_0', data)
      
   def cam_0_info_cb(self, data):
      """Add the camera_0 instance

      Args:
         data (CameraInfo): the camera info topic.
      """
      if not self.cam_0:
         self.cam_0 = Camera(data)
   
   def camera_general_cb(self, cam, data):
      """Callback for the camera topic. Add the image to an image queue.

      Args:
          cam (str): which camera (cam_0, cam_45, cam_n45)
          data (Image): image from the subscriber
      """
      try: 
         time_last_image = self.time_this_image
         # Get the image from the data
         self.this_image = bridge.imgmsg_to_cv2(data)
         # Update time
         self.time_this_image = data.header.stamp.to_sec()
         
         if time_last_image != self.time_this_image:
            # Add the image to the queue
            self.image_queue.put([self.this_image, self.time_this_image])
            
      # If there is a CvBridge error, print it      
      except CvBridgeError as err:
         print(err)
         
   def main(self):
      while not rospy.is_shutdown():
         if not self.image_queue.empty():
            this_image, this_image_time = self.image_queue.get()
            
            # if we don't subtract the initial camera time the frame difference can be 0.0 due to precision errors
            if not self.initial_camera_time:
               self.initial_camera_time = this_image_time
               
            this_image_time = this_image_time - self.initial_camera_time   
            self.OF_cam_0_module.step(this_image, this_image_time)
               
      
   
   
      
      
        
