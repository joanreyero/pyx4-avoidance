#!/usr/bin/env python2
from __future__ import division
import os
import numpy as np
from collections import deque
from sensor_msgs.msg import CameraInfo, Image
from matchedFilters import MatchedFilter
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from pyx4_avoidance.msg import flow as FlowMsg
from opticFlow import OpticFlow
from activation import get_activation
from avoidance_direction import get_direction
from obstacleFinder import ActivationDecisionMaker as DecisionMaker
from pyx4_avoidance.msg import activation as ActivationMsg
from pyx4.msg import pyx4_state
from pyx4_avoidance.msg import avoidancedecision as DecisionMsg
from pyx4_avoidance.msg import avoidancedirection as AvoidanceDirectionMsg
from camera_labels import *
from camera import Camera
import rospy
import sys

try:
   from queue import Queue
except ImportError:
   import Queue as Queue


from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


NODE_NAME = 'pyx4_avoidance_node'


class OpticFlowROS():
   
   def __init__(self, node_name,
                target_vel = 2,
                cam_0_topic='/resize_img/image', 
                cam_45_topic='/resize_img_45/image', 
                cam_n45_topic='/resize_img_n45/image', 
                cam_info="/resize_img/camera_info", 
                wait_for_imtopic_s=100,
                data_collection=False):
      
      self.node_name = node_name

      # Init empty cameras
      self.cam = None
      self.cam_info = cam_info

      # To iterate
      self.cam_iter = [C0, C45, CN45]

      self.cam_topics = {
         C0: cam_0_topic,
         C45: cam_45_topic,
         CN45: cam_n45_topic
      }

      self.image_queues = {
         C0: Queue(),
         C45: Queue(),
         CN45: Queue(),
      }

      self.image_times = {
         C0: 0.0,
         C45: 0.0,
         CN45: 0.0
      }

      self.initial_times = {
         C0: 0.0,
         C45: 0.0,
         CN45: 0.0
      }

      self.this_images = {
         C0: None,
         C45: None,
         CN45: None
      }

      self.vel = np.zeros(3)
      self.target_vel = target_vel

      self.decision_makers = {
         C0: DecisionMaker(self.target_vel),
         C45: DecisionMaker(self.target_vel, threshold_constant=1.2, min_decision=4),
         CN45: DecisionMaker(self.target_vel, threshold_constant=1.2, min_decision=4)
      }
            
      self.subscribers(wait_for_imtopic_s)
      self.publishers()
      
      self.OF_modules = {
         C0: OpticFlow(camera_instance=self.cam),
         C45: OpticFlow(camera_instance=self.cam),
         CN45: OpticFlow(camera_instance=self.cam),
      }
      
      
      self.matched_filters = {
         C0: self.get_matched_filter(self.cam),
         C45: self.get_matched_filter(self.cam, axis=[0.0, 0.0, 0.0]),
         CN45: self.get_matched_filter(self.cam, axis=[0.0, 0.0, 0.0]),
      }

      self.data_collection = data_collection
      if self.data_collection:
         self.pos_subs = rospy.Subscriber(
         '/mavros/local_position/pose', PoseStamped, self.data_collection_cb
         )
         #self.distance = 30.13
         self.distance = 30.13 - 4.8
         self.current_distance = self.distance

      self.side_decisions = {
         C45: deque([], maxlen=10),
         CN45: deque([], maxlen=10),
      }


   def publishers(self):
      """Initialise publishers
      """
      self.flow_publishers = {
         C0: rospy.Publisher(self.node_name + '/optic_flow', FlowMsg, queue_size=10),
         C45: rospy.Publisher(self.node_name + '/optic_flow_c45', FlowMsg, queue_size=10),
         CN45: rospy.Publisher(self.node_name + '/optic_flow_cn45', FlowMsg, queue_size=10)
      }
      
      self.flow_msgs = {
         C0: FlowMsg(),
         C45: FlowMsg(),
         CN45: FlowMsg()
      }

      self.activation_publishers = {
         C0: rospy.Publisher(self.node_name + '/activation', ActivationMsg, queue_size=10),
         C45: rospy.Publisher(self.node_name + '/activation_c45', ActivationMsg, queue_size=10),
         CN45: rospy.Publisher(self.node_name + '/activation_cn45', ActivationMsg, queue_size=10)
      }

      self.activation_msgs = {
         C0: ActivationMsg(),
         C45: ActivationMsg(),
         CN45: ActivationMsg()
      }

      self.decision_publishers = {
         C0: rospy.Publisher(self.node_name + '/decision', DecisionMsg, queue_size=10),
         C45: rospy.Publisher(self.node_name + '/decision_c45', DecisionMsg, queue_size=10),
         CN45: rospy.Publisher(self.node_name + '/decision_cn45', DecisionMsg, queue_size=10)
      }

      # Int messages: 0 -> stop or obstacle; 1 -> go or clear
      self.decision_msgs = {
         C0: DecisionMsg(),
         C45: DecisionMsg(),
         CN45: DecisionMsg()
      }

      self.avoidance_direction_publisher = rospy.Publisher(
         self.node_name + '/direction', AvoidanceDirectionMsg, queue_size=10
      )

      self.avoidance_direction_msg = AvoidanceDirectionMsg()  # Left, Right or Back
   
   
   def subscribers(self, wait_for_imtopic_s):
      """Subscribe to topics

      Args:
          wait_for_imtopic_s (int): number of seconds to wait ofr publishers
      """
      # Subscribe to the image topics
      self.cam_0_subs = rospy.Subscriber(
         self.cam_topics[C0], Image, self.camera_0_cb, queue_size=5)

      self.cam_45_subs = rospy.Subscriber(
         self.cam_topics[C45], Image, self.camera_45_cb, queue_size=5)

      self.cam_n45_subs = rospy.Subscriber(
         self.cam_topics[CN45], Image, self.camera_n45_cb, queue_size=5)
      
      self.cam_info_subs = rospy.Subscriber(
         self.cam_info, CameraInfo, self.cam_info_cb
      )

      self.vel_subs = rospy.Subscriber(
         '/mavros/local_position/velocity_local', TwistStamped, self.vel_subs_cb
      )

      self.pyx4_state_subs = rospy.Subscriber('/pyx4_node/pyx4_state', 
                                                pyx4_state, self.state_cb)
      
      try:
         rospy.loginfo('waiting for camera topics to be published')
         image_info_msg = rospy.wait_for_message(self.cam_info, CameraInfo, timeout=wait_for_imtopic_s)
         self.cam_frame_h = image_info_msg.height         # 120
         self.cam_frame_w = image_info_msg.width          # 16
         
         image_msg = rospy.wait_for_message(self.cam_topics[C0], Image, timeout=wait_for_imtopic_s)

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
      self.camera_general_cb(C0, data)

   def camera_45_cb(self, data):
      """Callback for one side camera

      Args:
          data (Image message): the callback data
      """
      self.camera_general_cb(C45, data)

   def camera_n45_cb(self, data):
      """Callback for one side camera

      Args:
          data (Image message): the callback data
      """
      self.camera_general_cb(CN45, data)
      
   def cam_info_cb(self, data):
      """Add the camera_0 instance

      Args:
         data (CameraInfo): the camera info topic.
      """
      if not self.cam:
         self.cam = Camera(data)

   def state_cb(self, data):
      rospy.loginfo(data)
      if data.flight_state in ('Teleoperation', 'Waypoint'):
         self.start_decision_makers()
   
   def camera_general_cb(self, cam, data):
      """Callback for the camera topic. Add the image to an image queue.

      Args:
          cam (str): which camera (cam_0, cam_45, cam_n45)
          data (Image): image from the subscriber
      """
      try: 
         
         time_last_image = self.image_times[cam]
         # Get the image from the data
         self.this_images[cam] = bridge.imgmsg_to_cv2(data)
         # Update time
         self.image_times[cam] = data.header.stamp.to_sec()
         
         if time_last_image != self.image_times[cam]:
            # Add the image to the queue
            self.image_queues[cam].put([self.this_images[cam], self.image_times[cam]])
            
      # If there is a CvBridge error, print it      
      except CvBridgeError as err:
         print(err)

   def vel_subs_cb(self, data):
      v = data.twist.linear
      self.vel = np.array([v.x, v.y, v.z])

   def data_collection_cb(self, data):
      data = data.pose.position
      self.current_distance = (self.distance - 
                               np.sqrt(data.x ** 2 + data.y ** 2))

   def publish_flow(self, flow, cam):
      if self.OF_modules[cam].initialised:
         cols = flow.shape[1]
         flat_flow = list(np.ravel(flow))
         self.flow_msgs[cam].header.stamp = rospy.Time.now()
         self.flow_msgs[cam].cols = cols
         self.flow_msgs[cam].flow = flat_flow
         self.flow_publishers[cam].publish(self.flow_msgs[cam])

   def publish_activation(self, activation, cam):
      if self.OF_modules[cam].initialised:
         self.activation_msgs[cam].header.stamp = rospy.Time.now()
         self.activation_msgs[cam].activation = activation
         self.activation_publishers[cam].publish(self.activation_msgs[cam])

   def publish_decision(self, d, cam):
      """Publish a camera decision

      Args:
          d (int): 0 -> stop or obstacle; 1 -> go or clear
          cam (str): C0, C45 or CN45
      """
      self.decision_msgs[cam].decision = d
      self.decision_msgs[cam].header.stamp = rospy.Time.now()
      self.decision_publishers[cam].publish(self.decision_msgs[cam])


   def publish_direction(self, d):
      """Publish the main decision message, which will make
      the drone go back, go left or go right

      Args:
          d (str): left, back, or right
      """
      self.avoidance_direction_msg.direction = d
      self.avoidance_direction_msg.header.stamp = rospy.Time.now()
      self.avoidance_direction_publisher.publish(self.avoidance_direction_msg)
      


   def get_matched_filter(self, cam, orientation=[0.0, 0.0, 0.0], axis=[0.0, 0.0, 0.0]):
      return MatchedFilter(
         cam.w, cam.h, 
         (cam.fovx_deg, cam.fovy_deg), orientation=orientation, axis=axis
      ).matched_filter


   def reset_desicion_makers(self):
      for c in self.decision_makers:
         self.decision_makers[c].reset()

   def start_decision_makers(self):
      self.decision_makers[C0].start()
      self.decision_makers[C45].start()
      self.decision_makers[CN45].start()
       
   def main(self):
      while not rospy.is_shutdown():
         for cam in self.cam_iter:
            if not self.image_queues[cam].empty():
               this_image, this_image_time = self.image_queues[cam].get()
               
               # if we don't subtract the initial camera time 
               # the frame difference can be 0.0 due to precision errors
               if not self.initial_times[cam]:
                  self.initial_times[cam] = this_image_time                  
               this_image_time = this_image_time - self.initial_times[cam]

               flow = self.OF_modules[cam].step(this_image, this_image_time)
               self.publish_flow(flow, cam)
               
               if self.OF_modules[cam].initialised:
                  activation = get_activation(flow, self.matched_filters[cam])
                  self.publish_activation(activation, cam)

                  if self.decision_makers[cam].started:
                     decision = self.decision_makers[cam].step(activation)
                     if cam == C0:
                        print(decision)
                     if decision:
                        print(cam, decision)
                        # TODO Do I need to publish this?
                        self.publish_decision(decision, cam)
                        
                        if cam == C0:
                           dir = get_direction(self.side_decisions[C45], 
                                               self.side_decisions[CN45])
                           self.publish_direction(dir)
                           # Turn off detection while turning
                           rospy.sleep(0.5)
                           # Reset the decision makers
                           self.reset_desicion_makers()
                           self.start_decision_makers()
                     else:
                        self.publish_decision(decision, cam)

                     if cam != C0:
                        self.side_decisions[cam].append(decision)

                  if self.data_collection and cam == C0:
                     rospy.loginfo('Activation: ' + str(activation))
                     rospy.loginfo('Distance: ' + str(self.current_distance))

            if self.data_collection and self.current_distance < 0.5:
               os.system("rosnode kill --all")

      
if __name__ == '__main__':
   rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
   
   import argparse
   parser = argparse.ArgumentParser(description="")
   # Stuff that goes in teleop
   parser.add_argument('--data_collection', '-d', type=bool, default=False)    
   parser.add_argument('--velocity', '-v', type=float, default=2.0)    
   args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
  
   OF = OpticFlowROS(NODE_NAME, target_vel=args.velocity, data_collection=False)
   OF.main()
      
        
