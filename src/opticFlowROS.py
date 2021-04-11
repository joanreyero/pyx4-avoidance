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
from avoidance_functions import get_direction, get_activation
from obstacleFinder import ActivationDecisionMaker as DecisionMaker
from pyx4_avoidance.msg import activation as ActivationMsg
from pyx4.msg import pyx4_state
from pyx4_avoidance.msg import avoidancedecision as DecisionMsg
from pyx4_avoidance.msg import avoidancedirection as AvoidanceDirectionMsg
from pyx4_avoidance.msg import avoidancetunneldata as AvoidanceTunnelDataMsg
import plotter_flow
from camera_labels import *
from camera import Camera
import rospy
import sys
from avoidance_behaviours import TunnelCenteringBehaviour, AvoidanceBehaviour, SaccadeBehaviour

try:
   from queue import Queue
except ImportError:
   import Queue as Queue


from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


NODE_NAME = 'pyx4_avoidance_node'


class OpticFlowROS():
   
   def __init__(self, node_name, avoidance_type,
                target_vel = 2,
                cam_0_topic='/resize_img/image', 
                cam_45_topic='/resize_img_45/image', 
                cam_n45_topic='/resize_img_n45/image', 
                cam_info="/resize_img/camera_info", 
                wait_for_imtopic_s=100,
                data_collection=False,
                save_flow=''):
      
      self.node_name = node_name

      # Init empty cameras
      self.cam = None
      self.cam_info = cam_info

      self.save_flow = save_flow

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
            
      self.subscribers(wait_for_imtopic_s)
      self.publishers()
      
      self.OF_modules = {
         C0: OpticFlow(camera_instance=self.cam),
         C45: OpticFlow(camera_instance=self.cam),
         CN45: OpticFlow(camera_instance=self.cam),
      }

      self._init_data_collection(data_collection)

      self.cameras = [C45, C0, CN45]
      self.last_flows = {C0: [], C45: [], CN45: []}

      self.avoidance_type = avoidance_type

      if self.avoidance_type == 'tunnel-centering':
         self.behaviour = TunnelCenteringBehaviour(self.cam)

      elif self.avoidance_type == 'saccade':
         self.behaviour = SaccadeBehaviour(self.cam)

      self.is_ready = False
      self._central_ready = True

   def _init_data_collection(self, data_collection):
      self.start_data_collection = False
      self.data_collection = data_collection
      if self.data_collection:
         self.pos_subs = rospy.Subscriber(
         '/mavros/local_position/pose', PoseStamped, self.data_collection_cb
         )
         #self.distance = 30.13
         self.distance = 25
         self.current_distance = self.distance

   def publishers(self):
      """Initialise publishers
      """

      self.avoidance_direction_publisher = rospy.Publisher(
         self.node_name + '/direction', AvoidanceDirectionMsg, queue_size=10
      )

      self.avoidance_direction_msg = AvoidanceDirectionMsg()  # Left, Right or Back

      self.avoidance_data_publisher = rospy.Publisher(
         self.node_name + '/avoidance_data', 
         AvoidanceTunnelDataMsg, 
         queue_size=10
      )
      self.avoidance_data_msg = AvoidanceTunnelDataMsg(
         vel=0.0,
         distance=0.0,
         activation_0=[],
         #activation_45=[],
         #activation_n45=[]
      )

      self.avoidance_data_tunnel_publisher = rospy.Publisher(
         self.node_name + '/avoidance_data_tunnel', 
         AvoidanceTunnelDataMsg,
         queue_size=10
      )
      
      self.avoidance_data_tunnel_msg = AvoidanceTunnelDataMsg(
         vel=0.0,
         distance=0.0,
         activation_0=[],
         activation_1=[],
         activation_2=[],
      )

      self.draw_publisher = self.image_pub = rospy.Publisher(self.node_name + '/optic_flow_draw', Image)
   
   
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
      if data.flight_state in ('Teleoperation', 'Waypoint'):
         if self.data_collection:
            self.start_data_collection=True
         self.behaviour.start()
         rospy.Timer(rospy.Duration(4), self.ready, oneshot=True)
         
         
   
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

   
   def publish_direction(self, angle, t):
      """Publish the main decision message, which will make
      the drone go back, go left or go right

      Args:
          d (str): left, back, or right
      """
      self.avoidance_direction_msg.angle = angle
      self.avoidance_direction_msg.type = t
      self.avoidance_direction_msg.header.stamp = rospy.Time.now()
      self.avoidance_direction_publisher.publish(self.avoidance_direction_msg)

   def publish_tunnel_data(self, activations):
      if self.start_data_collection:
         self.avoidance_data_tunnel_msg.vel=float(self.target_vel)
         self.avoidance_data_tunnel_msg.distance=self.current_distance
         self.avoidance_data_tunnel_msg.activation_0=list(activations[0])
         self.avoidance_data_tunnel_msg.activation_1=list(activations[1])
         self.avoidance_data_tunnel_msg.activation_2=list(activations[2])
         self.avoidance_data_tunnel_publisher.publish(self.avoidance_data_tunnel_msg)
            
   def get_flows(self, draw_image=False):
      flows = []
      for i, cam in enumerate(self.cameras):

         if self.image_queues[cam].empty():
            if len(self.last_flows[cam]) == 0:
               return False
            
            flow = self.last_flows[cam]
            
         else:
            this_image, this_image_time = self.image_queues[cam].get()
            
            # if we don't subtract the initial camera time 
            # the frame difference can be 0.0 due to precision errors
            if not self.initial_times[cam]:
               self.initial_times[cam] = this_image_time                  
            this_image_time = this_image_time - self.initial_times[cam]
            flow = self.OF_modules[cam].step(this_image, this_image_time)

            if not self.OF_modules[cam].initialised:
               return False

            self.last_flows[cam] = flow

            if draw_image == i and draw_image:
               draw = plotter_flow.draw_flow(flow, this_image, save=round(self.current_distance, 2))
               im_msg = bridge.cv2_to_imgmsg(draw, encoding="passthrough")
               self.draw_publisher.publish(im_msg)

            if i == 1 and self.save_flow:
               if ((self.current_distance < 21 and self.current_distance > 19) or
                   (self.current_distance < 11 and self.current_distance > 9) or
                   (self.current_distance < 6 and self.current_distance > 4)):
                                     
                  plotter_flow.save_flow(flow, this_image, 
                                       int(self.current_distance), 
                                       self.save_flow, just_img=False)
               
         flows.append(flow)
      return flows

   def ready(self, t):
      self.is_ready = True

   def central_ready(self, t):
      self._central_ready = True

       
   def main(self):
            
      while not rospy.is_shutdown():
            
         flows = self.get_flows(draw_image=False)
         if not self._central_ready:
            flows[1] *= 0
         
         if flows and self.is_ready:
            activations, direction = self.behaviour.step(flows)
            if activations:
               self.publish_tunnel_data(activations)               
            if direction and not self.data_collection:
               self.publish_direction(direction, 'relative')
               self.behaviour.reset()
               self.is_ready = False
               print('Direction: ' + str(direction))
               if abs(direction) > 45:
                  self._central_ready = False
                  duration = 3
                  rospy.Timer(rospy.Duration(5), self.central_ready, oneshot=True)
                  
               else: 
                  if self.avoidance_type == 'saccade':
                     duration = 2
                  else:
                     duration = 1
               rospy.Timer(rospy.Duration(duration), self.ready, oneshot=True)
            

         if self.data_collection and self.current_distance < 2:
            os.system("rosnode kill --all")

      
if __name__ == '__main__':
   rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
   
   import argparse
   parser = argparse.ArgumentParser(description="")
   # Stuff that goes in teleop
   parser.add_argument('--data_collection', '-d', action='store_true')    
   parser.add_argument('--save_flow', type=str, default='')    
   parser.add_argument('--velocity', '-v', type=float, default=2.0)
   
   args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
  
   OF = OpticFlowROS(NODE_NAME, target_vel=args.velocity, data_collection=args.data_collection, save_flow=args.save_flow, avoidance_type='tunnel-centering')
   OF.main()
      
        
