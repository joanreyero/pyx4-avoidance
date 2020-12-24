import rospy
import rosbag
import numpy as np
import pandas as pd
from pyx4.msg import pyx4_state
from geometry_msgs.msg import TwistStamped, PoseStamped
import sys
sys.path.append('../')
from matchedFilters import MatchedFilter
from activation import get_activation
from analiticsLabels import AnalyticsLabels as labels
from analiticsVars import *


class AvoidanceBagReader(object):
    """Class to read a rosbag and create a dataframe where
    each row corresponds to one timestamp.
    Rows with NaN data are removed.
    """
    def __init__(self, bag, topics=TOPICS,
                 flight_direction=np.array([1, 0]), 
                 distance = 30.13,
                 init_after_takeoff=True,
                 make_fovs = False,
                 verbose=False):
        """Initialise the class

        Args:
            bag (str): the rosbag file in dir bags. Without the .bag.
            topics (list, optional): list of topics. Defaults to TOPICS.
            flight_direction (aeeay, optional): Defaults to np.array([1, 0]).
            distance (float, optional): Dist to obstacle. Defaults to 30.0.
            init_after_takeoff (bool, optional): Start recording after take-off. 
                                                 Defaults to True.
            verbose (bool, optional): print the dataframe. Defaults to False.
        """
        # Get the rosbag
        self.bag = rosbag.Bag('bags/' + bag + '.bag')

        # We need a unit vector to make the projection
        self.flight_direction = (flight_direction / 
                                 np.linalg.norm(flight_direction))
        self.distance = float(distance)

        # The topic that marks state
        marker_topic = STATE_TOPIC

        self.cam_fovx = 45
        self.cam_fov_y = 27
        self.cam_w = 240
        self.cam_h = 135

        # Initialise variables that will be needed
        # if we need to consider also other FOVs
        if self.make_fovs:
            # Fields of view to remake the df with
            self.fovs = [90, 60, 30]  # Original is 120

            # Matched filters for each optic flow
            self.matched_filters = [MatchedFilter(
                self.cam_w / self.cam_fovx * fov,
                self.cam_h,
                (fov, self.cam_fov_y)
            ).matched_filter for fov in self.fovs]

        # We do not want to record the state topic in the CSV
        if init_after_takeoff:
            idx = topics.index(marker_topic)
            topics_cols = topics[: idx] + topics[idx + 1 :]
        else: topics_cols = topics

        # Columns to record
        self.cols = labels.from_topics(topics_cols) 

        # Whether to start recording
        if init_after_takeoff:
            self._init = False
        else:
            self._init = True

        # Make the dataframe
        self.df = self.make_df(topics, marker_topic, bag, verbose, make_fovs)

    def get_msg(self, topic, msg):
        """Return the information from a topic's message

        Args:
            topic (str): the topic
            msg (msg): the ROS message

        Returns:
            float: the relevant part of the message
        """
        label = labels.get_label(topic)
        if label == POSITION:
            return self.get_position_msg(msg)

        elif label == VELOCITY:
            return self.get_velocity_msg(msg)

        elif label == FLOW:
            return self.get_flow_msg(msg)

        elif label == ACTIVATION:
            return self.get_activation_msg(msg)

        elif label == ACTIVATION_45:
            return self.get_activation_msg(msg)

        elif label == ACTIVATION_N45:
            return self.get_activation_msg(msg)

        elif label == DECISION:
            return self.get_decision_msg(msg)

        elif label == DECISION_45:
            return self.get_decision_msg(msg)

        elif label == DECISION_N45:
            return self.get_decision_msg(msg)

    def get_position_msg(self, data):
        """Get the relevant part of a position message

        Args:
            data (PoseStamped): position message

        Returns:
            float: distance to obstacle
        """
        data = data.pose.position
        return self.distance - np.sqrt(data.x ** 2 + data.y ** 2)

    def get_velocity_msg(self, data):
        """Get the relvant info from a velocity message

        Args:
            data (TwistStamed): velocity message

        Returns:
            float: velocity in direction of movement
        """
        data = data.twist.linear
        return np.array([data.x, data.y]).dot(self.flight_direction)

    def get_flow_msg(self, data):
        """Get the optic flow info from a flow message

        Args:
            data (Flow.msg): an optic flow message that contains
                             a 1-d array and the number of columns. 
        """
        return np.reshape(np.array(data.flow), (-1, data.cols, 2))

    def get_activation_msg(self, data):
        """Get the activation info from an activation message

        Args:
            data (Activation.msg): float with the activation.
        """
        return float(data.activation)

    def get_decision_msg(self, data):
        """Get the activation info from an activation message

        Args:
            data (Activation.msg): float with the activation.
        """
        return float(data.decision)

    def parse_time(self, t):
        """Convert time in from rospy.Time to int with
        4 significait figures.

        Args:
            t (rospy.Time): time from rospy

        Returns:
            int: integer with the time in ns to 4sf.
        """
        t = t.to_nsec()
        return round(t, -(len(str(t)) - 4))
        

    def check_init(self, data):
        """Check whether to start recording

        Args:
            data (pyx4_state): pyx4_state messge
        """
        # If we are in Waypoint state, start
        if data.flight_state == 'Waypoint':
            self._init = True

    def make_fovs(self, df, save_name):
        """Crop the optic flow image for each FOV

        Args:
            df (pd.DataFrame): the original dataframe
            save_name (str): name for saving
        """
        # TODO Get info from camera class
        for i, fov in enumerate(self.fovs):
            df_c = df.copy()
            # Pixels to cut out
            c = (self.cam_w - int(self.cam_w / self.cam_fovx * fov)) / 2
            print(c)
            # Crop leaving the center untouched
            df_c[FLOW] = df_c[FLOW].apply(lambda a: a[:, c:-c, :])
            activation_col = [get_activation(flow, self.matched_filters[i]) for flow in df_c[FLOW]]

            df_c[ACTIVATION] = activation_col
            df_c[ACTIVATION_GRAD] = np.gradient(df_c[ACTIVATION])
            # Save
            self.save(save_name, df_c, fov)

    def save(self, name, df, fov):
        """Save a dataframe as CSV

        Args:
            name (str): Name
            df (pd.DataFrame): dataframe to save
            fov (int/str): the fov of the camera
        """
        df.to_csv('data/' + name + '-fov-' + str(fov) + '.csv', 
                  index_label='time')
        
    def make_df(self, topics, marker_topic, save_name, verbose, make_fovs):
        """Make the dataframe.

        Args:
            topics (list): list of topics
            marker_topic (str): topic that marks state
            save_name (str): name to save the df
            verbose (Bool): whether to prinf the df

        Returns:
            pandas.DataFrame: the df.
        """
        # Initialise the dataframe
        df = pd.DataFrame(columns=self.cols)
        # Initialise 0 time
        current_time = self.parse_time(rospy.Time())
        # Initialise a temp;ate dictionaty
        templ = {c: np.NaN for c in self.cols}
        # Initialise a dict for each iteration
        current = templ.copy()
        # Iterate thorugh the bag
        for topic, msg, t in self.bag.read_messages(topics=topics):
            # Convert to float with adequate precision
            t = self.parse_time(t)            
            
            # If we can start and the topoic is not marker
            if self._init and topic != marker_topic:
                
                if t == current_time:
                    # the key is the topic's label
                    # the message is given my get_msg
                    current[labels.get_label(topic)] = self.get_msg(topic, 
                                                                    msg)
                elif t > current_time:
                    # Add to the dataframe
                    df.loc[current_time] = current
                    # Restart current
                    current = templ.copy()
                    # the key is the topic's label
                    # the message is given my get_msg
                    current[labels.get_label(topic)] = self.get_msg(topic, 
                                                                    msg)
                    # Update time
                    current_time = t

            # Check initialisation
            if topic == marker_topic and not self._init:
                self.check_init(msg)


        df = df.dropna(subset=[DECISION])  # Drop empty (np.NaN)
        df = df.dropna(subset=[DECISION_45])  # Drop empty (np.NaN)
        df = df.dropna(subset=[DECISION_N45])  # Drop empty (np.NaN)
        df = df.dropna(subset=[ACTIVATION])  # Drop empty (np.NaN)
        df[ACTIVATION_GRAD] = np.gradient(df[ACTIVATION])
        
        if make_fovs:
            self.make_fovs(df, save_name)
            
        # Save the dataframe 120
        self.save(save_name, df, self.cam_fovx)
        if verbose:
            print(df)
            
        return df


if __name__ == '__main__':
    reader = AvoidanceBagReader('new', verbose=False, make_fovs=True)
