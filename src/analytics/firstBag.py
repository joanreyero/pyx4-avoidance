import rospy
import rosbag
import numpy as np
import pandas as pd
from pyx4.msg import pyx4_state
from geometry_msgs.msg import TwistStamped, PoseStamped
from analiticsLabels import AnalyticsLabels as labels
from analiticsVars import *

TOPICS = [
    '/mavros/local_position/pose',
    '/mavros/local_position/velocity_local', 
    '/pyx4_node/pyx4_state',
]

class AvoidanceBagReader(object):
    """Class to read a rosbag and create a dataframe where
    each row corresponds to one timestamp.
    Rows with NaN data are removed.
    """
    def __init__(self, bag, topics=TOPICS,
                 flight_direction=np.array([1, 0]), 
                 distance = 30.0,
                 init_after_takeoff=True,
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
        marker_topic = '/pyx4_node/pyx4_state'

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
        self.df = self.make_df(topics, marker_topic, bag, verbose)

    def get_msg(self, topic, msg):
        """Return the information from a topic's message

        Args:
            topic (str): the topic
            msg (msg): the ROS message

        Returns:
            float: the relevant part of the message
        """
        if labels.get_label(topic) == POSITION:
            return self.get_position_msg(msg)

        elif labels.get_label(topic) == VELOCITY:
            return self.get_velocity_msg(msg)

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

    def check_init(self, data):
        """Check whether to start recording

        Args:
            data (pyx4_state): pyx4_state messge
        """
        # If we are in Waypoint state, start
        if data.flight_state == 'Waypoint':
            self._init = True
        
    def make_df(self, topics, marker_topic, save_name, verbose):
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
        current_time = rospy.Time()
        # Initialise a temp;ate dictionaty
        templ = {c: np.NaN for c in self.cols}
        # Initialise a dict for each iteration
        current = templ.copy()
        # Iterate thorugh the bag
        for topic, msg, t in self.bag.read_messages(topics=topics):
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

        df = df.dropna()  # Drope empty (np.NaN)
        # Save
        df.to_csv('data/' + save_name + '.csv', index_label='time')
        
        if verbose:
            print(df)
            
        return df


if __name__ == '__main__':
    reader = AvoidanceBagReader('try', verbose=True)
