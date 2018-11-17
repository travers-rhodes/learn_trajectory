#!/usr/bin/env python
import rospy
import time

import feedbot_trajectory_logic.tracker_interface as tracker
import numpy as np
from learn_trajectory.srv import PlayTrajectory
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Quaternion


class SpoonFeeder:
  def __init__(self):
    rospy.logwarn("sleeping for 5 seconds before starting recorded motion")
    rospy.sleep(5)
    # quaternion is defined in order x,y,z,w
    self.defaultQuat = Quaternion(0.5, 0.5, 0.5, 0.5)
    self.tracker = tracker.TrackerInterface(self.defaultQuat)
    self.play_trajectory_topic = "trained_poses"
    self._play_trajectory = rospy.ServiceProxy("play_trajectory", PlayTrajectory)

  def follow_trajectory(self):
    self.tracker.start_updating_target_to_pose(self.play_trajectory_topic)
    rospy.logwarn("Playing trajectory at " + self.play_trajectory_topic)
    self._play_trajectory(String(self.play_trajectory_topic))

if __name__=="__main__":
  rospy.init_node('spoon_feeder', anonymous=True)
  s = SpoonFeeder()
  while (True):
    s.follow_trajectory()
    time.sleep(5)
