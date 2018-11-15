#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import JointState 

# take in a message msg and a file to write to f
def callback(msg, f):
  # https://stackoverflow.com/questions/44778/how-would-you-make-a-comma-separated-string-from-a-list-of-strings
  f.write(str(float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs)/1000000000) + ',' + ','.join(map(str, msg.position)) + "\n")

def main():
  rospy.init_node('joint_state_listener', anonymous=True)
  timestr = time.strftime("%Y%m%d-%H%M%S")
  recording_file = "joints_recording_" + timestr + ".txt"
  with open(recording_file, "w+") as f:
    rospy.Subscriber("/joint_states", JointState, callback, (f))
    rospy.spin();

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
       pass
