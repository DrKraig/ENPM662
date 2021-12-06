#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
def talker():
    first_finger = JointTrajectory()
    first_finger.joint_names = ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_LFJ1",
  "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
  "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
  "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
  "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4",
  "rh_THJ5", "rh_WRJ1", "rh_WRJ2"]
    newPoint = JointTrajectoryPoint()
    newPoint.positions = [0.0001, 0.0349, 0.76794, 0.349, 0.0003,
    0.01745, 0.0698, -0.3490, 0.15707,
    0.00032, 0.03490, 0.17453, 0.3490,
    -0.00048, 0.0349, 0.174, -0.349065,
    0.10471, 0.69813, 0.03490, 1.2217,
    0.5585, 0.4886, -0.523598]
    newPoint.time_from_start.nsecs = 5000000
    newPoint.velocities = [0.0 for _ in range(len(newPoint.positions))] 
    first_finger.points.append(newPoint)
    pub = rospy.Publisher('/rh_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('custom_first_finger_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        pub.publish(first_finger)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass