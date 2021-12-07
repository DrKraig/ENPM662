#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

def callback(data):
    if ( data.data == "Open Door"):
        talker()
        sub.unregister()
    
def main():
    rospy.init_node('FK_validation', anonymous=True)
    rospy.spin()

def talker():
    
    length = 6
    arm = JointTrajectory()
    arm.joint_names = ["ra_shoulder_pan_joint", "ra_shoulder_lift_joint", "ra_elbow_joint", "ra_wrist_1_joint",
  "ra_wrist_2_joint", "ra_wrist_3_joint"]
    # for i in range(10):
    newPoint = JointTrajectoryPoint()
    newPoint.time_from_start.secs = 1
    newPoint.velocities = [ 0.0 for _ in range(length)]
    newPoint.positions = [ 0.0 for _ in range(length)]
    newPoint.positions[0] = -1.5708
    arm.points.append(newPoint)
    
    counter = 0
    rate = rospy.Rate(100) # 10hz
    while counter < 1:
        pub.publish(arm)
        rate.sleep()
        counter += 1

    return
    
pub = rospy.Publisher('/ra_trajectory_controller/command', JointTrajectory, queue_size=10)
sub = rospy.Subscriber("/relay", String, callback)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass