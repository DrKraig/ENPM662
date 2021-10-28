#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def main():
    try:
        rospy.init_node('straight_line_publisher')
        pub = rospy.Publisher('/chatter', Float64, queue_size=10)
        rate = rospy.Rate(100)
        for i in range(100):
            pub.publish(0.0)
            print("Published velocity command - 0")
            rate.sleep()        
        for i in range(2000):
            pub.publish(-3.0)
            print("Published velocity command - -3")
            rate.sleep()
        for i in range(1000):
            pub.publish(0.0)
            print("Published velocity command - 0")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()