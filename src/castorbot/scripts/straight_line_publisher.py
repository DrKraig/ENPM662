#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def main():
    try:
        rospy.init_node('straight_line_publisher')
        pub = rospy.Publisher('/castorbot/rear_motor/command', Float64, queue_size=10)
        pub_right = rospy.Publisher('/castorbot/right_castor_motor/command', Float64, queue_size=10) 
        pub_left = rospy.Publisher('/castorbot/left_castor_motor/command', Float64, queue_size=10)
        rate = rospy.Rate(100)
        for i in range(100):
            pub.publish(0.0)
            pub_right.publish(0.0)
            pub_left.publish(0.0)
            print("Published velocity command - 0")
            rate.sleep()        
        for i in range(2000):
            pub.publish(-10.0)
            pub_right.publish(0.0)
            pub_left.publish(0.0)
            print("Published velocity command - -10")
            rate.sleep()
        for i in range(1000):
            pub.publish(0.0)
            pub_right.publish(0.0)
            pub_left.publish(0.0)
            print("Published velocity command - 0")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()