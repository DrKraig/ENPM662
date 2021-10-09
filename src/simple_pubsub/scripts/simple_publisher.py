#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class simple_talker:
    
    def __init__(self):
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
    
    def start_talking(self):    
        rate = rospy.Rate(10) # 10hz
        print("\n I'm publishing on /chatter topic go listen \n")
        while not rospy.is_shutdown():
            my_string = "Gossip"
            self.pub.publish(my_string)
            rate.sleep()
        print("\n I've stopped publishing \n")    
    
        
def main():
    try:
        rospy.init_node('simple_talker', anonymous=True)
        talker = simple_talker()
        talker.start_talking()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()