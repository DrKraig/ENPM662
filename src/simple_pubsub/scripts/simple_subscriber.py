#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class simple_listener:
    
	def __init__(self):
		self.sub = rospy.Subscriber("chatter",String, self.callback)
    
	def callback(self, message):
		print("\n I've heard : {}".format(message.data))

def main():
	try:
		rospy.init_node('simple_listener', anonymous=True)
		listener = simple_listener()
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()