#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist

class Fusion:
	def __init__(self):
                # array containes cat,blue,orange,cheese,collision
                self.behaviors = np.zeros((3,2))
                for i in range(3):
                    rospy.Subscriber(sys.argv[i+1], Point, self.generate_behavior,(i))
		self.pub= rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
		while not rospy.is_shutdown():
			self.fusion()

        def generate_behavior(self,msg,arg):
                    self.behaviors[arg] = np.array([msg.linear.x,msg.angular.z])

	def fusion(self):
		cmd_vel = Twist()
		cmd_vel.linear.x  = prevail_gate(or_gate(behavior[0,0],behavior[1,0]),behavior[2,0])
                cmd_vel.angular.z = prevail_gate(or_gate(behavior[0,1],behavior[1,1]),behavior[2,1])
		self.pub.publish(cmd_vel)

	
def or_gate(x, y):
        a = 1.02889
        b = 0.3574
        if x == 0 and y == 0:
            return 0
        else:
            return x*(np.exp(-((a*y**2+b*x*y)/(x**2+y**2)))) + y*(np.exp(-((a*x**2+b*x*y)/(x**2+y**2))))


def prevail_gate(x, y):
        return or_gate(x, or_gate(x, y))

if __name__ == '__main__':
	try:
		rospy.init_node("fusion")
		fus = Fusion()	
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")
