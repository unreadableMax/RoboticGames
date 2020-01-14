#! /usr/bin/env python
import numpy as np
import rospy
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class Test:
    
    def __init__(self):
        self.position  = [0, 0]
        self.orientation = [0]
        rospy.Subscriber("/cat/dead_reckoning", Pose, self.dead_reckoning_callback)
        pub = rospy.Publisher("/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        output = Twist()
        speed = 1
        distance = 3
        current_distance = 0
        output.linear.x = speed
        
        while not rospy.is_shutdown():
            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            
            output.linear.x = 1
            
            while(current_distance < distance):
                #Publish the velocity
                pub.publish(output)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= speed*(t1-t0)
                #After the loop, stops the robot
                
            output.linear.x = 0
                
            pub.publish(output)
            
    def dead_reckoning_callback(self, msg):
        self.position[0] = msg.position.x
        self.position[1] = msg.position.y
        self.orientation[0] = msg.orientation.z
        rospy.loginfo("Orientation " + str(self.orientation[0]))

if  __name__=="__main__":
    rospy.init_node("homing")
    try:
        node=Test()
    
    except rospy.ROSInterruptException:
        pass
