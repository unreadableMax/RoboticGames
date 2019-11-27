#! /usr/bin/env python
import numpy as np
import rospy
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class SimpleHoming:
    
    def __init__(self):
        # the final position
        final_position = np.array([2,4])
        
        #initialize position
        self.position  = np.random.normal(0,1,2)
        self.orientation = np.random.normal(0,1,1)
        '''
        the subscriber should be implemented here.
        it should read in the odometry data and write the position to
        self.position and the orientation to self.orientation.
        to achieve this the callback function needs to be a function of the class
        Simple Homing.
        Tutorial for Python Classes:
        https://www.w3schools.com/python/python_classes.asp
        '''
        rospy.Subscriber("dead_reckoning", Pose, self.dead_reckoning_callback)
        pub = rospy.Publisher("/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():
            output = Twist()
            output = closed_loop(self.position,self.orientation,final_position)
            pub.publish(output)
        
        
    def dead_reckoning_callback(self, msg):
        self.position[0] = msg.position.x
        self.position[1] = msg.position.y
        self.orientation[0] = msg.orientation.z


def closed_loop(position,orientation,target):
        output=Twist()
        '''
        TODO
        Function needs to be implemented in such a way that a Twist Message
        is returned which can controll the robot in such a way that he
        arrives at his target and remains there.
        For a reference on how to write a Twist message one can consult
        exercise_1.py
        '''
        targetX = target[0]
        targetY = target[1]
        selfPosX = position[0]
        selfPosY = position[1]
        selfOrienZ = orientation[0]
        treshold = 0.4
        #rospy.loginfo("position.X " + str(selfPosX))
        #rospy.loginfo("position.Y " + str(selfPosY))
        #rospy.loginfo("Selforientation.Z " + str(selfOrienZ))
        
        target_angle = calculate_target_angle(selfPosX, selfPosY, targetX, targetY)
        distance = calculate_distance(selfPosX, selfPosY, targetX, targetY)
        #rospy.loginfo("distance " + str(distance))
        rospy.loginfo("target_angle " + str(target_angle))
        
        angle_change_value = calc_anglechange(abs(selfOrienZ-target_angle))
        angle_increment = -angle_change_value if target_angle > selfOrienZ else angle_change_value
        output.angular.z = angle_increment
        
        output.linear.x = calc_velocity(distance, abs(selfOrienZ-target_angle))
        
        # check if the robot arrives at the target with respect to a certain treshold
        if (distance < 0.1):
            rospy.loginfo("target reached !!!")
            output.angular.z = 0
            output.linear.x = 0
        
        return output

def calc_anglechange(angleDiff):
        max_angleChange = 2.5
        angle_change = angleDiff**2 if (angleDiff**2)<max_angleChange else max_angleChange
        return angle_change
        
def calc_velocity(distance, angleDiff):
        # calc velocity based on distance
        vel_change_distance = distance if distance<1 else 1
        #clac velocity based on angle diff
        angle_faktor = 3
        vel_change_angle = 1-(angleDiff/angle_faktor)
        
        # take the smaller one
        vel_change = vel_change_distance if vel_change_distance<vel_change_angle else vel_change_angle
        return vel_change

def calculate_target_angle(curX, curY, targetX, targetY):
        x = targetX - curX
        y = targetY - curY
        # because dividing by 0...
        if x == 0:
            return math.radians(90) if y > 0 else math.radians(270)
        result = math.atan(y/x)
        if x < 0 and y < 0:
            result -= math.pi
        if x < 0 and y >= 0:
            result += math.pi
        return result
    
def calculate_distance(curPosX, curPosY, targetX, targetY):
        return math.sqrt((curPosX-targetX)**2 + (curPosY-targetY)**2)
  

if  __name__=="__main__":
    rospy.init_node("homing")
    try:
        node=SimpleHoming()
    
    except rospy.ROSInterruptException:
        pass
