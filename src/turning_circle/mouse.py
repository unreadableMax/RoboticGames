#! /usr/bin/env python
import numpy as np
import rospy
import sys
import mypkg.homing as hm

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class Mouse:
    
    def __init__(self):
        # the final position
        final_position = np.array([7.0, 3.0])

        # initialize position
        self.position = np.random.normal(0, 1, 2)
        self.orientation = 0  # np.random.normal(0, 1, 1) # this causes errors

        rospy.Subscriber("/mouse/dead_reckoning", Pose, self.pose_callback)

        drive_to_pose_puplisher = rospy.Publisher(
            "/mouse/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():
            output = Twist()
            output = closed_loop(
                self.position, self.orientation, final_position)
            drive_to_pose_puplisher.publish(output)

    def pose_callback(self, current_pose):
        self.position = np.array(
            [current_pose.position.x, current_pose.position.y])
        self.orientation = current_pose.orientation.z


def closed_loop(position, orientation, target):
    velocity_adjustment = Twist()

    res = hm.homing(position, orientation, target)

    # default values
    velocity_adjustment.linear.x = res[0]
    velocity_adjustment.angular.z = res[1]
    return velocity_adjustment
    
    

if  __name__=="__main__":
    rospy.init_node("mouse")
    try:
        node=Mouse()
    except rospy.ROSInterruptException:
        pass
