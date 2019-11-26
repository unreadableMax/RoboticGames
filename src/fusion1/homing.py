#! /usr/bin/env python
import numpy as np
import rospy
import sys
#import mypkg.homing as hm

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class SimpleHoming:

    def __init__(self):
        # the final position
        self.final_position = np.array(
            [float(sys.argv[1]), float(sys.argv[2])])

        # initialize position
        self.position = np.random.normal(0, 1, 2)
        self.orientation = 0  # np.random.normal(0, 1, 1) # this causes errors

        rospy.Subscriber("dead_reckoning", Pose, self.pose_callback)
        rospy.Subscriber("TargetPos", Twist, self.NewTarget_callback)

        pup = rospy.Publisher(
            "home_signal", Twist, queue_size=10)
        while not rospy.is_shutdown():
            output = Twist()
            output = closed_loop(
                self.position, self.orientation, self.final_position)
            pup.publish(output)

    def pose_callback(self, current_pose):
        # rospy.loginfo("\n\n%s",current_pose)
        self.position = np.array(
            [current_pose.position.x, current_pose.position.y])
        self.orientation = current_pose.orientation.z

    def NewTarget_callback(self, target):
        self.final_position[0] = target.angular.z
        self.final_position[1] = target.linear.x


def closed_loop(position, orientation, target):
    velocity_adjustment = Twist()

    # default values
    velocity_adjustment.linear.x = 0
    velocity_adjustment.angular.z = 0

    # Regel Parameter:
    p_distance = 3.0  # 3.0
    p_rot = 1  # 2.0
    max_speed = 1.0
    max_rot_speed = 1.0
    min_rot_speed = 0.0
    min_lin_speed = .9

    v_target = target - position
    M_rot = np.matrix([[np.cos(orientation), -np.sin(orientation)],
                       [np.sin(orientation), np.cos(orientation)]])
    v_target = np.asarray(np.dot(M_rot.T, v_target).T)

    distance_to_target = np.linalg.norm(v_target)

    v_view = np.array([1, 0])

    # -1 -> bitte um 180 grad drehen. 1 -> perfekt, so bleiben
    rot_error = np.dot(v_view, v_target / distance_to_target)

    # ------------------Regelung----------------------

    lin_speed = p_distance*distance_to_target*rot_error
    if distance_to_target > .5:  # do only drive backwards if we are close enouth to the target
        velocity_adjustment.linear.x = np.clip(
            lin_speed, min_lin_speed, max_speed)
    else:
        velocity_adjustment.linear.x = np.clip(
            lin_speed, -max_speed, max_speed)

    velocity_adjustment.linear.x = distance_to_target

    rot_speed = p_rot * (-.5 * rot_error + .5)  # mapping [-1,1] to [1,0]
    rot_speed = np.clip(rot_speed, min_rot_speed, max_rot_speed)
    velocity_adjustment.angular.z = np.sign(-v_target[1])*rot_speed

    return velocity_adjustment


if __name__ == "__main__":
    rospy.init_node("homing")
    try:
        node = SimpleHoming()

    except rospy.ROSInterruptException:
        pass
