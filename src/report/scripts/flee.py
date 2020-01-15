#! /usr/bin/env python
import numpy as np
import rospy
import sys
# import mypkg.homing as hm

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class SimpleHoming:

    def __init__(self):
        # the final position
        self.final_position = np.array([0.0, 0.0])

        # initialize position
        self.position = np.random.normal(0, 1, 2)
        self.orientation = 0  # np.random.normal(0, 1, 1) # this causes errors
        self.orientation_cat = 0

        rospy.Subscriber("/mouse/dead_reckoning", Pose, self.pose_callback)
        rospy.Subscriber("/cat/dead_reckoning",
                         Pose, self.NewTarget_callback)

        pup = rospy.Publisher(
            "HO_signal_mouse", Twist, queue_size=10)
        while not rospy.is_shutdown():
            output = Twist()
            output = closed_loop(
                self.position, self.orientation, self.final_position, self.orientation_cat)
            pup.publish(output)

    def pose_callback(self, current_pose):
        # rospy.loginfo("\n\n%s",current_pose)
        self.position = np.array(
            [current_pose.position.x, current_pose.position.y])
        self.orientation = current_pose.orientation.z

    def NewTarget_callback(self, target):
        self.final_position = np.array(
            [target.position.x, target.position.y])
        self.orientation_cat = target.orientation.z
        pass


def closed_loop(mouse_pos, mouse_orientation, cat_pos, cat_orientation):
    velocity_adjustment = Twist()

    # default values
    velocity_adjustment.linear.x = 0
    velocity_adjustment.angular.z = 0

    # Regel Parameter:
    p_distance = 3.0  # 3.0
    p_rot = 100  # 2.0
    max_speed = 1.0
    max_rot_speed = 1.0
    min_rot_speed = 0.0
    min_lin_speed = .9

    # NEW: rotate target vector 180
    v_flee_global = mouse_pos - cat_pos
    M_rot_mouse = np.matrix([[np.cos(mouse_orientation), -np.sin(mouse_orientation)],
                             [np.sin(mouse_orientation), np.cos(mouse_orientation)]])

    v_flee_rel2mouse = np.asarray(np.dot(M_rot_mouse.T, v_flee_global).T)

    # NEW-----try to flee tangential----------
    M_rot_cat = np.matrix([[np.cos(cat_orientation), -np.sin(cat_orientation)],
                           [np.sin(cat_orientation), np.cos(cat_orientation)]])

    v_flee_rel2cat = np.asarray(np.dot(M_rot_cat.T, v_flee_global).T)

    # turn around 90 deg depending on how the cat looks at us:

    if np.sign(-v_flee_rel2cat[1]) > 0:
        v_flee_rel2mouse = np.array(
            [v_flee_rel2mouse[1], -v_flee_rel2mouse[0]])
    else:
        v_flee_rel2mouse = np.array(
            [-v_flee_rel2mouse[1], v_flee_rel2mouse[0]])

    d = np.linalg.norm(v_flee_rel2mouse)

    v_view_mouse = np.array([1, 0])

    # -1 -> bitte um 180 grad drehen. 1 -> perfekt, so bleiben
    rot_error = np.dot(v_view_mouse, v_flee_rel2mouse / d)

    # ------------------Regelung----------------------

    lin_speed = rot_error*2.0
    velocity_adjustment.linear.x = np.clip(
        lin_speed, -max_speed, max_speed)

    # send the distance:
    velocity_adjustment.linear.z = d

    rot_speed = p_rot * (-.5 * rot_error + .5)  # mapping [-1,1] to [1,0]
    rot_speed = np.clip(rot_speed, min_rot_speed, max_rot_speed)
    # rot_speed = max_rot_speed
    velocity_adjustment.angular.z = np.sign(-v_flee_rel2mouse[1])*rot_speed

    return velocity_adjustment


if __name__ == "__main__":
    rospy.init_node("homing")
    try:
        node = SimpleHoming()

    except rospy.ROSInterruptException:
        pass
