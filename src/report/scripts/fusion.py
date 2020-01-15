#! /usr/bin/env python
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist


def and_gate(x, y):
    a = 2.28466
    b = -0.89817
    if x == 0 and y == 0:
        return 0
    else:
        return x*(1-np.exp(-((a*y**2+b*x*y)/(x**2+y**2)))) + y*(1-np.exp(-((a*x**2+b*x*y)/(x**2+y**2))))


def or_gate(x, y):
    a = 1.02889
    b = 0.3574
    if x == 0 and y == 0:
        return 0
    else:
        return x*(np.exp(-((a*y**2+b*x*y)/(x**2+y**2)))) + y*(np.exp(-((a*x**2+b*x*y)/(x**2+y**2))))


def invoke_gate(x, y):
    return and_gate(or_gate(x, y), x)


def prevail_gate(x, y):
    return or_gate(x, or_gate(x, y))


class Fusion:

    def __init__(self):
        # the final position
        #home_max_values = np.array([float(sys.argv[1]), float(sys.argv[2])])
        self.controlled_robot = sys.argv[1]

        self.CA_rot = 0.0
        self.HO_rot = 0.0
        self.FS_rot = 0.0
        self.KB_rot = 0.0

        self.CA_lin = 0.0
        self.dist_to_target = 2.0
        self.HO_lin = 0.0
        self.FS_lin = 0.0
        self.KB_lin = 0.0

        self.last_KB_callback_time = rospy.get_time()

        if self.controlled_robot == "cat":
            # subscribers:
            rospy.Subscriber("HO_signal_cat", Twist, self.HO_callback)
            rospy.Subscriber("CA_signal_cat", Twist, self.CA_callback)
            rospy.Subscriber("FS_signal_cat", Twist, self.FS_callback)
            rospy.Subscriber("KB_signal_cat", Twist, self.KB_callback)

            self.pup = rospy.Publisher(
                "/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

            self.lin_gain = 1.0
            self.rot_gain = 1.0
            self.lin_max = 1.0
            self.rot_max = .9

        elif self.controlled_robot == "mouse":
            # subscribers:
            rospy.Subscriber("HO_signal_mouse", Twist, self.HO_callback)
            rospy.Subscriber("CA_signal_mouse", Twist, self.CA_callback)
            rospy.Subscriber("FS_signal_mouse", Twist, self.FS_callback)
            rospy.Subscriber("KB_signal_mouse", Twist, self.KB_callback)

            self.pup = rospy.Publisher(
                "/mouse/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

            self.lin_gain = 1.0
            self.rot_gain = 1.0
            self.lin_max = .9
            self.rot_max = 1.0

        # rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=KB_signal

        # self.col_avoid_max_speed = float(sys.argv[1])
        #self.col_avoid_max_rot = float(sys.argv[2])
        #self.home_max_speed = float(sys.argv[3])
        #self.home_max_rot = float(sys.argv[4])

        while not rospy.is_shutdown():

            if self.controlled_robot == "cat":
                final_signal = self.combine_signals_cat()
                #final_signal.angular.z = self.CA_rot
                #final_signal.linear.x = self.CA_lin
                #final_signal.angular.z = self.HO_rot
                #final_signal.linear.x = 0.0
            else:
                final_signal = self.combine_signals_mouse()

            final_signal.angular.z = np.clip(
                final_signal.angular.z*self.rot_gain, -self.rot_max, self.rot_max)
            final_signal.linear.x = np.clip(
                final_signal.linear.x*self.lin_gain, -self.lin_max, self.lin_max)
            self.pup.publish(final_signal)

            if rospy.get_time() > self.last_KB_callback_time + .1:
                self.KB_lin = 0.0
                self.KB_rot = 0.0

    def HO_callback(self, signals):
        self.HO_rot = signals.angular.z
        self.dist_to_target = signals.linear.z
        self.HO_lin = signals.linear.x

    def CA_callback(self, signals):
        self.CA_rot = signals.angular.z
        self.CA_lin = signals.linear.x

    def FS_callback(self, signals):
        self.FS_rot = signals.angular.z
        self.FS_lin = signals.linear.x

    def KB_callback(self, signals):
        self.KB_rot = signals.angular.z
        self.KB_lin = signals.linear.x
        self.last_KB_callback_time = rospy.get_time()

    def combine_signals_cat(self):
        res_signal = Twist()

        # ignore collision-signal if we are close anouth
        if self.dist_to_target < .6:
            self.CA_rot = 0.0
            self.CA_lin = self.HO_lin

        res_signal.angular.z = prevail_gate(self.CA_rot,
                                            np.clip(or_gate(10.0*self.HO_rot, self.FS_rot), -1, 1))

        res_signal.linear.x = self.CA_lin

        return res_signal

    def combine_signals_mouse(self):
        res_signal = Twist()

        # if np.sign(self.FS_rot) != np.sign(self.HO_rot):
        #    print("no Homing")
        #    self.HO_rot = 0.0

        res_signal.angular.z = prevail_gate(self.CA_rot,
                                            np.clip(or_gate(10.0*self.HO_rot, self.FS_rot), -1, 1))

        res_signal.linear.x = self.CA_lin

        return res_signal


if __name__ == "__main__":
    rospy.init_node("fusion")
    try:
        node = Fusion()

    except rospy.ROSInterruptException:
        pass
