#! /usr/bin/env python
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


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

        self.CA_rot = 0.0
        self.HO_rot = 0.0
        self.FS_rot = 0.0
        self.KB_rot = 0.0

        self.CA_lin = 0.0
        self.HO_lin = 0.0
        self.FS_lin = 0.0
        self.KB_lin = 0.0

        self.lin_gain = 1.0
        self.lin_max = 1.0
        self.rot_gain = 1.0
        self.rot_max = 1.0

        self.last_KB_callback_time = rospy.get_time()

        # subscribers:
        rospy.Subscriber("home_signal", Twist, self.HO_callback)
        rospy.Subscriber("col_avoid_signal", Twist, self.CA_callback)
        rospy.Subscriber("FS_signal", Twist, self.FS_callback)
        rospy.Subscriber("KB_signal", Twist, self.KB_callback)

        # rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=KB_signal

        # self.col_avoid_max_speed = float(sys.argv[1])
        #self.col_avoid_max_rot = float(sys.argv[2])
        #self.home_max_speed = float(sys.argv[3])
        #self.home_max_rot = float(sys.argv[4])

        pup = rospy.Publisher(
            "/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():

            final_signal = self.combine_signals()
            #final_signal.angular.z = self.CA_rot
            #final_signal.linear.x = self.CA_lin

            final_signal.angular.z = np.clip(
                final_signal.angular.z*self.rot_gain, -self.rot_max, self.rot_max)
            final_signal.linear.x = np.clip(
                final_signal.linear.x*self.lin_gain, -self.lin_max, self.lin_max)
            pup.publish(final_signal)

            if rospy.get_time() > self.last_KB_callback_time + .1:
                self.KB_lin = 0.0
                self.KB_rot = 0.0

    def HO_callback(self, signals):
        self.HO_rot = signals.angular.z
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

    def combine_signals(self):
        res_signal = Twist()
        # todo: wand-folger in CA rein kriegen
        res_signal.angular.z = prevail_gate(self.CA_rot,
                                            or_gate(2.0*self.HO_rot, self.FS_rot))

        res_signal.linear.x = self.CA_lin

        return res_signal


if __name__ == "__main__":
    rospy.init_node("fusion")
    try:
        node = Fusion()

    except rospy.ROSInterruptException:
        pass
