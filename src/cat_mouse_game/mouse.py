#! /usr/bin/env python
import numpy as np
import rospy
import sys

from Packages.GameTheory.path_prediction import angle_change_mouse_scaled
from Packages.AnalogGates.analog_gates import prevail_gate

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


VELOCITY_FACTOR = 2  # remove when wheelradius is fixed in pd3x
UPDATE_PREDICTION_DELTA = .1  # in s


class Mouse:

    def __init__(self):
        self.position = [0, 0]
        self.orientation = [0]
        self.cat_position = [0, 0]
        self.cat_orientation = [0]
        self.CA_rot = 0.0

        rospy.Subscriber("/cat/dead_reckoning", Pose,
                         self.dead_reckoning_callback_cat)
        rospy.Subscriber("/mouse/dead_reckoning", Pose,
                         self.dead_reckoning_callback_mouse)
        rospy.Subscriber("CA_signal_mouse", Twist, self.CA_callback)
        pub = rospy.Publisher(
            "/mouse/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        output = Twist()
        speed = 0.2
        output.linear.x = speed

        while not rospy.is_shutdown():
            GT_rot = angle_change_mouse_scaled(pos_mouse=self.position, z_mouse=self.orientation[0],
                                               pos_cat=self.cat_position, z_cat=self.cat_orientation[0], update_time=1.0)

            output.angular.z = prevail_gate(self.CA_rot, GT_rot)

            rospy.loginfo('mouse: ' + str(output.angular.z))

            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()
            # TODO remove VELOCITY_FACTOR when fixed
            while(t1-t0 < UPDATE_PREDICTION_DELTA*VELOCITY_FACTOR):
                pub.publish(output)
                t1 = rospy.Time.now().to_sec()

            pub.publish(output)

    def dead_reckoning_callback_cat(self, msg):
        self.cat_position[0] = msg.position.x
        self.cat_position[1] = msg.position.y
        self.cat_orientation[0] = msg.orientation.z

    def dead_reckoning_callback_mouse(self, msg):
        self.position[0] = msg.position.x
        self.position[1] = msg.position.y
        self.orientation[0] = msg.orientation.z

    def CA_callback(self, signals):
        self.CA_rot = signals.angular.z
        # print(self.CA_rot)
        #self.CA_lin = signals.linear.x


def closed_loop(position, orientation, target):
    velocity_adjustment = Twist()

    res = hm.homing(position, orientation, target)

    # default values
    velocity_adjustment.linear.x = res[0]
    velocity_adjustment.angular.z = res[1]
    return velocity_adjustment


if __name__ == "__main__":
    rospy.init_node("mouse")
    try:
        node = Mouse()
    except rospy.ROSInterruptException:
        pass
