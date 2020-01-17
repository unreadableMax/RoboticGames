#! /usr/bin/env python
import rospy

from path_prediction import maxmin_solution_angle

from path_length import get_path_length
import numpy as np

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


VELOCITY_FACTOR = 2  # remove when wheelradius is fixed in pd3x
UPDATE_PREDICTION_DELTA = 1  # in s

v = 0.7
omega = 0.2


class Cat:

    def __init__(self):
        self.position = [0, 0]
        self.orientation = [0]
        self.mouse_position = [0, 0]
        self.mouse_orientation = [0]
        rospy.Subscriber("/cat/dead_reckoning", Pose,
                         self.dead_reckoning_callback_cat)
        rospy.Subscriber("/mouse/dead_reckoning", Pose,
                         self.dead_reckoning_callback_mouse)
        pub = rospy.Publisher(
            "/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        output = Twist()
        speed = 0.7
        output.linear.x = speed

        # new:
        self.r = v/omega
        print("r=", self.r)

        while not rospy.is_shutdown():
            output.angular.z = maxmin_solution_angle(pos_cat=self.position, z_cat=self.orientation[0],
                                                     pos_mouse=self.mouse_position, z_mouse=self.mouse_orientation[
                                                         0],
                                                     mouse_or_cat='cat', update_time=1.0)
            rospy.loginfo(output.angular.z)

            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()
            # TODO remove VELOCITY_FACTOR when fixed
            while(t1-t0 < UPDATE_PREDICTION_DELTA*VELOCITY_FACTOR):
                # pub.publish(output)
                # test:
                l = get_path_length(
                    self.r, self.orientation[0], self.position, self.mouse_position)

                t1 = rospy.Time.now().to_sec()

            # pub.publish(output)

    def dead_reckoning_callback_cat(self, msg):
        self.position[0] = msg.position.x
        self.position[1] = msg.position.y
        self.orientation[0] = msg.orientation.z
        #rospy.loginfo("x: " + str(self.position[0]) + " y: " + str(self.position[1]) + ' z: ' + str(self.orientation[0]))

    def dead_reckoning_callback_mouse(self, msg):
        self.mouse_position[0] = msg.position.x
        self.mouse_position[1] = msg.position.y
        self.mouse_orientation[0] = msg.orientation.z


if __name__ == "__main__":
    rospy.init_node("cat")
    try:
        node = Cat()

    except rospy.ROSInterruptException:
        pass
