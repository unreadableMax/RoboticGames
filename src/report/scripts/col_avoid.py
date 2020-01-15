#!/usr/bin/env python

import numpy as np
import rospy
import sys
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class CollisionAvoidance:

    def __init__(self):

        self.vel_x = 0.0
        self.rot_vel = 0.0

        # True -> we are the cat. False -> we are the mouse
        self.controlled_robot = sys.argv[1]

        if self.controlled_robot == "cat":
            rospy.Subscriber("cat/sonar", PointCloud, self.sonar_callback)
            # uncomment this if u need velocity-information of the robot:
            # rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom",
            #                 Odometry, self.velocity_callback)
            self.CA_puplisher = rospy.Publisher(
                "CA_signal_cat", Twist, queue_size=10)
        elif self.controlled_robot == "mouse":
            rospy.Subscriber("mouse/sonar", PointCloud,
                             self.sonar_callback)
            self.CA_puplisher = rospy.Publisher(
                "CA_signal_mouse", Twist, queue_size=10)

        rospy.spin()

    def velocity_callback(self, current_odometry):
        self.vel_x = current_odometry.twist.twist.linear.x
        self.rot_vel = current_odometry.twist.twist.angular.z

    def sonar_callback(self, current_sonar_scan):
        signal = self.calc_CA_signal(current_sonar_scan)
        self.CA_puplisher.publish(signal)

    def calculate_force(self, sonar_angles, sonar_ranges):

        force = np.zeros(2)
        w = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        # if self.current_vel_x < .1:
        #    w = [.1, .1, .1, .1, 1, 1, 1, 1]
        for i in range(0, len(sonar_angles)):

            # calculate normalized vectors:
            y = np.sin(sonar_angles[i])
            x = np.cos(sonar_angles[i])
            v = np.array([-x, -y])  # length(v)=1
            l = sonar_ranges[i]
            force = force + w[i]*(1/l)*(1/l) * v

        force = force/8.0

        return force

    def calc_CA_signal(self, current_sonar_scan):
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonar_points = current_sonar_scan.points
        # Die Orientierung der einzelnen Sensoren folgt:
        sonar_angles = np.array(
            [-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        sonar_angles = sonar_angles / 360.0 * 2 * np.pi

        # berechnung des Abstands
        sonar_ranges = np.zeros(len(sonar_angles))
        for i in range(0, len(sonar_angles)):
            sonar_ranges[i] = np.sqrt(
                sonar_points[i].x**2 + sonar_points[i].y**2)

        # Kraft welche auf Roboter wirkt
        force = self.calculate_force(sonar_angles, sonar_ranges)

        velocity_adjustment = Twist()

        p_rot_speed = 1.2  # 1.2
        max_lin_speed = 1.0  # .40
        max_rot_speed = 1.0
        min_range = .2  # 0.5
        max_range = .3
        #current_smallest_dist = np.min(sonar_ranges[1:6])
        current_smallest_dist = np.min(sonar_ranges)
        default_speed = 1.0  # 0.4

        v_view = np.array([1, 0])
        force_strength = np.linalg.norm(force)
        rot_error = np.dot(force/force_strength, v_view)

        if current_smallest_dist < max_range:
            m = 1/(max_range-min_range)
            velocity_adjustment.linear.x = np.clip(
                m*(current_smallest_dist-min_range), -max_lin_speed, max_lin_speed)

            turn_direction = -force[1]
            velocity_adjustment.angular.z = np.sign(
                turn_direction) * np.clip(p_rot_speed*force_strength*(-.5*rot_error+.5), 0, max_rot_speed)

        else:
            velocity_adjustment.linear.x = np.clip(
                default_speed, 0, max_lin_speed)
            velocity_adjustment.angular.z = 0
        #velocity_adjustment.angular.z = 0
        # self.cat_CA_puplisher.publish(velocity_adjustment)
        return velocity_adjustment


if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")

    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass
