#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class CollisionAvoidance:

    def __init__(self):

        self.current_vel_x = 0.0
        self.current_ang_z = 0.0

        rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom",
                         Odometry, self.velocity_callback)
        rospy.Subscriber("cat/sonar",
                         PointCloud, self.sonar_callback)

        self.col_avoid_publisher = rospy.Publisher(
            "col_avoid_signal", Twist, queue_size=10)

        rospy.spin()

    def velocity_callback(self, current_odometry):
        self.current_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_z = current_odometry.twist.twist.angular.z

    def sonar_callback(self, current_sonar_scan):
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
        max_range = .4
        #current_smallest_dist = np.min(sonar_ranges[1:6])
        current_smallest_dist = np.min(sonar_ranges)
        default_speed = 1.0  # 0.4

        v_view = np.array([1, 0])
        force_strength = np.linalg.norm(force)
        rot_error = np.dot(force/force_strength, v_view)

        if current_smallest_dist < max_range:
            m = 1/(max_range-min_range)
            velocity_adjustment.linear.x = np.clip(
                m*(current_smallest_dist-min_range), -0.0, max_lin_speed)

            turn_direction = -force[1]
            velocity_adjustment.angular.z = np.sign(
                turn_direction) * np.clip(p_rot_speed*force_strength*(-.5*rot_error+.5), 0, max_rot_speed)

        else:
            velocity_adjustment.linear.x = np.clip(
                default_speed, 0, max_lin_speed)
            velocity_adjustment.angular.z = 0
        #velocity_adjustment.angular.z = 0
        print("sending")
        self.col_avoid_publisher.publish(velocity_adjustment)

    def calculate_force(self, sonar_angles, sonar_ranges):

        force = np.zeros(2)
        w = [.5, 1, 0, .5, .5, 0, 1, .5]
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


if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")

    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass
