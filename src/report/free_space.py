#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

'''
Die gesammte Kollisionsvermeidung ist in einer Klasse verpackt um die momentanigen Geschwindigkeitsdaten aus dem callback des
Geschwindigkeitssubscribers herauszuholen und diese in der berechnung der neuen Richtgeschwindigkeit zu benutzen.
alternativ haetten hier auch globale Variabeln verwendet werden koennen, diese Methode wird in der Community allerdings als
eleganter angesehen.
'''


class CollisionAvoidance:

    def __init__(self):

        self.current_vel_x = 0.0
        self.current_ang_z = 0.0

        # sonar stuff:
        self.sonar_angles = np.array(
            [-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        self.sonar_angles = self.sonar_angles / 360.0 * 2 * np.pi
        self.sonar_ranges = np.ones(len(self.sonar_angles))*10

        # initialize 360 scan-vecs:
        self.compare_vecs = np.array([[1, 0], [1, 1]/np.sqrt(2), [0, 1], [-1, 1]/np.sqrt(
            2), [-1, 0], [-1, -1]/np.sqrt(2), [0, -1], [1, -1]/np.sqrt(2)])
        self.initial_distance = 10
        self.scan_ranges = np.array([self.initial_distance, self.initial_distance, self.initial_distance,
                                     self.initial_distance, self.initial_distance, self.initial_distance, self.initial_distance, self.initial_distance])

        rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom",
                         Odometry, self.velocity_callback)
        rospy.Subscriber("cat/sonar",
                         PointCloud, self.sonar_callback)

        self.free_space_puplisher = rospy.Publisher(
            "FS_signal", Twist, queue_size=10)

        rospy.spin()

    def velocity_callback(self, current_odometry):
        self.current_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_z = current_odometry.twist.twist.angular.z

    def sonar_callback(self, current_sonar_scan):
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonar_points = current_sonar_scan.points
        # Die Orientierung der einzelnen Sensoren folgt:

        # berechnung des Abstands
        self.last_sonar_ranges = self.sonar_ranges
        for i in range(0, len(self.sonar_angles)):
            self.sonar_ranges[i] = np.sqrt(
                sonar_points[i].x**2 + sonar_points[i].y**2)

        velocity_adjustment = Twist()

        p_lin_speed = 1.0
        max_lin_speed = 1.0
        max_ang_speed = 1.0
        w = np.array([1.0, 1.0, 1.0])  # min,max,sum

        min_left = w[0]/np.min(self.sonar_ranges[0:3])
        max_right = w[1]*np.max(self.sonar_ranges[0:3])
        average_left = w[2]*np.sum(self.sonar_ranges[0:3])/4.0

        min_right = w[0]/np.min(self.sonar_ranges[4:7])
        max_left = w[1] * np.max(self.sonar_ranges[4:7])
        average_right = w[2]*np.sum(self.sonar_ranges[4:7])/4.0

        attraction_to_left = min_left + average_left + max_left
        attraction_to_right = min_right + average_right + max_right

        velocity_adjustment.linear.x = np.clip(p_lin_speed *
                                               (np.min(self.sonar_ranges)-.1), 0, max_lin_speed)
        velocity_adjustment.angular.z = np.clip(
            attraction_to_right - attraction_to_left, -max_ang_speed, max_ang_speed)

        # velocity_adjustment.linear.x = default_lin_speed
        # velocity_adjustment.angular.z = np.sign(
        #    -force[1])*p_rot_speed*rot_error

        self.free_space_puplisher.publish(velocity_adjustment)


if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")

    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass
