#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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

        '''
        Die verwendung eines Kraftbasierten Ansatzes bedeutet, dass die momentane Geschwindigkeit
        modifiziert wird.
        Dazu muss sie allerdings zunaechst bekannt sein.
        Der Roboter in der Simulation stellt diese bereits ueber einen sogenannten Subscriber
        zur verfuegung. Ein Tutorium ist auf folgender Website zu finden:
        http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
        '''
        rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom",
                         Odometry, self.velocity_callback)
        rospy.Subscriber("/robotic_games/sonar",
                         PointCloud, self.sonar_callback)

        '''
        Das Ergebniss der Berechnung wird dem Roboter als soll-geschwindigkeit zurueckgegeben.
        dies passiert ueber einen sogenannten Publisher
        (siehe wieder http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 )
        geregelt, der Name des Topics wird dabei von der Simulation vorgegeben.
        '''
        self.col_avoid_publisher = rospy.Publisher(
            "/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

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
        '''
        ROS kommuniziert ueber feste vordefinierte Datenpakete, Messages genannt.
        fuer Geschwindigkeiten wird dafuer haeufig der sogenannte Twist verwendet.

        Naehere informationen zum Twist sind unter:
        https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
        zu finden.

        Da es sich hier um ein 2-Dimensionales System handelt, wird lediglich die linear-
        Geschwindigkeit in x und die Winkelgeschwindigkeit in z Richtung verwendet.
        '''
        velocity_adjustment = Twist()

        p_rot_speed = 1.2
        p_lin_speed = .1
        max_lin_speed = .40
        min_range = .5
        default_speed = 0.2

        v_view = np.array([1, 0])
        force_strength = np.linalg.norm(force)
        rot_error = np.dot(force/force_strength, v_view)
        rot_error = (-.5*rot_error+.5)  # linear interpolate [-1,1] to [1,0]

        if np.min(sonar_ranges) < min_range:
            velocity_adjustment.linear.x = np.clip(
                p_lin_speed*(1/(force_strength)), 0.0, max_lin_speed)
            velocity_adjustment.angular.z = np.sign(
                -force[1])*p_rot_speed*rot_error
        else:
            velocity_adjustment.linear.x = default_speed
            velocity_adjustment.angular.z = 0.0

        self.col_avoid_publisher.publish(velocity_adjustment)

    def calculate_force(self, sonar_angles, sonar_ranges):
        '''
        HIER KOMMT DER CODE HIN

        Das Ergebniss der Berechnungen soll in Form eines 2-dimensionalen
        Arrays zurueckgegeben werden.
        Die 1. Komponente entspricht dabei der Aenderung der Lineargeschwindigkeit
        und die 2. enstprechend der Aenderung der Winkelgeschwindigkeit.
        '''
        # todo: tunnelblick wenn schnell unterwegs
        w_focus_to_front = np.array([.1, .8, .8, 1, 1, .8, .8, .1])

        F = np.zeros(2)
        for i in range(0, len(sonar_angles)):

            y = np.sin(sonar_angles[i])
            x = np.cos(sonar_angles[i])
            e = np.array([x, y])  # length(e)=1
            l = sonar_ranges[i]
            f = -e/(l**2)
            F = F + f

        return F/8


if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")

    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass
