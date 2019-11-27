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
        rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom", Odometry, self.velocity_callback)
        rospy.Subscriber("/robotic_games/sonar", PointCloud, self.sonar_callback)

        '''
        Das Ergebniss der Berechnung wird dem Roboter als soll-geschwindigkeit zurueckgegeben.
        dies passiert ueber einen sogenannten Publisher
        (siehe wieder http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 )
        geregelt, der Name des Topics wird dabei von der Simulation vorgegeben.
        '''
        self.col_avoid_publisher = rospy.Publisher("/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

        rospy.spin()

    def velocity_callback(self, current_odometry):
        self.current_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_z = current_odometry.twist.twist.angular.z

    def sonar_callback(self, current_sonar_scan):
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonar_points = current_sonar_scan.points
        # Die Orientierung der einzelnen Sensoren folgt:
        sonar_angles = np.array([-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        sonar_angles = sonar_angles / 360.0 * 2 * np.pi

        #berechnung des Abstands
        sonar_ranges = np.zeros(len(sonar_angles))
        for i in range(0, len(sonar_angles)):
            sonar_ranges[i] = np.sqrt(sonar_points[i].x**2 + sonar_points[i].y**2)

        #Kraft welche auf Roboter wirkt
        force=self.calculate_force(sonar_angles,sonar_ranges)
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
        velocity_adjustment.linear.x  = 1-force[0]
        velocity_adjustment.angular.z = force[1]
        self.col_avoid_publisher.publish(velocity_adjustment)

    def calculate_force(self,sonar_angles,sonar_ranges):
        '''
        HIER KOMMT DER CODE HIN

        Das Ergebniss der Berechnungen soll in Form eines 2-dimensionalen
        Arrays zurueckgegeben werden.
        Die 1. Komponente entspricht dabei der Aenderung der Lineargeschwindigkeit
        und die 2. enstprechend der Aenderung der Winkelgeschwindigkeit.
        '''
        CHANGE_ANGLE_TRESHOLD = 0.5   # in meter
        sonar_weights = np.array([0.1, 0.4, 0.9, 1.3, 1.3, 0.9, 0.4, 0.1])
        
        force = np.zeros(2)
        # calculate smallest distance
        smallest_distance = np.amin(sonar_ranges)
        # check from which angle this small range comes from
        indices_smallest_angle = np.argmin(sonar_ranges)
        id_smallest_angle = indices_smallest_angle
        # change angle if it's below a certain treshold
        if (smallest_distance <= CHANGE_ANGLE_TRESHOLD):
            if id_smallest_angle <= 3:
                force[1] = -self.weight_anglechange_by_distance(smallest_distance)
            else:
                force[1] = self.weight_anglechange_by_distance(smallest_distance)
        
        rospy.loginfo('indice of the smallest angle: ' + str(id_smallest_angle))
        rospy.loginfo('distance: ' + str(smallest_distance))
        rospy.loginfo('delta angle' + str(force[1]))
        
        # force[0] = self.check_if_object_is_front(sonar_ranges, smallest_distance)
        force[0] = self.weight_velocitychange_by_distance(smallest_distance)
        return force

    def weight_anglechange_by_distance(self, distance):
        factor = 3.5 # big=smallChange small=bigChange
        new_angle = 1/(distance*factor) if 1/(distance*factor)<3.5 else 3.5
        return new_angle
        
    def weight_velocitychange_by_distance(self, distance):
        factor = 5.5 #  1/ (0.2*5.5) = 0.9
        vel_change = 1/(distance*factor) if 1/(distance*factor)<1 else 1
        return vel_change
    
    # returns velocity change
    def check_if_object_is_front(self, sonar_ranges, smallest_distance):
        stop_treshold = 0.1
        if sonar_ranges[3] < stop_treshold and sonar_ranges[4] < stop_treshold:
            return 1



if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")

    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass
