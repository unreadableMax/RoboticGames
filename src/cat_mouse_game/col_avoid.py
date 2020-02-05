#!/usr/bin/env python

import numpy as np
import rospy
import sys
import math
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

from Packages.GameTheory.game_theory import r_cat
from Packages.GameTheory.game_theory import r_mouse


class CollisionAvoidance:

    def __init__(self):

        self.vel_x = 0.0
        self.rot_vel = 0.0
        self.d_c = .3
        self.d_s = .4
        self.d_offset = .4
        self.robot_radius = .1
        self.r = 0
        self.pos = np.array([0.0,0.0])
        self.orientation = 0.0
        self.target_pos = np.array([0.0,0.0])
        

        # True -> we are the cat. False -> we are the mouse
        self.controlled_robot = sys.argv[1]

        rospy.Subscriber("/cat/dead_reckoning", Pose,
                         self.dead_reckoning_callback_cat)
        rospy.Subscriber("/mouse/dead_reckoning", Pose,
                         self.dead_reckoning_callback_mouse)

        if self.controlled_robot == "cat":
            rospy.Subscriber("cat/sonar", PointCloud, self.sonar_callback)
            # uncomment this if u need velocity-information of the robot:
            # rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom",
            #                 Odometry, self.velocity_callback)

            self.CA_puplisher = rospy.Publisher(
                "CA_signal_cat", Twist, queue_size=10)

            self.r = r_cat
            self.d_c = r_cat + self.robot_radius #critical distance -> full CA
            self.d_s = self.d_c + self.d_offset # start distance -> start with CA
        elif self.controlled_robot == "mouse":
            rospy.Subscriber("mouse/sonar", PointCloud,
                             self.sonar_callback)
            self.CA_puplisher = rospy.Publisher(
                "CA_signal_mouse", Twist, queue_size=10)
            self.d_c = r_mouse + self.robot_radius
            self.d_s = self.d_c + self.d_offset
            self.r = r_mouse

        rospy.spin()
    
    def dead_reckoning_callback_cat(self, msg):
        if self.controlled_robot == "cat":
            self.pos[0] = msg.position.x
            self.pos[1] = msg.position.y
            self.orientation = msg.orientation.z
        else:
            self.target_pos[0] = msg.position.x
            self.target_pos[1] = msg.position.y

    def dead_reckoning_callback_mouse(self, msg):
        if self.controlled_robot == "mouse":
            self.pos[0] = msg.position.x
            self.pos[1] = msg.position.y
            self.orientation = msg.orientation.z
        else:
            self.target_pos[0] = msg.position.x
            self.target_pos[1] = msg.position.y

    def velocity_callback(self, current_odometry):
        self.vel_x = current_odometry.twist.twist.linear.x
        self.rot_vel = current_odometry.twist.twist.angular.z

    def sonar_callback(self, current_sonar_scan):
        #signal = self.calc_CA_signal_via_turningcycle(current_sonar_scan)
        signal = self.calc_CA_signal_via_force(current_sonar_scan)
        self.CA_puplisher.publish(signal)

    def calc_CA_signal_via_force(self, current_sonar_scan):
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

        velocity_adjustment = Twist()
        
        smallest_dist = np.min(sonar_ranges)

        if smallest_dist > self.d_s:
            velocity_adjustment.angular.z = 0.0
            return velocity_adjustment

        # Kraft welche auf Roboter wirkt
        force = self.calculate_force(sonar_angles, sonar_ranges)

        w = 1.0
        if self.does_other_bot_produce_force(force,smallest_dist):
                velocity_adjustment.angular.z = 0.0
                if self.controlled_robot == "cat":
                    return velocity_adjustment
                else:
                    w = .3

        turn_direction = np.sign(-force[1])
        
        m = -1.0/(self.d_s-self.d_c)
        adjust_power = m*(smallest_dist-self.d_s)*w

        velocity_adjustment.angular.z = turn_direction * adjust_power

        velocity_adjustment.angular.z = np.clip(
            velocity_adjustment.angular.z, -1, 1)

        return velocity_adjustment

    def does_other_bot_produce_force(self,force,smallest_distance):
        force_strength = np.linalg.norm(force)

        v_target = self.target_pos - self.pos
        # calculate v_target, relative to robot:
        ca = np.cos(self.orientation)
        sa = np.sin(self.orientation)
        R = np.array([[ca, -sa],[sa, ca]])
        v_target = np.asarray(np.dot(R.T, v_target).T)

        dist2target = np.linalg.norm(v_target)
        ev_target = np.array(v_target/dist2target)

        ef = np.array(force/force_strength)
        #print(np.dot(-ef,ev_target))
        # print(np.dot(-ef,ev_target))
        if dist2target-self.robot_radius-.1 < self.d_s:
            if np.dot(-ef,ev_target) > .95:
                print(np.dot(-ef,ev_target))
                return True
        
        return False

    def calc_CA_signal_via_turningcycle(self, current_sonar_scan):
        velocity_adjustment = Twist()
        buffered_radius = self.robot_radius + self.r
        left_overlap_count = 0
        right_overlap_count = 0
        p = self.get_sonar_as_10_points(current_sonar_scan)
        w = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,1.0,1.0]
        for i in range(0, 9):

            if i <= 4:
                v_r = p[i] - np.array([0, -self.r])
                diff = buffered_radius - np.linalg.norm(v_r)
                if diff > 0:
                    right_overlap_count = right_overlap_count + diff*w[i]

            else:
                v_r = p[i] - np.array([0, self.r])
                diff = buffered_radius - np.linalg.norm(v_r)
                if diff > 0:
                    left_overlap_count = left_overlap_count+diff*w[i]

        
        if left_overlap_count == right_overlap_count:
            velocity_adjustment.angular.z = 0.0
        elif left_overlap_count > right_overlap_count:
            velocity_adjustment.angular.z = 1.0
        else:
            velocity_adjustment.angular.z = -1.0

        return velocity_adjustment

    def get_sonar_as_10_points(self,current_sonar_scan):
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

        p = np.array([[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0]])
        for i in range(0, len(sonar_angles)):

            # calculate normalized vectors:
            y = np.sin(sonar_angles[i])
            x = np.cos(sonar_angles[i])
            p[i] = np.array([x, y])*sonar_ranges[i]

        p01 = p[1]+(p[0]-p[1])/2.0
        p67 = p[6]+(p[7]-p[6])/2.0

        p = np.array([p[0],p01,p[1],p[2],p[3],p[4],p[5],p[6],p67,p[7]])

        return p

    def get_sonar_as_8_points(self,current_sonar_scan):
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

        p = np.array([[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0],[.0,.0]])
        for i in range(0, len(sonar_angles)):

            # calculate normalized vectors:
            y = np.sin(sonar_angles[i])
            x = np.cos(sonar_angles[i])
            p[i] = np.array([x, y])*sonar_ranges[i]

        return p

    def calculate_force(self, sonar_angles, sonar_ranges):

        force = np.zeros(2)
        w = [1.0, 1.0, .5, 1.0, 1.0, .5, 1.0, 1.0]
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
