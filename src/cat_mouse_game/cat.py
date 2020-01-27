#! /usr/bin/env python
import rospy

from Packages.GameTheory.path_prediction import angle_change_cat_scaled
from Packages.AnalogGates.analog_gates import prevail_gate

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from Packages.GameTheory.path_prediction import CAT_MAX_SPEED
from Packages.GameTheory.path_prediction import CAT_MAX_ANGLE

VELOCITY_FACTOR = 2  # remove when wheelradius is fixed in pd3x
UPDATE_PREDICTION_DELTA = 0.1  # in s


class Cat:

    def __init__(self):
        self.position = [0, 0]
        self.orientation = [0]
        self.mouse_position = [0, 0]
        self.mouse_orientation = [0]

        self.CA_rot = 0.0

        rospy.Subscriber("/cat/dead_reckoning", Pose,
                         self.dead_reckoning_callback_cat)
        rospy.Subscriber("/mouse/dead_reckoning", Pose,
                         self.dead_reckoning_callback_mouse)

        rospy.Subscriber("CA_signal_cat", Twist, self.CA_callback)

        pub = rospy.Publisher(
            "/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        output = Twist()
        speed = 0.7
        output.linear.x = CAT_MAX_SPEED

        while not rospy.is_shutdown():
            GT_rot = angle_change_cat_scaled(pos_cat=self.position, z_cat=self.orientation[0],
                                             pos_mouse=self.mouse_position, z_mouse=self.mouse_orientation[0], update_time=1.0)

            output.angular.z = prevail_gate(self.CA_rot, GT_rot)*CAT_MAX_ANGLE

            rospy.loginfo('cat: ' + str(output.angular.z))

            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()
            # TODO remove VELOCITY_FACTOR when fixed
            while(t1-t0 < UPDATE_PREDICTION_DELTA*VELOCITY_FACTOR):
                pub.publish(output)
                t1 = rospy.Time.now().to_sec()

            pub.publish(output)

    def dead_reckoning_callback_cat(self, msg):
        self.position[0] = msg.position.x
        self.position[1] = msg.position.y
        self.orientation[0] = msg.orientation.z
        #rospy.loginfo("x: " + str(self.position[0]) + " y: " + str(self.position[1]) + ' z: ' + str(self.orientation[0]))

    def dead_reckoning_callback_mouse(self, msg):
        self.mouse_position[0] = msg.position.x
        self.mouse_position[1] = msg.position.y
        self.mouse_orientation[0] = msg.orientation.z

    def CA_callback(self, signals):
        self.CA_rot = signals.angular.z
        #self.CA_lin = signals.linear.x


if __name__ == "__main__":
    rospy.init_node("cat")
    try:
        node = Cat()

    except rospy.ROSInterruptException:
        pass
