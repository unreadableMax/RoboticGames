#! /usr/bin/env python
import rospy

from path_prediction import max_min_angle_cat

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

# remove when wheelradius is fixed in pd3x
VELOCITY_FACTOR = 2

class Test:
    
    def __init__(self):
        self.position  = [0, 0]
        self.orientation = [0]
        rospy.Subscriber("/cat/dead_reckoning", Pose, self.dead_reckoning_callback)
        pub = rospy.Publisher("/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        output = Twist()
        speed = 0.7
        output.linear.x = speed
        
        while not rospy.is_shutdown():
            #rospy.loginfo("x: " + str(self.position[0]) + " y: " + str(self.position[1]) + ' z: ' + str(self.orientation[0]))
            output.angular.z = max_min_angle_cat(self.position, self.orientation[0], [-7, -1], 0)
            rospy.loginfo(output.angular.z)
            
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()
            
            # TODO remove VELOCITY_FACTOR when fixed
            while(t1-t0 < 1 * VELOCITY_FACTOR):
                pub.publish(output)
                t1 = rospy.Time.now().to_sec()
            
            pub.publish(output)
            
    
    
    def dead_reckoning_callback(self, msg):
        self.position[0] = msg.position.x
        self.position[1] = msg.position.y
        self.orientation[0] = msg.orientation.z

if  __name__=="__main__":
    rospy.init_node("homing")
    try:
        node=Test()
    
    except rospy.ROSInterruptException:
        pass
