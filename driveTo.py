#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Import String message type

class Robot:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.target_data = None  # Store received data

        # Subscriber for /processed_aoa_data
        rospy.Subscriber("/processed_aoa_data", String, self.callback)

    def move_to_distance(self, distance):
        outData = Twist()
        t0 = rospy.get_time()
        current_distance = 0

        outData.linear.x = 0.2 if distance > 0 else -0.2
        while not rospy.is_shutdown() and abs(current_distance) < abs(distance):
            self.vel_pub.publish(outData)
            t1 = rospy.get_time()
            current_distance = 0.2 * (t1 - t0)
            self.rate.sleep()
        
        outData.linear.x = 0
        self.vel_pub.publish(outData)

    def rotate_to_angle(self, target_angle):
        outData = Twist()
        t0 = rospy.get_time()
        current_angle = 0
        outData.angular.z = -0.3 if target_angle < 0 else 0.3
        while not rospy.is_shutdown() and abs(current_angle) < abs(target_angle):
            self.vel_pub.publish(outData)
            t1 = rospy.get_time()
            current_angle = 0.3 * (t1 - t0)
            self.rate.sleep()
        
        outData.angular.z = 0
        self.vel_pub.publish(outData)

    def go_to_target(self, target_angle, target_distance):
        self.rotate_to_angle(target_angle)
        self.move_to_distance(target_distance)

    def callback(self, msg):
        """ Callback function to process incoming data """
        self.target_data = msg.data

if __name__ == '__main__':
    try:
        myRobot = Robot()
        rospy.loginfo("Robot is ready and waiting for target data.")

        while not rospy.is_shutdown():
            if myRobot.target_data:
                try:
                    data_list = myRobot.target_data.split(",")
                    target_angle = -float(data_list[3])  # Convert to float
                    target_distance = float(data_list[1])  # Convert to float
                    rospy.loginfo(f"Moving to angle: {target_angle}, distance: {target_distance}")
                    myRobot.go_to_target(target_angle, target_distance)
                    
                    myRobot.target_data = None  # Reset data after processing
                except (IndexError, ValueError) as e:
                    rospy.logerr(f"Error processing data: {e}")

            myRobot.rate.sleep()

    except rospy.ROSInterruptException:
        pass
