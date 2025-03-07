#!/usr/bin/env python3

#how to communicate with other file
#import?
import rospy, math
from geometry_msgs.msg import Twist

class Robot:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

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

    def callback(data):
        try:
            data = msg.data

if __name__ == '__main__':
    rospy.init_node("robot_drive")
    sub = rospy.Subscriber("/processed_aoa_data", String, callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    
    try:
        myRobot = Robot()
        data_list = data.split(",")
        target_angle = -data_list[3]  # Set desired angle in degrees
        target_distance = data_list[1] # Set desired distance in meters
        myRobot.go_to_target(target_angle, target_distance)
        
    except rospy.ROSInterruptException:
        pass
