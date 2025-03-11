#!/usr/bin/env python3

import rospy, math
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
        # to recieve updated values before completing job
        rospy.Timer(rospy.Duration(3), self.process_target_data)

    def move_to_distance(self, distance):
        outData = Twist()
        t0 = rospy.get_rostime().secs
        
        while not rospy.is_shutdown() and t0 == 0:
            t0 = rospy.get_rostime().secs
        
        current_distance = 0

        if distance < 1 and distance > -1:
            outData.linear.x = 0
        elif distance > 0:
            outData.linear.x = 0.2 
        else:
            outData.linear.x = -0.2
        
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

        while not rospy.is_shutdown() and t0 == 0:
            t0 = rospy.get_rostime().secs
        
        current_angle = 0

        if target_angle < 0:
            outData.angular.z = -1 * degrees2radians(30)
        else:
            outData.angular.z = degrees2radians(30)

        while not rospy.is_shutdown() and abs(current_angle) < abs(target_angle):
            self.vel_pub.publish(outData)
            t1 = rospy.get_rostime().secs
            current_angle = degreesToRadians(33) * (t1 - t0)
        
        self.rate.sleep()
        outData.angular.z = 0
        self.vel_pub.publish(outData)

    def go_to_target(self, target_angle, target_distance):
        self.rotate_to_angle(target_angle)
        self.move_to_distance(target_distance)

    def callback(self, msg):
        """ Callback function to process incoming data """
        self.target_data = msg.data

    def degreesToRadians(degree):
        return degree * (math.pi/180.0)

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
