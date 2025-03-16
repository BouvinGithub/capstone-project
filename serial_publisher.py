#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String

def serial_publisher():
    rospy.init_node('serial_publisher_node', anonymous=True)
    pub = rospy.Publisher('/aoa_data', String, queue_size=10)
    ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8").strip()
            print(line)
            pub.publish(line)
        rate.sleep()
if __name__ == '__main__':
    try:
        serial_publisher()
    except rospy.ROSInterruptException:
        pass
