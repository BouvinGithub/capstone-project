#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String

def serial_publisher():
    rospy.init_node('serial_publisher_node', anonymous=True)
    pub = rospy.Publisher('/aoa_data', String, queue_size=10)
    ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
    rate = rospy.Rate(10)

    skip = 6
    count = 0
    try:
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8").strip()
                if count <= skip:
                    pub.publish(line)
                else:
                    count = 0
                count += 1
            rate.sleep()
    finally:
        print("shutting down serial publisher")
        ser.close()
        
if __name__ == '__main__':
    try:
        serial_publisher()
    except rospy.ROSInterruptException:
        pass
