import rospy
from std_msgs.msg import String
import time

def publish_test_data():
    rospy.init_node('test_aoa_publisher', anonymous=True)
    pub = rospy.Publisher('/processed_aoa_data', String, queue_size=10)
    
    angle = float(input("Enter angle (degrees, 0 is straight ahead): "))
    distance = float(input("Enter distance (meters): "))
    
    rate = rospy.Rate(10)
    

    start_time = time.time()
    while time.time() - start_time < 1.0:
        msg = f"0,{distance},0,{angle}"
        rospy.loginfo(f"Publishing: {msg}")
        pub.publish(msg)
        rate.sleep()
    
    while distance > 0:
        msg = f"0,{distance},0,0"
        rospy.loginfo(f"Publishing: {msg}")
        pub.publish(msg)
        distance -= 0.2 
        rate.sleep()
    
    # Publish a few more times at distance zero
    for _ in range(3):
        msg = f"0,0,0,0"
        rospy.loginfo(f"Publishing: {msg}")
        pub.publish(msg)
        rate.sleep()
    
    rospy.loginfo("Test complete.")

if __name__ == "__main__":
    try:
        publish_test_data()
    except rospy.ROSInterruptException:
        pass
