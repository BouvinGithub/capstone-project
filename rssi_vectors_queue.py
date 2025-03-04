import rospy
import numpy as np
from collections import deque
from std_msgs.msg import String

# Constants
K = 1.19*(10**-5)
BUFFER_SIZE = 20
TAU = 1
NUM_DATA = 4
RSSI = 1
ELEVATION = 2
AZIMUTH = 3

target_topic = "/processed_aoa_data"
lock = threading.Lock()

class AnchorVector:
    def __init__(self, buffer_size):
        self.buffer_size = buffer_size
        self.vectors = deque(maxlen=buffer_size) 
    
    def add_vector(self, rssi, distance, elevation, azimuth):
        """Adds a new vector to the circular buffer."""
        with lock:
            self.vectors.append([rssi, distance, elevation, azimuth])

    def weighted_average(self):
        """Computes the weighted average of RSSI, distance, elevation, and azimuth using exponential decay weights."""
        with lock:
            if len(self.vectors) < BUFFER_SIZE:
                return None
            n = len(self.vectors)
            weights = [np.exp(-(k - 1) / TAU) for k in range(1, BUFFER_SIZE)]
            weight_sum = sum(weights)
            weighted_values = [sum(w * vec[i] for w, vec in zip(weights, self.vectors)) / weight_sum for i in range(NUM_DATA)]
            return tuple(weighted_values)

def calculate_distance(rssi):
    """Calculates distance using the Friis equation."""
    return K / (10 ** (rssi / 10))

def process_rssi_data(msg):
    """Processes RSSI data from the ROS topic."""
    try:
        data = msg.data
        try:
            data_list = data.split(",")
            if len(data_list) > NUM_DATA:
                rssi = int(data_list[RSSI])
                distance = calculate_distance(rssi)
                elevation = float(data_list[ELEVATION])
                azimuth = float(data_list[AZIMUTH])
                
                anchor_vector.add_vector(rssi, distance, elevation, azimuth)

                if len(anchor_vector.vectors) >= BUFFER_SIZE:
                    avg_vector = anchor_vector.weighted_average()
                    output_msg = f"{avg_vector[0]:.2f},{avg_vector[1]:.2f},{avg_vector[2]:.2f},{avg_vector[3]:.2f}"
                    pub.publish(output_msg)
    
        except (ValueError, UnicodeDecodeError):
            rospy.logwarn("Received malformed data, ignoring it.")
    except Exception as e:
        rospy.logerr(f"Error processing RSSI data: {e}")

if __name__ == "__main__":
    rospy.init_node("rssi_processor")
    anchor_vector = AnchorVector(BUFFER_SIZE)
    pub = rospy.Publisher(target_topic, String, queue_size=10)
    rospy.Subscriber("/aoa_data", String, process_rssi_data)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()
