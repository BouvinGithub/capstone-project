import serial
import numpy as np
import threading
from collections import deque

# Constants
BAUD_RATE = 115200
K = 1.19*(10**-5)
BUFFER_SIZE = 20
TAU = 1
NUM_DATA = 4
RSSI = 1
ELEVATION = 2
AZIMUTH = 3

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
            if len(self.vectors)<BUFFER_SIZE:
                return None
            n = len(self.vectors)
            weights = [np.exp(-(k - 1) / tau) for k in range(1, BUFFER_SIZE)]
            weight_sum = sum(weights)
            weighted_values = [sum(w * vec[i] for w, vec in zip(weights, self.vectors)) / weight_sum for i in range(NUM_DATA)]
            return tuple(weighted_values)


def calculate_distance(rssi):
    """Calculates distance using the Friis equation."""
    return K / (10 ** (rssi / 10))

def process_rssi_data(port, baudrate, anchor_vector): #get from topic not serial
    """Reads data from a specified serial port and processes RSSI values into an AnchorVector object."""
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            while True:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    data_list = data.split(",")
                    if len(data_list) > NUM_DATA:
                        rssi = int(data_list[RSSI])
                        distance = calculate_distance(rssi)
                        elevation = float(data_list[ELEVATION])
                        azimuth = float(data_list[AZIMUTH])
                        
                        anchor_vector.add_vector(rssi, distance, elevation, azimuth)

                        if len(anchor_vector.vectors) >= BUFFER_SIZE:
                            avg_vector = anchor_vector.weighted_average()
                            print(f"Avg RSSI: {avg_vector[0]:.2f}, Distance: {avg_vector[1]:.2f}m, Elevation: {avg_vector[2]:.2f}, Azimuth: {avg_vector[3]:.2f}")
    
    except serial.SerialException as e:
        print(f"Error: {e}")
        break
    except KeyboardInterrupt:
        print("\nExiting program.")
        break

def start_rssi_thread(port, anchor_vector):
    """Starts the RSSI processing in a separate thread."""
    rssi_thread = threading.Thread(target=process_rssi_data, args=(port, BAUD_RATE, anchor_vector), daemon=True)
    rssi_thread.start()

if __name__ == "__main__":
    port = input("Enter the COM port: ")
    anchor_vector = AnchorVector(BUFFER_SIZE)
    start_rssi_thread(port, anchor_vector)
    
    while True:
        try:
            pass  # Main thread continues running while RSSI thread processes data
        except KeyboardInterrupt:
            print("\nExiting program.")
            break
