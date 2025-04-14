# --- Python 3.12 Compatibility Patch ---
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

# --- Libraries ---
import time
import threading
import cv2
import cv2.aruco as aruco
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# --- CONFIGURATION CONSTANTS ---
TAKEOFF_ALTITUDE   = 1.0       # meters
ASCEND_VELOCITY    = -0.3      # m/s upward in NED frame (negative Z is up)
NO_MARKER_TIMEOUT  = 90        # seconds until timeout landing
COMMAND_INTERVAL   = 1         # seconds between velocity commands
SERIAL_BAUD        = 57600
CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']

# --- FUNCTIONS ---
def try_connect(conn_str, baud=SERIAL_BAUD):
    try:
        return connect(conn_str, baud=baud, wait_ready=False, heartbeat_timeout=60)
    except:
        return None

def send_velocity(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def detect_aruco_marker(cap):
    ret, frame = cap.read()
    if not ret:
        return False
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    return ids is not None and len(ids) > 0

# --- MAIN ---
def main():
    # Connect to Pixhawk
    vehicle = None
    for conn_str in CONNECTION_STRINGS:
        vehicle = try_connect(conn_str)
        if vehicle:
            print(f"Connected to Pixhawk on {conn_str}")
            break
    if not vehicle:
        print("Failed to connect to Pixhawk.")
        return

    # Camera init
    cap = cv2.VideoCapture(
        'libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1 ! '
        'videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink',
        cv2.CAP_GSTREAMER
    )
    if not cap.isOpened():
        print("Could not open camera.")
        return

    # Arm and take off
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Arming...")
        time.sleep(1)

    print("Ascending...")
    start_time = time.time()
    marker_found = False

    while True:
        elapsed = time.time() - start_time
        if elapsed > NO_MARKER_TIMEOUT:
            print("Timeout reached. Initiating landing.")
            break

        if detect_aruco_marker(cap):
            print("ArUco Marker Detected! Initiating landing.")
            marker_found = True
            break

        send_velocity(vehicle, 0, 0, ASCEND_VELOCITY)
        time.sleep(COMMAND_INTERVAL)

    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Landing...")
        time.sleep(1)

    print("Landed and disarmed.")
    cap.release()
    vehicle.close()

# --- EXECUTION ---
if __name__ == '__main__':
    main()
