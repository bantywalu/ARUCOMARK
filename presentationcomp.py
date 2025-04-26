# --- Python 3.12 Compatibility Patch ---
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

# --- Core Imports ---
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# --- Configuration ---
SERIAL_BAUD = 57600
CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
DETECTOR_PARAMS = aruco.DetectorParameters_create()
TARGET_MARKER_ID = 0  # <<< Only land if Marker ID = 0

# --- Helper: Try Multiple Pixhawk Ports ---
def connect_to_pixhawk():
    for conn_str in CONNECTION_STRINGS:
        try:
            print(f"Trying {conn_str}...")
            vehicle = connect(conn_str, baud=SERIAL_BAUD, wait_ready=True, timeout=60)
            print(f"Connected to Pixhawk on {conn_str}")
            return vehicle
        except Exception as e:
            print(f"Failed on {conn_str}: {e}")
    raise RuntimeError("Could not connect to any Pixhawk port.")

# --- Main ---
def main():
    vehicle = connect_to_pixhawk()

    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! "
        "videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )
    if not cap.isOpened():
        print("Camera failed to open.")
        vehicle.close()
        return

    print(f"Watching for ArUco markers... (target to land: ID {TARGET_MARKER_ID})")
    detection_start_time = time.time()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame.")
                time.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=DETECTOR_PARAMS)

            if ids is not None and len(ids) > 0:
                lat = vehicle.location.global_frame.lat
                lon = vehicle.location.global_frame.lon

                # Always print all detected markers
                print(f"Detected {len(ids)} markers at GPS ({lat:.6f}, {lon:.6f}): ", end="")
                print([marker_id for marker_id in ids.flatten()])

                # Now check if TARGET_MARKER_ID is among them
                for marker_id in ids.flatten():
                    if marker_id == TARGET_MARKER_ID:
                        print(f"Target Marker {TARGET_MARKER_ID} detected. Initiating landing...")
                        vehicle.mode = VehicleMode("LAND")
                        while vehicle.mode.name != "LAND":
                            print(f"Waiting for LAND mode... Current mode: {vehicle.mode.name}")
                            time.sleep(1)
                        return  # Exit after landing command sent

            if time.time() - detection_start_time > 120:
                print("Timeout: No marker detected in 120 seconds.")
                break

    except KeyboardInterrupt:
        print("KeyboardInterrupt: Exiting.")

    finally:
        cap.release()
        vehicle.close()
        print("Camera and vehicle closed. Done.")

if __name__ == "__main__":
    main()