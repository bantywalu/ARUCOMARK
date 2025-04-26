#!/usr/bin/env python3
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
import threading
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# --- Configuration ---
TAKEOFF_ALTITUDE   = 4        # meters
ASCEND_VELOCITY    = -0.0     # m/s upward in NED (negative Z is up)
NO_MARKER_TIMEOUT  = 20       # seconds
SERIAL_BAUD        = 57600
CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']
TARGET_MARKER_ID   = 0        # <<< detect only marker 0

# --- Connection Helpers ---
def try_connect(conn_str, baud=SERIAL_BAUD):
    try:
        print(f"Trying to connect on {conn_str}...")
        vehicle = connect(conn_str, baud=baud, wait_ready=False, heartbeat_timeout=60)
        print("Connected:", conn_str)
        return vehicle
    except Exception as e:
        print(f"Failed on {conn_str}: {e}")
        return None

def connect_pixhawk():
    for conn in CONNECTION_STRINGS:
        vehicle = try_connect(conn)
        if vehicle:
            return vehicle
    raise RuntimeError("Unable to connect to Pixhawk.")

# --- Takeoff ---
def arm_and_takeoff(vehicle, target_altitude):
    print("Waiting for vehicle to be armable...")
    while not vehicle.is_armable:
        time.sleep(1)
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.2f} m")
        if alt >= target_altitude * 0.95:
            break
        time.sleep(1)

# --- Continuous Ascent ---
def start_continuous_ascent(vehicle, velocity_m_s):
    stop_event = threading.Event()
    def ascent_loop():
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            0, 0, velocity_m_s,
            0, 0, 0,
            0, 0
        )
        while not stop_event.is_set():
            vehicle.send_mavlink(msg)
            time.sleep(0.1)
    thread = threading.Thread(target=ascent_loop, daemon=True)
    thread.start()
    return stop_event, thread

# --- Main ---
def main():
    vehicle = connect_pixhawk()
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)

    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! "
        "videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )
    if not cap.isOpened():
        print("Could not open camera.")
        vehicle.close()
        return
    
    # ─── start recording to file ───
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    recorder = cv2.VideoWriter('flight_recording1.mp4', fourcc, fps, (width, height))

    aruco_dict      = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    detector_params = aruco.DetectorParameters_create()

    print("Ascending until marker 0 is detected or timeout.")
    stop_event, ascend_thread = start_continuous_ascent(vehicle, ASCEND_VELOCITY)
    detection_start_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Frame read failed.")
                time.sleep(0.1)
                continue

            recorder.write(frame)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

            # --- Only keep marker 0 ---
            if ids is not None:
                ids_flat = ids.flatten()
                keep_idx = [i for i, m in enumerate(ids_flat) if m == TARGET_MARKER_ID]
                if keep_idx:
                    ids = ids[keep_idx]
                    corners = [corners[i] for i in keep_idx]
                else:
                    ids = None

            if ids is not None and len(ids) > 0:
                stop_event.set()
                ascend_thread.join(timeout=1.0)

                lat = vehicle.location.global_frame.lat
                lon = vehicle.location.global_frame.lon
                for marker_id in ids.flatten():
                    print(f"{{ GPS: ({lat:.6f}, {lon:.6f}), Marker ID: {marker_id} }}")

                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(frame, "Marker 0 Detected!", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

                print("Marker 0 detected. Landing now.")
                break
            else:
                cv2.putText(frame, "Searching for marker 0...", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Quit pressed. Exiting.")
                break

            if time.time() - detection_start_time > NO_MARKER_TIMEOUT:
                print("Timeout. No marker 0 detected.")
                break

    except KeyboardInterrupt:
        print("Keyboard interrupt received.")

    finally:
        stop_event.set()
        if ascend_thread.is_alive():
            ascend_thread.join(timeout=2.0)
        print("Setting LAND mode...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        cap.release()
        recorder.release()
        vehicle.close()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
