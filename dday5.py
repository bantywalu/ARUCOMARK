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
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# --- Configuration ---
TAKEOFF_ALTITUDE     = 3.0     # meters
NO_MARKER_TIMEOUT    = 20      # seconds
CENTER_THRESH_PIX    = 20      # pixels tolerance for centered
KP_PIX_TO_VEL        = 0.001   # m/s per pixel error
MAX_VEL              = 0.5     # max horizontal velocity m/s

SERIAL_BAUD          = 57600
CONNECTION_STRINGS   = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']
TARGET_MARKER_ID     = 0       # <<< Only detect marker ID 0

# --- Helper: connect to Pixhawk ---
def try_connect(conn_str, baud=SERIAL_BAUD):
    try:
        print(f"Trying {conn_str}…")
        v = connect(conn_str, baud=baud, wait_ready=False, heartbeat_timeout=60)
        print(f"Connected on {conn_str}")
        return v
    except Exception as e:
        print(f"Failed on {conn_str}: {e}")
        return None

def connect_pixhawk():
    for s in CONNECTION_STRINGS:
        v = try_connect(s)
        if v:
            return v
    raise RuntimeError("Unable to connect to Pixhawk on any port.")

# --- Helper: arm & takeoff ---
def arm_and_takeoff(vehicle, target_alt):
    print("Waiting for armable…")
    while not vehicle.is_armable:
        time.sleep(1)
    print("Arming…")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    print(f"Taking off to {target_alt} m…")
    vehicle.simple_takeoff(target_alt)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Alt: {alt:.2f} m")
        if alt >= target_alt * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

# --- Helper: send velocity in body frame ---
def send_body_velocity(vehicle, vx, vy, vz=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# --- Main ---
def main():
    vehicle = connect_pixhawk()
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)
    print(f"Holding at ~{TAKEOFF_ALTITUDE} m, searching and centering on Marker 0…")

    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,format=NV12,"
        "width=1920,height=1080,framerate=30/1 ! "
        "videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )
    if not cap.isOpened():
        print("Camera open failed.")
        vehicle.close()
        return

    aruco_dict      = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector_params = aruco.DetectorParameters_create()
    start_time      = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            h, w = frame.shape[:2]
            center_px = (w // 2, h // 2)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

            # --- Filter for marker 0 only ---
            if ids is not None:
                ids_flat = ids.flatten()
                keep_idx = [i for i, m in enumerate(ids_flat) if m == TARGET_MARKER_ID]
                if keep_idx:
                    ids = ids[keep_idx]
                    corners = [corners[i] for i in keep_idx]
                else:
                    ids = None

            if ids is not None and len(ids) > 0:
                pts = corners[0][0]
                marker_px = (int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1])))

                dx = marker_px[0] - center_px[0]
                dy = marker_px[1] - center_px[1]

                vx = -KP_PIX_TO_VEL * dy
                vy =  KP_PIX_TO_VEL * dx

                vx = max(min(vx, MAX_VEL), -MAX_VEL)
                vy = max(min(vy, MAX_VEL), -MAX_VEL)

                if abs(dx) < CENTER_THRESH_PIX and abs(dy) < CENTER_THRESH_PIX:
                    lat = vehicle.location.global_frame.lat
                    lon = vehicle.location.global_frame.lon
                    print(f"Centered — GPS: ({lat:.6f}, {lon:.6f}), Marker ID: {ids[0][0]}")
                    print("Landing now.")
                    vehicle.mode = VehicleMode("LAND")
                    break
                else:
                    send_body_velocity(vehicle, vx, vy, 0)
                    print(f"Centering: dx={dx}, dy={dy}, vx={vx:.2f}, vy={vy:.2f}")

            if (time.time() - start_time) > NO_MARKER_TIMEOUT:
                print("Timeout—no marker 0 found.")
                break

    except KeyboardInterrupt:
        print("Interrupted by user—landing.")
        vehicle.mode = VehicleMode("LAND")
    finally:
        cap.release()
        vehicle.close()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
