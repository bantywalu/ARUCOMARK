#!/usr/bin/env python3

# --- Python 3.12 Compatibility Monkey Patch ---
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

# --- Core Imports ---
import sys
import time
import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# --------- Connection Helper ---------
def safe_connect():
    possible_conns = [
        '127.0.0.1:14550',    # SITL
        '/dev/ttyAMA0',       # Pi serial
        '/dev/ttyACM0',       # USB cable
        '/dev/serial0',       # General
    ]
    for conn in possible_conns:
        try:
            print(f"Trying to connect to {conn}...")
            vehicle = connect(conn, baud=57600, wait_ready=True)
            print(f"Connected to {conn}")
            return vehicle
        except Exception as e:
            print(f"Failed to connect to {conn}: {e}")
    print("Unable to connect to any vehicle.")
    sys.exit(1)

# Connect to vehicle
vehicle = safe_connect()

# --- Functions ---
def get_field_params():
    try:
        width = float(input("Enter field width (m): "))
        height = float(input("Enter field height (m): "))
    except Exception:
        print("Invalid input for field dimensions. Aborting mission.")
        return None, None
    return width, height

def get_start_point(width, height):
    fig, ax = plt.subplots()
    ax.set_title("Select Drone Start Position in Field")
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_xlabel("Width (m)")
    ax.set_ylabel("Height (m)")
    rect = plt.Rectangle((0, 0), width, height, fill=False, ls='--', color='gray')
    ax.add_patch(rect)
    plt.ion()
    plt.show()
    coords = []
    try:
        coords = plt.ginput(1, timeout=-1)
    except Exception:
        coords = []
    plt.close(fig)
    if not coords:
        return None, None
    x0, y0 = coords[0]
    x0 = max(0, min(x0, width))
    y0 = max(0, min(y0, height))
    print(f"Selected start point: ({x0:.2f}, {y0:.2f}) meters")
    return x0, y0

def send_ned_velocity(vehicle, vx, vy, vz, duration=1):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # only velocities
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    for _ in range(int(duration * 5)):
        vehicle.send_mavlink(msg)
        time.sleep(0.2)

def fly_to_center(vehicle, x0, y0, width, height):
    x_c, y_c = width / 2.0, height / 2.0
    dN, dE = y_c - y0, x_c - x0
    diag = math.hypot(width, height)
    vehicle.mode = VehicleMode("GUIDED")

    if not vehicle.armed:
        vehicle.armed = True
        while not vehicle.armed:
            print("Arming...")
            time.sleep(1)

    if vehicle.location.global_relative_frame.alt < 1.0:
        print("Taking off to 2m...")
        vehicle.simple_takeoff(2)
        while vehicle.location.global_relative_frame.alt < 1.5:
            time.sleep(0.1)

    print(f"Flying toward center (dN={dN:.1f}, dE={dE:.1f})...")
    start_time = time.time()
    while True:
        loc = vehicle.location.global_relative_frame
        rem_n = dN
        rem_e = dE
        distance = math.hypot(rem_n, rem_e)
        if distance < 0.5:
            print("Reached center.")
            return True
        if time.time() - start_time > 60:
            print("Timeout. Landing.")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(5)
            return False
        if distance > diag:
            print("Out of bounds. Landing.")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(5)
            return False
        speed = 1.0
        vx = (rem_n / distance) * speed if distance != 0 else 0
        vy = (rem_e / distance) * speed if distance != 0 else 0
        send_ned_velocity(vehicle, vx, vy, 0, duration=0.5)

def nudge_to_marker_center(vehicle, marker_offset_px, cap):
    if marker_offset_px is None:
        return
    off_x, off_y = marker_offset_px
    print(f"Marker offset: ({off_x:.1f}, {off_y:.1f}) px")
    Kp = 0.01
    for attempt in range(3):
        if abs(off_x) <= 25 and abs(off_y) <= 25:
            print("Marker centered.")
            break
        vx = -Kp * off_y
        vy =  Kp * off_x
        vx = max(min(vx, 0.5), -0.5)
        vy = max(min(vy, 0.5), -0.5)
        send_ned_velocity(vehicle, vx, vy, 0, duration=0.5)
        send_ned_velocity(vehicle, 0, 0, 0, duration=0.2)
        ret, frame = cap.read()
        if not ret:
            break
        corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        if ids is not None and len(ids) > 0:
            c0 = corners[0][0]
            mkr_cx = np.mean(c0[:, 0])
            mkr_cy = np.mean(c0[:, 1])
            frame_cx = frame.shape[1] / 2.0
            frame_cy = frame.shape[0] / 2.0
            off_x = mkr_cx - frame_cx
            off_y = mkr_cy - frame_cy
            print(f"New offset: ({off_x:.1f}, {off_y:.1f}) px")
        else:
            print("Marker lost.")
            break
    print("Nudging complete.")

# --- Main ---
width, height = get_field_params()
if width is None or height is None:
    vehicle.close()
    sys.exit("Mission aborted.")

x0, y0 = get_start_point(width, height)
if x0 is None or y0 is None:
    if vehicle.armed:
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        vehicle.armed = False
    vehicle.close()
    sys.exit("User aborted.")

if not fly_to_center(vehicle, x0, y0, width, height):
    vehicle.close()
    sys.exit(1)

print("At center. Ascending...")
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()
    sys.exit("Camera failed.")

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('flight_recording1.mp4', fourcc, 20.0, (frame_width, frame_height))
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

vertical_start = time.time()
last_vel_time = 0
marker_found = False
marker_offset = None

while True:
    ret, frame = cap.read()
    if not ret:
        break
    out.write(frame)
    corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    if ids is not None and len(ids) > 0:
        marker_found = True
        c0 = corners[0][0]
        mkr_cx = np.mean(c0[:, 0])
        mkr_cy = np.mean(c0[:, 1])
        frame_cx = frame.shape[1] / 2.0
        frame_cy = frame.shape[0] / 2.0
        offset_x = mkr_cx - frame_cx
        offset_y = mkr_cy - frame_cy
        marker_offset = (offset_x, offset_y)
        print("Marker detected.")
        break
    if time.time() - vertical_start > 20:
        print("Timeout: no marker.")
        break
    if time.time() - last_vel_time >= 1.0:
        send_ned_velocity(vehicle, 0, 0, -0.5, duration=0.5)
        last_vel_time = time.time()

send_ned_velocity(vehicle, 0, 0, 0, duration=1)

if marker_found and marker_offset is not None:
    vehicle.mode = VehicleMode("GUIDED")
    nudge_to_marker_center(vehicle, marker_offset, cap)
    vehicle.mode = VehicleMode("LAND")
else:
    vehicle.mode = VehicleMode("LAND")

while True:
    ret, frame = cap.read()
    if ret:
        out.write(frame)
    alt = vehicle.location.global_relative_frame.alt
    if alt <= 0.1 or not vehicle.armed:
        break
    time.sleep(0.1)

cap.release()
out.release()
vehicle.close()
print("Mission complete.")