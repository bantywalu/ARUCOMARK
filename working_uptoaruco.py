# --- Python 3.12 Monkey Patch ---
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import time
from dronekit import VehicleMode

# ... [Assume vehicle is already connected, armed, and in GUIDED mode] ...

# Start ascending (e.g., by sending velocity or throttle commands in a loop)
search_start = time.time()
marker_found = False

while True:
    # Capture frame from camera and run ArUco detection
    ret, frame = cap.read()
    if not ret:
        continue  # skip if frame not available
    corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    
    if ids is not None and len(ids) > 0:
        # Marker detected
        marker_found = True
        print("ArUco marker detected!")
        break  # exit the loop to stop ascending
    
    # Check timeout
    if time.time() - search_start > 90:
        print("Marker not found within timeout. Initiating landing...")
        break  # exit loop due to timeout
    
    # If no marker yet and timeout not reached, continue ascending
    # (For example, raise altitude or maintain upward velocity)
    # Here you could send a MAVLink velocity command or adjust throttle.
    # E.g., send an upward velocity command in GUIDED mode:
    # send_ned_velocity(0, 0, +0.5)  # ascend at 0.5 m/s (NED: positive z is down, so use negative for up if using setpoint)
    # (Implement send_ned_velocity as needed to publish MAVLink SET_POSITION_TARGET_LOCAL_NED)
    
    time.sleep(0.1)  # small delay to avoid tight looping
