# Monkey patch for Python 3.12:
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import time
import cv2
import cv2.aruco as aruco
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ----- CONFIGURATION CONSTANTS -----
TAKEOFF_ALTITUDE   = 1.0      # Altitude (in meters) for initial takeoff
ASCEND_VELOCITY    = -0.3     # In local NED: negative Z means upward (m/s)
COMMAND_DURATION   = 1        # Duration (seconds) for each velocity command
NO_MARKER_TIMEOUT  = 20       # Max seconds to send upward commands if no marker is seen
SERIAL_BAUD        = 57600    # Baud rate for Pixhawk connection
CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']  # Verify these ports for your setup

# ----- Helper Functions -----
def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Sends a command to move the drone with the specified velocities (m/s) in the local NED frame.
    In NED, a negative velocity_z means upward movement.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,                                # time_boot_ms, target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,      # frame
        0b0000111111000111,                       # type_mask (only velocity enabled)
        0, 0, 0,                                  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,         # velocities in m/s
        0, 0, 0,                                  # accelerations (not supported)
        0, 0                                     # yaw, yaw_rate (not used)
    )
    print(f"Sending NED velocity: vx={velocity_x}, vy={velocity_y}, vz={velocity_z} for {duration} sec")
    end_time = time.time() + duration
    while time.time() < end_time:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def try_connect(connection_string, baud=SERIAL_BAUD):
    try:
        print(f"Attempting connection on port: {connection_string}")
        # Using wait_ready=False to allow us to inspect telemetry manually
        vehicle = connect(connection_string, baud=baud, wait_ready=False, heartbeat_timeout=60)
        # Optionally, try to read a parameter (such as 'armed') to confirm connectivity:
        print("Successfully connected; vehicle armed state:", vehicle.armed)
        return vehicle
    except Exception as e:
        print(f"Error connecting on {connection_string}: {e}")
        return None

def connect_pixhawk():
    vehicle = None
    for cs in CONNECTION_STRINGS:
        vehicle = try_connect(cs)
        if vehicle is not None:
            print(f"Connected to Pixhawk using port: {cs}")
            break
    if vehicle is None:
        print("Failed to connect to Pixhawk on any provided port. Please verify wiring and port names.")
    return vehicle

def arm_and_takeoff(vehicle, target_altitude):
    """
    Arms the vehicle and takes off to a specified altitude.
    """
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    print("Arming vehicle...")
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Vehicle armed. Initiating takeoff!")
    vehicle.simple_takeoff(target_altitude)
    # Wait until the vehicle reaches a safe takeoff altitude.
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Current Altitude: {current_alt:.2f} m")
        if current_alt >= target_altitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

# ----- MAIN CODE -----
def main():
    # 1. Connect to the Pixhawk autopilot.
    print("Connecting to Pixhawk autopilot...")
    vehicle = connect_pixhawk()
    if vehicle is None:
        return

    # 2. Arm the vehicle and perform a proper takeoff.
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)

    # 3. Open the camera using a GStreamer pipeline (suitable for Raspberry Pi).
    cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! "
        "videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )
    if not cap.isOpened():
        print("Error: Could not open camera.")
        vehicle.close()
        return

    # 4. Initialize ArUco marker detection.
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # 5. Begin upward velocity commands while scanning for an ArUco marker.
    print("Beginning continuous ascent. The drone will ascend until an ArUco marker is detected or a timeout occurs.")
    detection_start_time = time.time()

    try:
        while True:
            # Capture a frame from the camera.
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame from camera.")
                continue

            # Determine the center of the frame.
            frame_height, frame_width = frame.shape[:2]
            frame_center = (frame_width // 2, frame_height // 2)
            cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)  # Blue dot at center

            # Convert frame to grayscale and detect ArUco markers.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None:
                print("Detected marker IDs:", ids.flatten())
                cv2.putText(frame, "Marker Detected!", (30, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                print("Marker detected. Stopping ascent and initiating landing sequence.")
                cv2.imshow("ArUco Detection", frame)
                cv2.waitKey(500)
                break

            # Display current status on the frame.
            cv2.putText(frame, "Ascending...", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow("ArUco Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Check if timeout has been reached.
            if (time.time() - detection_start_time) > NO_MARKER_TIMEOUT:
                print("No marker detected within timeout. Stopping ascent.")
                break

            # Send upward velocity command.
            send_ned_velocity(vehicle, 0, 0, ASCEND_VELOCITY, COMMAND_DURATION)
    except KeyboardInterrupt:
        print("User interrupted the control loop.")

    # 6. Command landing.
    print("Commanding landing...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(10)

    # 7. Cleanup resources.
    cap.release()
    cv2.destroyAllWindows()
    vehicle.close()
    print("Shutdown complete.")

if __name__ == '__main__':
    main()
