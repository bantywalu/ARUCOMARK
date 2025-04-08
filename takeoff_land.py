# Monkey patch for Python 3.12:
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import time
import sys
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ----- CONFIGURATION CONSTANTS -----
# List of possible connection strings; adjust according to your system
CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/ttyUSB0', '/dev/serial0']
SERIAL_BAUD = 57600
TAKEOFF_ALTITUDE = 0.3    # 30 centimeters (0.3 meters)
TIMEOUT_SECONDS = 15      # Timeout for reaching the target altitude
HOVER_TIME = 5            # Hover time at target altitude (seconds)
RETRY_DELAY = 5           # Delay (seconds) between connection retries

def try_connect(connection_string, baud=SERIAL_BAUD):
    """
    Try to connect to the Pixhawk using the given connection string.
    Returns a vehicle object if successful, or None otherwise.
    """
    try:
        print(f"Attempting connection on port: {connection_string}")
        vehicle = connect(connection_string, baud=baud, wait_ready=True, heartbeat_timeout=60)
        print(f"Connected to Pixhawk on port: {connection_string}")
        return vehicle
    except Exception as e:
        print(f"Error connecting on {connection_string}: {e}")
        return None

def connect_pixhawk():
    """
    Loops through a list of connection strings until a connection is established.
    Retries the list if no connection is found.
    """
    while True:
        for port in CONNECTION_STRINGS:
            vehicle = try_connect(port)
            if vehicle is not None:
                return vehicle
        print("Failed to connect on all ports. Retrying in {} seconds...".format(RETRY_DELAY))
        time.sleep(RETRY_DELAY)

def arm_and_takeoff(vehicle, target_altitude, timeout):
    """
    Arms the vehicle and initiates takeoff.
    The function returns after the target altitude is reached or after a timeout.
    """
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    # Set vehicle mode to GUIDED and arm the vehicle
    print("Setting vehicle mode to GUIDED and arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    # Command takeoff
    print(f"Taking off to {target_altitude * 100:.0f} centimeters...")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches the target altitude or timeout
    start_time = time.time()
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        elapsed = time.time() - start_time
        print(f"Current Altitude: {current_alt:.2f} m (elapsed {elapsed:.1f} sec)")
        if current_alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break
        if elapsed > timeout:
            print("Timeout reached before target altitude. Proceeding to landing.")
            break
        time.sleep(0.5)

def main():
    # 1. Connect to the Pixhawk
    print("Connecting to Pixhawk autopilot...")
    vehicle = connect_pixhawk()
    if vehicle is None:
        print("Unable to connect to any Pixhawk port. Exiting.")
        sys.exit(1)

    # 2. Arm the vehicle and take off (or timeout after 15 seconds)
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE, TIMEOUT_SECONDS)

    # 3. Hover at the target altitude for a fixed time
    print(f"Hovering at target altitude for {HOVER_TIME} seconds...")
    time.sleep(HOVER_TIME)

    # 4. Initiate landing regardless of current altitude
    print("Initiating landing...")
    vehicle.mode = VehicleMode("LAND")
    
    # Wait until the vehicle is disarmed
    while vehicle.armed:
        print("Waiting for landing to complete...")
        time.sleep(1)

    print("Landed successfully. Closing vehicle connection.")
    vehicle.close()

if __name__ == '__main__':
    main()
