# Monkey patch for Python 3.12:
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ----- CONFIGURATION CONSTANTS -----
TAKEOFF_ALTITUDE   = 0.3      # 30 centimeters (0.3 meters)
TAKEOFF_TIMEOUT    = 15       # Timeout (seconds) before initiating landing regardless of altitude
SERIAL_BAUD        = 57600    # Baud rate for Pixhawk connection
CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']  # Verify these ports for your setup

# ----- Helper Functions -----
def try_connect(connection_string, baud=SERIAL_BAUD):
    try:
        print(f"Attempting connection on port: {connection_string}")
        vehicle = connect(connection_string, baud=baud, wait_ready=False, heartbeat_timeout=60)
        # A simple property check to confirm a connection (for example, reading 'armed' status)
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

def arm_and_takeoff(vehicle, target_altitude, timeout):
    """
    Arms the vehicle and initiates takeoff to a specified altitude.
    If the target altitude is not reached within 'timeout' seconds,
    the function exits and landing will be initiated.
    """
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)
    
    print("Setting vehicle mode to GUIDED and arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    # Initiate takeoff
    print(f"Taking off to {target_altitude * 100:.0f} centimeters...")
    vehicle.simple_takeoff(target_altitude)
    
    # Monitor altitude for up to the timeout duration.
    start_time = time.time()
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        elapsed = time.time() - start_time
        print(f"Current Altitude: {current_alt:.2f} m (elapsed {elapsed:.1f} sec)")
        if current_alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break
        if elapsed >= timeout:
            print("Timeout reached before reaching target altitude.")
            break
        time.sleep(0.5)

# ----- MAIN CODE -----
def main():
    # 1. Connect to the Pixhawk autopilot.
    print("Connecting to Pixhawk autopilot...")
    vehicle = connect_pixhawk()
    if vehicle is None:
        return

    # 2. Arm the vehicle and perform a takeoff to 30 centimeters (or timeout after 15 seconds).
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE, TAKEOFF_TIMEOUT)

    # 3. Initiate landing immediately after the takeoff (or timeout) phase.
    print("Initiating landing...")
    vehicle.mode = VehicleMode("LAND")
    
    # Wait for the landing process to complete (vehicle disarms).
    while vehicle.armed:
        print("Waiting for landing to complete...")
        time.sleep(1)
    
    print("Landed successfully. Closing vehicle connection.")
    vehicle.close()

if __name__ == '__main__':
    main()
