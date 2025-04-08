# Monkey patch for Python 3.12:
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import time
from dronekit import connect, VehicleMode
import sys

# ----- CONFIGURATION CONSTANTS -----
CONNECTION_STRING = '/dev/ttyAMA0'  # Update this port as needed for your setup
SERIAL_BAUD = 57600
TAKEOFF_ALTITUDE = 0.3    # 30 centimeters (0.3 meters)
TIMEOUT_SECONDS = 15      # Timeout for takeoff monitoring (15 seconds)
HOVER_TIME = 5            # Time to hover before landing (optional)

def connect_pixhawk():
    print("Connecting to Pixhawk...")
    try:
        vehicle = connect(CONNECTION_STRING, baud=SERIAL_BAUD, wait_ready=True)
        print("Connected to Pixhawk.")
        return vehicle
    except Exception as e:
        print("Failed to connect:", e)
        sys.exit(1)

def arm_and_takeoff(vehicle, target_altitude, timeout):
    """
    Arms the vehicle and initiates takeoff.
    If the vehicle does not reach the target altitude within 'timeout' seconds,
    the function returns anyway.
    """
    # Wait until the vehicle is armable.
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)
    
    # Set mode to GUIDED and arm.
    print("Setting vehicle mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    print("Arming vehicle...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    # Start takeoff.
    print(f"Initiating takeoff to {target_altitude * 100:.0f} centimeters...")
    vehicle.simple_takeoff(target_altitude)
    
    # Monitor altitude until target altitude is reached or timeout expires.
    start_time = time.time()
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        elapsed = time.time() - start_time
        print(f"Current Altitude: {current_alt:.2f} m (elapsed {elapsed:.1f} sec)")
        if current_alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break
        if elapsed > timeout:
            print("Timeout reached before reaching target altitude.")
            break
        time.sleep(0.5)

def main():
    # 1. Connect to Pixhawk
    vehicle = connect_pixhawk()

    # 2. Arm and takeoff with a timeout.
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE, TIMEOUT_SECONDS)

    # 3. Hover for a brief moment (optional).
    print(f"Hovering for {HOVER_TIME} seconds...")
    time.sleep(HOVER_TIME)

    # 4. Initiate landing.
    print("Initiating landing...")
    vehicle.mode = VehicleMode("LAND")
    
    # Wait until the vehicle is disarmed.
    while vehicle.armed:
        print("Waiting for landing to complete...")
        time.sleep(1)
    
    print("Landed successfully. Closing vehicle connection.")
    vehicle.close()

if __name__ == '__main__':
    main()
