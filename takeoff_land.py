from dronekit import connect, VehicleMode
import time

# ----- CONFIGURATION CONSTANTS -----
CONNECTION_STRING = '/dev/ttyAMA0'  # Adjust as needed
SERIAL_BAUD = 57600
TAKEOFF_ALTITUDE = 0.3  # 30 centimeters

def connect_pixhawk():
    print("Connecting to Pixhawk...")
    vehicle = connect(CONNECTION_STRING, baud=SERIAL_BAUD, wait_ready=True)
    print("Connected to Pixhawk.")
    return vehicle

def arm_and_takeoff(vehicle, target_altitude):
    # Wait until the vehicle is armable.
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)

    # Set vehicle mode to GUIDED and arm the vehicle.
    print("Setting vehicle mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    print("Arming vehicle...")
    vehicle.armed = True

    # Confirm vehicle is armed.
    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    # Command a takeoff to the target altitude.
    print(f"Taking off to {target_altitude * 100:.0f} centimeters...")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height.
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Current Altitude: {current_alt:.2f} m")
        # When altitude is within 95% of target_altitude, break.
        if current_alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break
        time.sleep(0.5)

def main():
    # 1. Connect to the Pixhawk.
    vehicle = connect_pixhawk()

    # 2. Arm the vehicle and take off to the target altitude.
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)

    # 3. Hover for a moment at the target altitude.
    print("Hovering at target altitude...")
    time.sleep(5)

    # 4. Command the vehicle to land.
    print("Initiating landing...")
    vehicle.mode = VehicleMode("LAND")

    # Wait until the vehicle has landed and disarmed.
    while vehicle.armed:
        print("Waiting for landing to complete...")
        time.sleep(1)
    
    print("Landed successfully. Closing vehicle connection.")
    vehicle.close()

if __name__ == '__main__':
    main()
