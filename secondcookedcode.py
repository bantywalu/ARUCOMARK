from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the flight controller
print("[INFO] Connecting to vehicle on /dev/ttyAMA0...")
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("[INFO] Arming motors...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"[INFO] Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_alt:.1f}m")
        if current_alt >= target_altitude * 0.95:
            print("[INFO] Target altitude reached")
            break
        time.sleep(1)

def auto_land():
    print("[INFO] Initiating landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(1)
    print("[INFO] Landed and disarmed")

# Usage
arm_and_takeoff(3)  # Try low altitude for safety
time.sleep(5)       # Hover
auto_land()

vehicle.close()
