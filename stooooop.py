from dronekit import connect, VehicleMode
import time

# Monkey patch for Python 3.10+
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

# Connect to the drone
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True)

print("[INFO] Forcing LAND...")
vehicle.mode = VehicleMode("LAND")

# Wait until disarmed
while vehicle.armed:
    print(f" Altitude: {vehicle.location.global_relative_frame.alt:.1f}m")
    time.sleep(1)

print("[INFO] Disarmed. Closing connection.")
vehicle.close()
