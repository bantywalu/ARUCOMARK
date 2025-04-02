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



# Start a 30-second timer to auto-disarm
def auto_disarm_after_timeout(timeout=30):
    print(f"[INFO] Auto-disarm will trigger in {timeout} seconds...")
    time.sleep(timeout)
    if vehicle.armed:
        print("[INFO] Auto-disarming...")
        vehicle.armed = False

# Call this right after arming
vehicle.armed = True

# Start the timer in the background
import threading
threading.Thread(target=auto_disarm_after_timeout, daemon=True).start()
