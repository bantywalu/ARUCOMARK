# Monkey patch for Python 3.12 where collections.MutableMapping was moved to collections.abc.MutableMapping
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect
import time

def main():
    print("Connecting to Pixhawk autopilot...")
    try:
        vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)  # adjust if needed
    except Exception as e:
        print("Error connecting to Pixhawk:", e)
        return

    print("Connected. Waiting for GPS lock...")

    # Wait for a valid GPS fix
    while True:
        # vehicle.location.global_frame should provide lat and lon once a fix is available.
        location = vehicle.location.global_frame
        # Some Pixhawk setups also provide a fix type via vehicle.gps_0.fix_type (2 = 2D fix, 3 = 3D fix)
        fix_type = getattr(vehicle.gps_0, 'fix_type', 0)
        
        # Check if GPS fix is valid (using either method)
        if location.lat is not None and location.lon is not None and fix_type > 1:
            print("GPS Lock achieved!")
            print(f"Latitude: {location.lat:.6f}, Longitude: {location.lon:.6f}")
            break
        else:
            print("Waiting for GPS fix...", end="\r")
        time.sleep(1)

    vehicle.close()

if __name__ == '__main__':
    main()
