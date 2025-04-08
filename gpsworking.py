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
        # Adjust the connection string and baud rate as needed
        vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
    except Exception as e:
        print("Error connecting to Pixhawk:", e)
        return

    print("Connected. Starting continuous GPS status check...")

    try:
        while True:
            # Retrieve GPS fix type (usually, 2: 2D fix, 3: 3D fix)
            fix_type = getattr(vehicle.gps_0, 'fix_type', 0)
            # Get the current global location
            location = vehicle.location.global_frame

            if fix_type > 1 and location.lat is not None and location.lon is not None:
                print(f"GPS Lock: Latitude: {location.lat:.6f}, Longitude: {location.lon:.6f}")
            else:
                print("No valid GPS fix.")
            
            # Wait 5 seconds before rechecking
            time.sleep(5)
    except KeyboardInterrupt:
        print("\nStopping GPS check.")
    finally:
        vehicle.close()

if __name__ == '__main__':
    main()
