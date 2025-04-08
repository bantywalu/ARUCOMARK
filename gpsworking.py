# Monkey patch for Python 3.12:
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect
import time

def try_connect(connection_string, baud=57600):
    try:
        print(f"Trying connection string: {connection_string}")
        vehicle = connect(connection_string, baud=baud, wait_ready=False, heartbeat_timeout=60)
        print("Connected successfully using:", connection_string)
        return vehicle
    except Exception as e:
        print(f"Error connecting using {connection_string}: {e}")
        return None

def main():
    # List of potential connection strings—adjust or add others as needed.
    connection_strings = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']
    
    vehicle = None
    for cs in connection_strings:
        vehicle = try_connect(cs)
        if vehicle is not None:
            break

    if vehicle is None:
        print("Failed to connect to the Pixhawk autopilot on any provided port.")
        return

    # Allow a brief delay for the vehicle to send some telemetry.
    time.sleep(2)
    
    # Check current GPS data (if any)
    location = vehicle.location.global_frame
    if location.lat is not None and location.lon is not None:
        print(f"Current GPS Location: Latitude: {location.lat:.6f}, Longitude: {location.lon:.6f}")
    else:
        print("No valid GPS fix available yet.")
    
    vehicle.close()

if __name__ == '__main__':
    main()
