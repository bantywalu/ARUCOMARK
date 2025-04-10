# Monkey patch for Python 3.12 compatibility:
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import time
from dronekit import connect

CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']
SERIAL_BAUD = 57600

def try_connect():
    for connection_string in CONNECTION_STRINGS:
        try:
            print(f"Trying port {connection_string}...")
            vehicle = connect(connection_string, baud=SERIAL_BAUD, wait_ready=True, heartbeat_timeout=60)
            print(f"Connected successfully on {connection_string}")
            return vehicle
        except Exception as e:
            print(f"Failed on {connection_string}: {e}")
    return None

vehicle = try_connect()

if vehicle is None:
    print("Could not connect on any port.")
else:
    # Brief pause to ensure valid current location
    time.sleep(2)

    # Force-set home location manually
    current_location = vehicle.location.global_frame
    vehicle.home_location = current_location

    print(f"Home location manually set to: Lat: {current_location.lat}, Lon: {current_location.lon}, Alt: {current_location.alt}")

    vehicle.close()
