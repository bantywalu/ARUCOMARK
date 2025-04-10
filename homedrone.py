# Monkey patch for Python 3.12 compatibility:
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect
import time

# Connect (update the port if needed)
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True, heartbeat_timeout=60)

# Wait briefly to get a valid current location
time.sleep(2)

# Force-set home location manually
current_location = vehicle.location.global_frame
vehicle.home_location = current_location

print(f"Home location manually set to: Lat: {current_location.lat}, Lon: {current_location.lon}, Alt: {current_location.alt}")

# Close vehicle connection
vehicle.close()
