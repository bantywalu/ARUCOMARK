from dronekit import LocationGlobal, Command

def set_home_to_current(vehicle):
    current_location = vehicle.location.global_frame
    vehicle.home_location = current_location
    print(f"Home location set manually to: {current_location.lat}, {current_location.lon}, {current_location.alt}")

# In your main() after connecting:
vehicle = connect_pixhawk()
if vehicle is None:
    return

# Manually set home location immediately after connection
set_home_to_current(vehicle)
