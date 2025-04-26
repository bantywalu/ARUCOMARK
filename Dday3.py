import cv2
import cv2.aruco as aruco
import numpy as np
import time, threading
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ── Configuration ────────────────────────────────────────────────────────────
TAKEOFF_ALTITUDE   = 4
ASCEND_VELOCITY    = -0.0
NO_MARKER_TIMEOUT  = 20
SERIAL_BAUD        = 57600
CONNECTION_STRINGS = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/serial0']
TARGET_MARKER_ID   = 0          

def main():
    vehicle = connect_pixhawk()
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)

    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! "
        "videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )
    if not cap.isOpened():
        print("Could not open camera.")
        vehicle.close()
        return

    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    recorder = cv2.VideoWriter('flight_recording1.mp4',
                               cv2.VideoWriter_fourcc(*'mp4v'),
                               fps, (width, height))

    aruco_dict      = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector_params = aruco.DetectorParameters_create()

    stop_event, ascend_thread = start_continuous_ascent(vehicle, ASCEND_VELOCITY)
    detection_start_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            recorder.write(frame)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict,
                                                  parameters=detector_params)

            
            if ids is not None:
                ids_flat = ids.flatten()
                keep_idx = [i for i, m in enumerate(ids_flat)
                            if m == TARGET_MARKER_ID]
                if keep_idx:
                    ids     = ids[keep_idx]
                    corners = [corners[i] for i in keep_idx]
                else:
                    ids = None               

            if ids is not None:              
                stop_event.set()
                ascend_thread.join(timeout=1.0)

                lat = vehicle.location.global_frame.lat
                lon = vehicle.location.global_frame.lon
                print(f"{{ GPS: ({lat:.6f}, {lon:.6f}), Marker ID: 0 }}")

                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(frame, "Marker 0 Detected!", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

                print("Target detected. Landing now.")
                break
            else:
                cv2.putText(frame, "Searching for marker 0…", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            if cv2.waitKey(1) & 0xFF == ord('q'):  break
            if time.time() - detection_start_time > NO_MARKER_TIMEOUT: break

    finally:
        stop_event.set(); ascend_thread.join(timeout=2.0)
        vehicle.mode = VehicleMode("LAND"); time.sleep(1)
        cap.release(); recorder.release(); vehicle.close()

if __name__ == "__main__":
    main()
