import cv2
import cv2.aruco as aruco
import math
import numpy as np

# ---- Configuration Constants ----
MARKER_REAL_SIZE = 0.05  # Actual marker width in meters (e.g., 5 cm)
FOCAL_LENGTH = 800       # Estimated focal length in pixels (adjust via calibration)

def estimate_distance(pixel_width):
    if pixel_width == 0:
        return None
    return (MARKER_REAL_SIZE * FOCAL_LENGTH) / pixel_width

def main():
    # Open camera with GStreamer pipeline (compatible with libcamera on Raspberry Pi)
    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,format=NV12,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )

    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    print("Starting ArUco detection. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture frame.")
            break

        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)
        cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        guidance_text = ""
        if ids is not None:
            for i, marker_corners in enumerate(corners):
                aruco.drawDetectedMarkers(frame, corners, ids)
                pts = marker_corners[0]
                marker_center = (int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1])))
                cv2.circle(frame, marker_center, 5, (0, 0, 255), -1)

                dx = marker_center[0] - frame_center[0]
                dy = marker_center[1] - frame_center[1]
                threshold = 20
                guidance = []

                if dx > threshold:
                    guidance.append("Move Left")
                elif dx < -threshold:
                    guidance.append("Move Right")

                if dy > threshold:
                    guidance.append("Move Up")
                elif dy < -threshold:
                    guidance.append("Move Down")

                guidance_text = "Centered" if not guidance else ", ".join(guidance)
                cv2.putText(frame, guidance_text, (marker_center[0] - 50, marker_center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                pt1 = pts[0]
                pt2 = pts[1]
                pixel_width = math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
                distance = estimate_distance(pixel_width)

                if distance is not None:
                    distance_text = f"{distance:.2f} m"
                    cv2.putText(frame, distance_text, (marker_center[0] - 50, marker_center[1] + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    print(f"Marker ID {ids[i][0]} estimated distance: {distance:.2f} meters")
                break  # Only process the first marker for now

        cv2.imshow("ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
