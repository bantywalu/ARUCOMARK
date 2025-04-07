import cv2
import cv2.aruco as aruco
import math
import numpy as np

# ---- Configuration Constants ----
MARKER_REAL_SIZE = 0.05  # Actual marker width in meters (e.g., 5 cm)
FOCAL_LENGTH = 800       # Estimated focal length in pixels (adjust via calibration)

def estimate_distance(pixel_width):
    """
    Estimate the distance from the camera to the marker.
    Formula: distance = (marker_real_width * focal_length) / pixel_width
    """
    if pixel_width == 0:
        return None
    return (MARKER_REAL_SIZE * FOCAL_LENGTH) / pixel_width

def main():
    # Open the default camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    # Use a predefined ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    print("Starting ArUco detection. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture frame.")
            break

        # Get frame dimensions and compute the frame center
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)

        # Optionally, draw the frame center (blue dot)
        cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)

        # Convert frame to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        guidance_text = ""
        if ids is not None:
            # Process each detected marker (here we assume one marker for guidance)
            for i, marker_corners in enumerate(corners):
                # Draw detected marker boundaries and ID
                aruco.drawDetectedMarkers(frame, corners, ids)
                # marker_corners shape is (1, 4, 2), so extract the points
                pts = marker_corners[0]
                # Compute the centroid of the marker
                marker_center = (int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1])))
                cv2.circle(frame, marker_center, 5, (0, 0, 255), -1)

                # Calculate difference between marker center and frame center
                dx = marker_center[0] - frame_center[0]
                dy = marker_center[1] - frame_center[1]
                threshold = 20  # pixels threshold for "centered"
                guidance = []

                # Determine horizontal guidance:
                if dx > threshold:
                    guidance.append("Move Left")
                elif dx < -threshold:
                    guidance.append("Move Right")

                # Determine vertical guidance:
                if dy > threshold:
                    guidance.append("Move Up")
                elif dy < -threshold:
                    guidance.append("Move Down")

                if not guidance:
                    guidance_text = "Centered"
                else:
                    guidance_text = ", ".join(guidance)

                # Display guidance text near the marker
                cv2.putText(frame, guidance_text, (marker_center[0] - 50, marker_center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # For distance estimation, compute the pixel width using the top edge of the marker
                pt1 = pts[0]  # top-left
                pt2 = pts[1]  # top-right
                pixel_width = math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
                distance = estimate_distance(pixel_width)
                if distance is not None:
                    distance_text = f"{distance:.2f} m"
                    cv2.putText(frame, distance_text, (marker_center[0] - 50, marker_center[1] + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    print(f"Marker ID {ids[i][0]} estimated distance: {distance:.2f} meters")
                # For this example, process only the first detected marker
                break

        # Display the annotated frame
        cv2.imshow("ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
