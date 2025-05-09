import cv2
import cv2.aruco as aruco

def get_frame(cap):
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to capture frame from camera.")
        return None
    return frame

def main():
    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! "
        "videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    print("Starting ArUco marker detection. Press 'q' to exit.")

    while True:
        frame = get_frame(cap)
        if frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_corner in enumerate(corners):
                top_left = marker_corner[0][0]
                cv2.putText(
                    frame,
                    f"ID: {ids[i][0]}",
                    (int(top_left[0]), int(top_left[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
                print(f"Detected marker ID: {ids[i][0]}")

        cv2.imshow("ArUco Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
