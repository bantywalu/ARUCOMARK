# import cv2
# import cv2.aruco as aruco

# def main():
#     # Open a connection to the default camera
#     cap = cv2.VideoCapture(0)
#     if not cap.isOpened():
#         print("Error: Could not open the camera.")
#         return

#     # Use the new function name: getPredefinedDictionary
#     # Choose the dictionary that matches your markers
#     aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

#     # If you want advanced parameter tuning:
#     parameters = aruco.DetectorParameters()
    
#     # (Optional) A new recommended approach is to create an ArucoDetector object:
#     # detector = aruco.ArucoDetector(aruco_dict, parameters)
#     # Then call corners, ids, rejected = detector.detectMarkers(frame)

#     print("Starting ArUco marker detection. Press 'q' to exit.")

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Error: Unable to read frame from camera.")
#             break

#         # Convert the frame to grayscale
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Detect markers (older function style, still works)
#         corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

#         # If any markers are found
#         if ids is not None and len(ids) > 0:
#             # Draw bounding boxes and IDs on the frame
#             aruco.drawDetectedMarkers(frame, corners, ids)
#             for i, marker_corner in enumerate(corners):
#                 # Top-left corner (x, y)
#                 top_left = marker_corner[0][0]
#                 cv2.putText(
#                     frame,
#                     f"ID: {ids[i][0]}",
#                     (int(top_left[0]), int(top_left[1]) - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.5,
#                     (0, 255, 0),
#                     2
#                 )

#         cv2.imshow("ArUco Detection", frame)

#         # Press 'q' to quit
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()
import cv2
import cv2.aruco as aruco

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    # Use the new function name for getting the dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()  

    print("Looking for an ArUco marker...")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to read frame from camera.")
            break

        # Convert frame to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for marker in ids:
                print(f"Detected marker ID: {marker[0]}")
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("ArUco Detection", frame)
            cv2.waitKey(1000)
            break  

        cv2.imshow("ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
