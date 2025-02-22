import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_id = 8
marker_size = 200

marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite("marker55.png", marker_image)
print("Marker generated and saved as marker45.png")
