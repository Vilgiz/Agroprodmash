import cv2
import cv2.aruco


# Load the input image
cap = cv2.VideoCapture(1)
ret, image = cap.read()
class Marker:
    def __init__(self, id, center, corners):
            self.id = id
            self.center = center
            [self.topLeft, self.topRight, self.bottomRight, self.bottomLeft] = corners


    def detectArucoMarkers(self, image):
            value = 0
            count = 0
            while value < 3:
                count +=1
                markers = {}
                arucoDictionary = cv2.aruco.getPredefinedDictionary(
                    cv2.aruco.DICT_4X4_50)
                arucoParameters = cv2.aruco.DetectorParameters()
                (corners, ids, rejected) = cv2.aruco.detectMarkers(
                    image, arucoDictionary, parameters=arucoParameters)
                value = len(corners) 

                if count > 10:
                    print('[ERROR] Could not find all markers')
                    return None

            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = [int(topRight[0]), int(topRight[1])]
                bottomRight = [int(bottomRight[0]), int(bottomRight[1])]
                bottomLeft = [int(bottomLeft[0]), int(bottomLeft[1])]
                topLeft = [int(topLeft[0]), int(topLeft[1])]
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                markers[markerID] = Marker(
                    markerID, [cX, cY], [topLeft, topRight, bottomRight, bottomLeft])
                
                self.markers_copy = markers.copy()
            return markers
    
mar = Marker()
i = mar.detectArucoMarkers(image)
pass