import cv2
import math
import json
import os

# Load the input image
cap = cv2.VideoCapture(0)
ret, image = cap.read()
image = cv2.imread("i.png")

class Marker:
    def __init__(self, id, center, corners):
            self.id = id
            self.center = center
            [self.topLeft, self.topRight, self.bottomRight, self.bottomLeft] = corners

class ImageProcessor():
    def detectArucoMarkers(self, image):
            markers = {}

            arucoDictionary = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50)
            
            arucoParameters = cv2.aruco.DetectorParameters()

            (corners, ids, rejected) = cv2.aruco.detectMarkers(
                image, arucoDictionary, parameters=arucoParameters)

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
    def s(self):
        markers = self.__detectArucoMarkers(image)

    

ip = ImageProcessor()
cap = cv2.VideoCapture(1)
ret, image = cap.read()

s = ip.detectArucoMarkers(image)
print(s[0].bottomLeft)
cv2.circle(image, s[0].bottomLeft, 8, (255, 0, 255), -3)
cv2.circle(image, s[0].topRight, 8, (255, 0, 255), -3)
cv2.circle(image, s[0].bottomRight, 8, (255, 0, 255), -3)
cv2.circle(image, s[0].topLeft, 8, (255, 0, 255), -3)
cv2.line(image, s[0].bottomLeft, s[0].topLeft, (250, 100, 0), 2)
real_dlina = 58
pixel_dlina_Y  = s[0].bottomLeft[0] - s[0].topLeft[0]
pixel_dlina_X  = s[0].bottomLeft[1] - s[0].topLeft[1]
print(pixel_dlina_Y)
print(pixel_dlina_X)
print(real_dlina)
pixel_dlina = math.sqrt((pixel_dlina_X) ** 2 + (pixel_dlina_Y) ** 2)
print(pixel_dlina)

coff = pixel_dlina / real_dlina

result =  pixel_dlina / coff

print("###")
print(result)
#cv2.namedWindow("Video") 
#cv2.imshow('Video', image)
#cv2.waitKey(0)


with open("coff.json", "w") as file:
    json.dump(coff, file)

pass