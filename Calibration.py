import cv2
import numpy as np
import os
import json
from cv2 import ROTATE_90_CLOCKWISE
from random import randint


class Marker():
    def __init__(self, id, center, corners):
        self.id = id
        self.center = center
        [self.topLeft, self.topRight, self.bottomRight, self.bottomLeft] = corners


class ImageProcessor():
    def __init__(self):
        if os.path.isfile('processor.json'):
            self.__load_settings()
        else:
            self.__load_defaults()
        self.ID = (0.1,0.1)
        self.speed = 0

    def __load_defaults(self):
        self.rotation_angle = 0
        self.M = None
        self.save_settings()

    def __load_settings(self):
        with open('processor.json', 'r') as file:
            config = json.load(file)
        #self.rotation_angle = config['rotation_angle']
        #self.XOY = config['aruco_XOY']
        #self.scale = config['scale']
        try:
            self.M = np.array(config['rotation_matrix'])
            self.M_original_image = np.array(config['rotation_matrix_original'])
        except Exception:
            self.M = None
            self.M_original_image = None

    def save_settings(self):
        config = {}
        #config['rotation_angle'] = self.rotation_angle
        config['rotation_matrix'] = self.M.tolist()
        config['rotation_matrix_original'] = self.M_original_image.tolist()
        #config['aruco_XOY'] = self.XOY
        #config['scale'] = self.scale

        with open('processor.json', 'w') as f:
            json.dump(config, f)

    def __detectArucoMarkers(self, image):
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
    

    def perspective_correction(self, image, calibration = False):
        h,  w = image.shape[:2]
        if calibration:
            markers = self.__detectArucoMarkers(image)
            cv2.circle(image, markers[1].bottomLeft, 3, (0, 0, 255), 3)
            cv2.circle(image, markers[2].bottomLeft, 3, (0, 255, 0), 3)
            cv2.circle(image, markers[3].bottomLeft, 3, (255, 0, 0), 3)
            cv2.circle(image, markers[0].bottomLeft, 3, (255, 0, 255), 3)
            cv2.imshow('detectresult',image)
            start_p = np.float32([markers[1].bottomLeft,markers[2].bottomLeft,
                                  markers[3].bottomLeft,markers[0].bottomLeft])
            dest_p = np.float32([[860,-790],[860,210],
                                 [160,210],[160,-790]])
            self.M = cv2.getPerspectiveTransform(start_p,dest_p)

            dest_p_original_image = np.float32([[0,0],[1000,0],
                                 [1000,700],[0,700]])
            self.M_original_image = cv2.getPerspectiveTransform(start_p,dest_p_original_image)
            # Сохраняем М
            self.save_settings()
            result = cv2.warpPerspective(image, self.M,(w, h))
            result = result[0:800 ,0:800]
            result = cv2.resize(result, (0, 0), fx=0.5, fy=0.5)
            cv2.imwrite('rez.png',result)
            cv2.imshow('calibresult',result)
        else:
            result = cv2.warpPerspective(image, self.M,(w, h))
            result = result[0:740 ,0:740]
            result = cv2.rotate(result, ROTATE_90_CLOCKWISE)
            result = cv2.resize(result, (0, 0), fx=0.5, fy=0.5)
            return(result)

if __name__ == '__main__':

    ip = ImageProcessor()
    cam = cv2.VideoCapture(1)

    while True:

        ret, frame = cam.read()
        frame = cv2.imread("image.png")
        cv2.imshow('Video', frame)

        key = cv2.waitKey(1)
        if key == ord('p'):
            ip.perspective_correction(frame, calibration = True)
        if key == ord('l'):
            cv2.imshow('result',ip.perspective_correction(frame))

    

        