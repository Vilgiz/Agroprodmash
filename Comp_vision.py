import cv2
import math
import numpy as np
from pyzbar import pyzbar
from math import sqrt
from kalmanfilter import KalmanFilter
from Queue import CircularQueue
import time

class Vision():

    def __init__(self) -> None:
        self.track_cans = {}
        self.track_id = 0
        self.cans_prev_frame = []
        self.cans_curr_frame = []
        self.count = 0
        self.param1 = 1
        self.param2 = 0.26  # 0.43
        self.focus = 0
        self.center_of_QR = []
        self.detect_cans_with_qr_code = []
        self.brightness_factor = 0.629
        self.contrast_factor = 1.12
        self.saturation_factor = 0.932
        self.pred = None

    def Find_contors(self, frame):
        self.count += 1
        self.cans_curr_frame = []
        self.cans_curr_frame = []

        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(self.gray, cv2.HOUGH_GRADIENT_ALT, 1, 75, param1=self.param1, param2=self.param2,
                                   minRadius=150, maxRadius=160)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            red_circles = []
            for i in circles[0, :]:
                red_circles.append((i[0], i[1], i[2]))
            for circle in red_circles:
                self.radius_cv = int(circle[2])
                x_cv = circle[0]
                y_cv = circle[1]
                self.center_cv = (x_cv, y_cv)
                self.cans_curr_frame.append((int(x_cv), int(y_cv)))

    def Find_Rocks(self, frame):
        if self.count <= 2:
            for pt in self.cans_curr_frame:
                for pt2 in self.cans_prev_frame:
                    self.distance = math.hypot(pt2[0] - pt[0], pt2[1] - pt[1])
                    if self.distance < 50:
                        self.track_cans[self.track_id] = pt
                        self.track_id += 1
        else:
            self.track_cans_copy = self.track_cans.copy()
            self.cans_curr_frame_copy = self.cans_curr_frame.copy()

            for self.obj_id, pt2 in self.track_cans_copy.items():
                self.obj_exists = False
                for pt in self.cans_curr_frame_copy:
                    self.distance = math.hypot(pt2[0] - pt[0], pt2[1] - pt[1])
                    if self.distance < 50:
                        self.track_cans[self.obj_id] = pt
                        self.obj_exists = True
                        if pt in self.cans_curr_frame:
                            self.cans_curr_frame.remove(pt)
                            continue
                if not self.obj_exists:
                    self.track_cans.pop(self.obj_id)
            for pt in self.cans_curr_frame:
                self.track_cans[self.track_id] = pt
                self.track_id += 1
        self.cans_prev_frame = self.cans_curr_frame.copy()
        self.__check()
        self.__detect_QR(frame)        
        self.__show_circle(frame)

        if self.count == 1:
            # ? cv2.createTrackbar('param1', 'Video', 1, 1000, self.__onChange1)   
            # ? cv2.createTrackbar('param2', 'Video', 1, 1000, self.__onChange2) 

            # ? cv2.createTrackbar('brightness_factor', 'Video', 1, 2000, self.__onbrightness_factor) 
            # ? cv2.createTrackbar('contrast_factor', 'Video', 1, 2000, self.__onContrast_factor) 
            # ? cv2.createTrackbar('saturation_factor', 'Video', 1, 2000, self.__onsaturation_factor) 
            pass

    def __show_circle(self, frame):
        for self.obj_id, pt in self.track_cans.items():
            self.center_cv = pt
            cv2.circle(frame, self.center_cv, self.radius_cv, (255, 0, 255), 3)
            cv2.circle(frame, self.center_cv, 3, (0, 255, 0), 3)
            cv2.putText(frame, str(self.obj_id),
                        (pt[0], pt[1] - 7), 0, 1, (0, 0, 255), 2)
        cv2.namedWindow("Video") 
        cv2.imshow('Video', frame)

    def __trans_coord(self):
        self.track_cans_temp = tuple(self.track_cans.items())
        self.track_only_coord = []
        for i in self.track_cans_temp:
            self.track_only_coord.append(i[1])

    def __check(self):
        self.__trans_coord()
        self.track_CANS = []
        for pip in range(len(self.track_only_coord)):
            if pip == len(self.track_only_coord) - 1:
                self.track_CANS.append(
                    [self.track_only_coord[pip][0], self.track_only_coord[pip][1]])
                break
            distance = math.sqrt((self.track_only_coord[pip][0] - self.track_only_coord[pip+1][1])**2 +
                                 (self.track_only_coord[pip][1] - self.track_only_coord[pip+1][1])**2)
            if distance > 5:
                self.track_CANS.append(
                    [self.track_only_coord[pip][0], self.track_only_coord[pip][1]])

    
    def __onChange1(self, value1):
        self.param1 = value1

    def __onChange2(self, value2):
        value2 = (value2 / 1000) - 0.001
        self.param2 = value2

    def __onbrightness_factor(self, value4):
        value4 = (value4 / 1000)
        self.brightness_factor = value4

    def __onContrast_factor(self, value5):
        value5 = (value5 / 1000)
        self.contrast_factor = value5

    def __onsaturation_factor(self, value6):
        value6 = (value6 / 1000)
        self.saturation_factor = value6

    def __detect_QR(self, frame):
        qr_codes = pyzbar.decode(self.gray)

        self.center_of_QR = []     
    
        for qr_code in qr_codes:

            qr_data = qr_code.data.decode("utf-8")
            
            (x, y, w, h) = qr_code.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            weight_division_by_two = w/2
            height_division_by_two = h/2
            center = (int(x + weight_division_by_two), int(y + height_division_by_two))
            cv2.circle(frame, center, 3, (0, 255, 0), 3)
            
            cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)               
            self.center_of_QR.append(center)  

    def detect_cans_with_qr(self, frame):
        self.detect_cans_with_qr_code = []
        for cans in self.track_only_coord:
            if self.center_of_QR is not None:
                for qr in self.center_of_QR:
                    distance = sqrt((cans[1] - qr[1])**2 + (cans[0] - qr[0])**2)
                    if distance <= 120:
                        self.detect_cans_with_qr_code.append(cans)
                        print("YES, YES, YES, YES ,YES, YES, YES")
            else:
                break
    
    def prediction(self, img, positions):
        i = 0
        for pt in positions:
            center = [pt[0][0], pt[0][1]]  # Извлекаем элемент из вложенного массива
            cv2.circle(img, center, 8, (220, 0, 0), -1)
            cv2.putText(img, str(i), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)      
            predicted = kf.predict(center[0], center[1])
            #cv2.circle(img, predicted, 8, (20, 0, 255), -1)
            print("tyda")
            print(predicted)
            cv2.namedWindow("Viwdeeeo") 
            cv2.imshow('Viwdeeeo', img)
            i += 1
        self.pred = predicted
        return img


if __name__ == '__main__':
    
    size = 10

    video = cv2.VideoCapture(0)
    Vis = Vision()
    kf = KalmanFilter()
    coord = CircularQueue(size)
    

    while True:
        ret, warped_image = video.read()

        cv2.circle(warped_image, Vis.pred, 8, (20, 0, 255), -1)
        cv2.waitKey(1)

        Vis.Find_contors(warped_image)
        Vis.Find_Rocks(warped_image)
        Vis.detect_cans_with_qr(warped_image)
        print("ottyda")
        print(Vis.detect_cans_with_qr_code)
        item = Vis.detect_cans_with_qr_code

        if item is not None:
            if item != []: 
                coord.enqueue(item)
                time.sleep(0.4)
                if size == coord.size:
                    data = coord.print_data()
                    #data = [[(50,300)],[(134,320)],[(152,280)],[(223,300)],[(240,320)],
                             #[(330,280)],[(364,300)],[(400,320)],[(459,280)],[(470,300)]] 
                    cv2.circle(warped_image, Vis.pred, 8, (20, 0, 255), -1)
                    cv2.namedWindow("Viwdeo") 
                    cv2.imshow('Viwdeo', warped_image)
                    Vis.prediction(warped_image, data)
                    coord.dequeue()
