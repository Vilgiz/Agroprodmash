import cv2
import math
import numpy as np
from pyzbar import pyzbar
from math import sqrt
from Queue import CircularQueue
from Robot import Robot
import time
import json

class Vision():

    def __init__(self) -> None:
        self.track_cans = {}
        self.track_id = 0
        self.cans_prev_frame = []
        self.cans_curr_frame = []
        self.count = 0
        self.param1 = 1
        self.param2 = 0.545  # 0.43
        self.focus = 0
        self.center_of_QR = []
        self.detect_cans_with_qr_code = []
        self.brightness_factor = 0.725
        self.contrast_factor = 1.661
        self.saturation_factor = 1.371
        self.pred = None
        self.B_color_low = 1
        self.R_color_low = 1
        self.G_color_low = 1
        self.B_color_hight = 4
        self.R_color_hight = 254
        self.G_color_hight = 254

        self.elapsed_time = 1.4
        self.end_line = None
        self.start_line = None

        self.size = 16

    def Find_contors(self, frame):
        self.count += 1
        self.cans_curr_frame = []
        self.cans_curr_frame = []

        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        circles = cv2.HoughCircles(self.gray, cv2.HOUGH_GRADIENT_ALT, 1, 75, param1=self.param1, param2=self.param2,
                                   minRadius=130, maxRadius=150)
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
        #self.__detect_QR(frame)        
        self.__show_circle(frame)

        if self.count == 1:
            # ? cv2.createTrackbar('param1', 'Video', 1, 1000, self.__onChange1)   
            # ? cv2.createTrackbar('param2', 'Video', 1, 1000, self.__onChange2) 

            # ? cv2.createTrackbar('brightness_factor', 'Video', 1, 2000, self.__onbrightness_factor) 
            # ? cv2.createTrackbar('contrast_factor', 'Video', 1, 2000, self.__onContrast_factor) 
            # ? cv2.createTrackbar('saturation_factor', 'Video', 1, 2000, self.__onsaturation_factor) 

            # ? cv2.createTrackbar('__B_color_low', 'Video', 1, 254, self.__B_color_low) 
            # ? cv2.createTrackbar('__R_color_low', 'Video', 1, 254, self.__R_color_low) 
            # ? cv2.createTrackbar('__G_color_low', 'Video', 1, 254, self.__G_color_low) 
            # ? cv2.createTrackbar('__B_color_hight', 'Video', 1, 254, self.__B_color_hight) 
            # ? cv2.createTrackbar('__R_color_hight', 'Video', 1, 254, self.__R_color_hight) 
            # ? cv2.createTrackbar('__G_color_hight', 'Video', 1, 254, self.__G_color_hight) 
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

    def __B_color_low(self, value6):
        self.B_color_low = value6

    def __R_color_low(self, value6):
        self.R_color_low = value6

    def __G_color_low(self, value6):
        self.G_color_low = value6

    def __B_color_hight(self, value6):
        self.B_color_hight = value6

    def __R_color_hight(self, value6):
        self.R_color_hight = value6

    def __G_color_hight(self, value6):
        self.G_color_hight = value6    

    def detect_cans_with_qr(self):
        self.detect_cans_with_qr_code = []
        for cans in self.track_only_coord:
            if self.center_of_QR is not None:
                for qr in self.center_of_QR:
                    distance = sqrt((cans[1] - qr[1])**2 + (cans[0] - qr[0])**2)
                    if distance <= 150:
                        self.detect_cans_with_qr_code.append(cans)
                        #print("YES, YES, YES, YES ,YES, YES, YES")
            else:
                break


    def factor(self, frame):
        brightened_image = cv2.convertScaleAbs(frame, alpha=self.brightness_factor, beta=0)

        contrasted_image = cv2.convertScaleAbs(brightened_image, alpha=self.contrast_factor, beta=0)

        hsv_image = cv2.cvtColor(contrasted_image, cv2.COLOR_BGR2HSV)
        hsv_image[:, :, 1] = hsv_image[:, :, 1] * self.saturation_factor
        saturated_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

        return saturated_image

    def find_qr_alt(self):
        self.center_of_QR = []  
        for i in self.track_only_coord:
            self.qr_there = 0
            frame = self.HSV
            white = cv2.imread('white.jpg')
            center = i
            side_length = int(self.radius_cv * np.sqrt(2))
            top_left = (center[0] - side_length // 2, center[1] - side_length // 2)
            bottom_right = (center[0] + side_length // 2, center[1] + side_length // 2)
            cv2.rectangle(frame, top_left, bottom_right, (0, 0, 255), 2)
            y1 = top_left[1]
            y2 = bottom_right[1]
            x1 = top_left[0]
            x2 = bottom_right[0]
            if y1 >= 0:
                if y2 >= 0:
                    if x1 >= 0:
                        if x2 >= 0:

                            cropped_image = frame[y1:y2, x1:x2]
                            white = white[y1:y2, x1:x2]
                            #height, width = cropped_image.shape
                            height, width = cropped_image.shape[:2]


                            threshold_area = 1000 

                            low_red = (self.B_color_low, self.R_color_low, self.G_color_low)                                       # ? Ползунки
                            high_red = (self.B_color_hight, self.R_color_hight, self.G_color_hight)                                # ? Ползунки

                            mask = cv2.inRange(cropped_image, low_red, high_red)

                            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Поиск контуров

                            for contour in contours:
                                area = cv2.contourArea(contour)  # Вычисление площади контура
                                if area > threshold_area:  # Проверка площади на пороговое значение
                                    center = i                  
                                    cv2.putText(mask, "QR!", center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2) 
                                    self.center_of_QR.append(center)
                                    #print("FIND")

                            #cv2.namedWindow("white") 
                            #cv2.imshow('white', white)

                            #cv2.namedWindow("Videdo") 
                            #cv2.imshow('Videdo', cropped_image)

                            cv2.namedWindow("Vierdedo") 
                            cv2.imshow('Vierdedo', mask)

    def new_prediction(self, img, positions):
        i = 0
        img = img[0:1200, :]                                                 
        self.cofficient = self.time_calibration()
        self.distance_of_prediction = 520 + self.cofficient                 # ! 520 - расстояние предсказания в пикселях от левого края 
        xList = [item for item in range(0, self.distance_of_prediction)]    # ! Я ЕГО ПОМЕНЯЛ - НАСТРОЙ КАК ТЕБЕ НУЖНО!          
        for pt in positions:
            center = [pt[0][0], pt[0][1]]
            if i == Vis.size-1:
                pass
            else:
                position = [positions[i+1][0][0], positions[i+1][0][1]]
                cv2.line(img, center, position, (250, 100, 0), 2)
            cv2.circle(img, center, 8, (220, 0, 0), -1)
            cv2.putText(img, str(i), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)      
            i += 1
        pos_x = []
        pos_y = []
        for xy in positions:
            pos_x.append(xy[0][0])
            pos_y.append(xy[0][1])

        A,B,C = np.polyfit(pos_x, pos_y, 2)  

        for x in xList:
            y = int(A * x ** 2 + B * x + C)  
            cv2.circle(img, (x,y), 2, (0, 255, 0), cv2.FILLED)
            if x == self.distance_of_prediction-1:                   
                cv2.circle(img, (x,y), 9, (0, 0, 255), cv2.FILLED)
                self.right_predcition = (x,y)
        return img

    def sort(self):
        self.detect_cans_with_qr_code
        self.track_only_coord

        self.Grabable_cans = []

        if self.track_only_coord is not None:
            if self.track_only_coord != []:
                if self.detect_cans_with_qr_code == []:
                    self.Grabable_cans.append(self.track_only_coord)
                    #print("Add!ADDADDADD")

        for i in self.track_only_coord:
            for j in self.detect_cans_with_qr_code:
                if i != j:
                    self.Grabable_cans.append([i])
                    #print("Add!")
                else:
                    pass

    def coord_transform(self):
        self.X = Vis.right_predcition[1]
        self.Y = Vis.right_predcition[0]


    def start_timer(self):
        return time.time()                                  # Возвращаем текущее время


    def stop_timer(self, start_time):
        end_time = time.time()                               # Получаем текущее время
        self.elapsed_time = end_time - start_time            # Вычисляем прошедшее время
        return self.elapsed_time
        
    def time_calibration(self):
        distance = math.sqrt((self.end_line[0][1] - self.start_line[0][1])**2 + (self.end_line[0][0] - self.start_line[0][0])**2)
        self.speed_cans = distance / self.elapsed_time        # пиксель в секунду
        average_time = 1                                      # среднее время прохождения банки
        temp = average_time - self.elapsed_time
        temp = temp ** 2
        temp = math.sqrt(temp)
        Kol_vo_pixel_shift = 5 // temp
        if average_time <= self.elapsed_time:
            self.cofficient = Kol_vo_pixel_shift       #поздно
        if average_time > self.elapsed_time:
            self.cofficient = - Kol_vo_pixel_shift       # РАНО
        #self.cofficient = 1
        return int(self.cofficient/10)                          # ! /10 - калибровка силы влияния поправки на предсказание
                                                                # ! 10 - это очень много, почти не вляет, НАСТРОЙ!
        

if __name__ == '__main__':
    

    video = cv2.VideoCapture(0)
    Vis = Vision()
    Vis.size = 16
    coord = CircularQueue(Vis.size)
    robot_tsp = Robot(print_debug = True)



    robot_tsp.speed = 3000
    robot_tsp.coord = 3000
    robot_tsp.start()
    robot_tsp.send_start()
    cotun = 0
    coord.size = 0

    while True:
        ret, warped_image = video.read()
        warped_image = Vis.factor(warped_image)
        peredacha = warped_image

        cv2.waitKey(1)

        Vis.Find_contors(warped_image)
        Vis.Find_Rocks(warped_image)                         # * все банки


        if Vis.track_only_coord is not None:
            if Vis.track_only_coord != []:
                Vis.find_qr_alt()
        
        Vis.detect_cans_with_qr()

        Vis.sort()                                           # ? сортируем

        if Vis.Grabable_cans == []:
            cotun += 1
            if cotun >= 10:
                coord.dequeue()
                coord.size = 0
                coord.tail = 0
                cotun = 0

        item = Vis.Grabable_cans

        if item is not None:
            if item != []: 
                item = item[0]
                coord.enqueue(item)
                coord.size += 1

                if coord.size == 1:
                    start_time = Vis.start_timer()
                    print("Таймер включен.")
                    Vis.start_line = coord.queue[0]


                if Vis.size == coord.size:

                    Vis.elapsed_time = Vis.stop_timer(start_time)
                    print(f"Прошло времени: {Vis.elapsed_time} секунд.")
                    Vis.end_line = coord.queue[Vis.size-1]


                    data = coord.print_data()
                    Vis.new_prediction(peredacha, data)
                    coord.dequeue()
                    coord.size = 0
                    coord.tail = 0
                    Vis.coord_transform()              # ! Координаты
                    with open("coff.json", "r") as file:
                        coff = json.load(file)
                    height, width, channels = peredacha.shape
                    height = height / coff
                    X = Vis.X / coff
                    Y = Vis.Y / coff
                    Y = height - Y
                    print(X)
                    print(Y)

                    cv2.line(peredacha, (Vis.start_line[0][0], 0), (Vis.start_line[0][0], 1200), (0, 0, 250), 2)
                    cv2.line(peredacha, (Vis.end_line[0][0], 0), (Vis.end_line[0][0], 1200), (0, 0, 250), 2)

                    cv2.imshow('Line', peredacha)

                    robot_tsp.speed = X
                    robot_tsp.id = Y
                    robot_tsp.send_step(Y, X)
                    robot_tsp.__cast = lambda id, speed = robot_tsp.speed, coord = robot_tsp.coord: robot_tsp.conn.sendall(
                        f'cast;{int(speed)};{int(coord)};'.encode())
                    pass
                    