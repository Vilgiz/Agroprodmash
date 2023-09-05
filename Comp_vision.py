import cv2
import math
import numpy as np
from Camera import Camera


class Vision():

    def __init__(self) -> None:
        self.track_cans = {}
        self.track_id = 0
        self.cans_prev_frame = []
        self.cans_curr_frame = []
        self.count = 0
        self.param1 = 1
        self.param2 = 0.26  # 0.43

    def Find_contors(self, frame):
        self.count += 1
        self.cans_curr_frame = []
        self.cans_curr_frame = []

        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(self.gray, cv2.HOUGH_GRADIENT_ALT, 1, 75, param1=self.param1, param2=self.param2,
                                   minRadius=120, maxRadius=130)
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
        self.__show_circle(frame)

        # ? if self.count == 1:
            # ? cv2.createTrackbar('param1', 'frame', 1, 1000, self.__onChange1)   
            # ? cv2.createTrackbar('param2', 'frame', 1, 1000, self.__onChange2) 

    def __show_circle(self, frame):
        for self.obj_id, pt in self.track_cans.items():
            self.center_cv = pt
            cv2.circle(frame, self.center_cv, self.radius_cv, (255, 0, 255), 3)
            cv2.circle(frame, self.center_cv, 3, (0, 255, 0), 3)
            cv2.putText(frame, str(self.obj_id),
                        (pt[0], pt[1] - 7), 0, 1, (0, 0, 255), 2)
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

    def __detect_QR(self, frame):
        """ for self.obj_id, pt in self.track_cans.items():
            self.center_cv = pt
            cv2.circle(frame, self.center_cv, self.radius_cv, (255, 0, 255), 3)
            cv2.circle(frame, self.center_cv, 3, (0, 255, 0), 3)
            cv2.putText(frame, str(self.obj_id),
                        (pt[0], pt[1] - 7), 0, 1, (0, 0, 255), 2) """
        
        pass



if __name__ == '__main__':

    camera = Camera()
    Vis = Vision()

    while True:
        warped_image = camera.get_image()
        cv2.waitKey(1)

        Vis.Find_contors(warped_image)
        Vis.Find_Rocks(warped_image)
        print(Vis.track_only_coord)
