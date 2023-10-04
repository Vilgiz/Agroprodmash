import cv2
import math
import numpy as np
from pyzbar import pyzbar
from math import sqrt
from kalmanfilter import KalmanFilter
from Queue import CircularQueue
import time

from Comp_vision import Vision
from Robot import Robot



    
if __name__ == '__main__':
    
    size = 6

    video = cv2.VideoCapture(1)
    Vis = Vision()
    kf = KalmanFilter()
    coord = CircularQueue(size)
    

    while True:
        ret, warped_image = video.read()
        warped_image = Vis.factor(warped_image)
        peredacha = warped_image

        cv2.waitKey(1)

        Vis.Find_contors(warped_image)
        Vis.Find_Rocks(warped_image)
        Vis.detect_cans_with_qr(peredacha)
        print(Vis.detect_cans_with_qr_code)
        #item = Vis.detect_cans_with_qr_code
        item = Vis.track_only_coord

        if item is not None:
            if item != []: 
                item = item[0]
                coord.enqueue(item)
                time.sleep(0.2)
                if size == coord.size:
                    data = coord.print_data()
                    #data = [[(50,300)],[(134,320)],[(152,280)],[(223,300)],[(240,320)],
                             #[(330,280)],[(364,300)],[(400,320)],[(459,280)],[(470,300)]] 
                    #cv2.circle(warped_image, Vis.pred, 8, (20, 0, 255), -1)
                    #cv2.namedWindow("Viwdeo") 
                    #cv2.imshow('Viwdeo', warped_image)
                    Vis.prediction(peredacha, data)
                    coord.dequeue()




## ! Все, нет нихуя больше пока в main































## ? А больше и не надо 00