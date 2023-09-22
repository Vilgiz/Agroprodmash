from kalmanfilter import KalmanFilter
import cv2

kf = KalmanFilter()

img = cv2.imread("blue_background.jpg")

ball1_positions = [(50, 100), (100, 150), (150, 200), (200, 250), (250, 300), 
                   (300, 350), (350, 400), (400, 450), (450, 500), (500, 550)]

for pt in ball1_positions:
    cv2.circle(img, pt, 15, (220, 20, 220), -1) # фиолетовый
    predicted = kf.predict(pt[0], pt[1])

    cv2.circle(img, predicted, 15, (20, 0, 20), 4)


cv2.imshow("Img", img)
cv2.waitKey(0)