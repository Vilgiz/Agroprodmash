
import cv2
import numpy as np

# Инициализация фильтра Калмана для предсказания траекторий
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

# Открытие видеопотока
video = cv2.VideoCapture(1)

while True:
    # Считывание кадра из видеопотока
    ret, frame = video.read()
    
    # Преобразование кадра в оттенки серого
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Применение размытия для снижения шума
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Обнаружение объектов с использованием алгоритма Хафа
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 75,
                               param1 = 0.5, param2 = 0.26, minRadius=130, maxRadius=140)
    
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        
        for (x, y, r) in circles:
            # Рисование красной точки в центре обнаруженного круга
            cv2.circle(frame, (x, y), r, (0, 0, 255), 4)
            
            # Обновление состояния фильтра Калмана
            kalman.correct(np.array([[x], [y]], dtype=np.float32))
            prediction = kalman.predict()
            
            # Рисование предсказанной траектории
            #cv2.circle(frame, (int(prediction[0]), int(prediction[1])), 5, (255, 0, 0), -1)
    
    # Отображение кадра
    cv2.imshow("Video", frame)
    
    # Прерывание цикла по нажатию клавиши 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Освобождение ресурсов
video.release()
cv2.destroyAllWindows()