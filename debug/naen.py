import cv2
import numpy as np
video = cv2.VideoCapture(1)
while(True):
    # Загрузка изображения

    ret, image = video.read()

# Преобразование изображения в оттенки серого
    """ gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)

    # Применение преобразования Хафа для поиска линий
    lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=100)

    # Отображение найденных линий на изображении
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 - 1000 * b)
            y1 = int(y0 + 1000 * a)
            x2 = int(x0 + 1000 * b)
            y2 = int(y0 - 1000 * a)
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2) """
    edges = cv2.Canny(image, 50, 150)  # Параметры 50 и 150 можно настроить для лучшего результата



    lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=100)  # Параметр порога можно настроить для лучшего результата


    min_width = 1
    max_width = 1
    min_height = 10000
    max_height = 10000

    for line in lines:
        rho, theta = line[0]
        if np.pi/4 < theta < 3*np.pi/4:  # Фильтруйте вертикальные линии
            x1 = int(rho / np.cos(theta))
            y1 = 0
            x2 = int((rho - image.shape[0] * np.sin(theta)) / np.cos(theta))
            y2 = image.shape[0]
            
            # Определите размер прямоугольников, которые вы ищете
            width = x2 - x1
            height = y2 - y1
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Отрисуйте прямоугольник на изображении
            
            if width > min_width and width < max_width and height > min_height and height < max_height:
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Отрисуйте прямоугольник на изображении

        

    # Отображение изображения с выделенными квадратами
    cv2.imshow('Dewcted S', image)

    cv2.waitKey(1)
cv2.destroyAllWindows()