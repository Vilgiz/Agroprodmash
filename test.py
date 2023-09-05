import cv2
from pyzbar import pyzbar

# Открытие видеопотока
video = cv2.VideoCapture(1)

while True:
    ret, frame = video.read()
    
    # Конвертация изображения в оттенки серого
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Обнаружение QR-кодов
    qr_codes = pyzbar.decode(gray)
    
    for qr_code in qr_codes:
        # Извлечение содержимого QR-кода
        qr_data = qr_code.data.decode("utf-8")
        
        # Извлечение координат контура QR-кода
        (x, y, w, h) = qr_code.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Рисование текста содержимого QR-кода
        cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
    cv2.imshow("QR Code Reader", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()