import cv2
import numpy as np
import serial
import time

# Настройки камеры и видео
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Настройки последовательного порта для связи с полётным контроллером
SERIAL_PORT = "/dev/ttyUSB0"  # или /dev/ttyAMA0
BAUD_RATE = 9600

# Инициализация последовательного порта
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Подождать инициализации порта

# Настройка камеры
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# Основной цикл
while True:
    ret, frame = cap.read()
    if not ret:
        print("Ошибка захвата кадра")
        continue

    # --- Обработка изображения ---
    # Простой пример фильтрации по цвету (огонь: оттенки красного/оранжевого)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_fire = np.array([0, 150, 150])
    upper_fire = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_fire, upper_fire)

    # Находим контуры
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Берём контур с максимальной площадью
        fire_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(fire_contour)
        
        # Рисуем рамку для визуализации
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)

        # Определяем пиксельные координаты центра обнаруженного пожара
        fire_px_x = x + w//2
        fire_px_y = y + h//2

        # Формируем пакет данных для полётного контроллера
        packet = f"FIRE:1;PX:{fire_px_x};PY:{fire_px_y};\n"
        ser.write(packet.encode('utf-8'))
        print(f"Отправка пакета: {packet.strip()}")

    else:
        # Если пожар не обнаружен
        packet = "FIRE:0;\n"
        ser.write(packet.encode('utf-8'))
        print("Пожар не обнаружен")

    # Показ видео (для отладки)
    cv2.imshow("Fire Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Освобождение ресурсов
cap.release()
cv2.destroyAllWindows()
ser.close()
