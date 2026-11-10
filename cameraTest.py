import cv2

# Принудительно открываем через V4L2 бэкенд
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Ошибка: Не удалось открыть камеру через V4L2")
    exit()

# Явно задаем стандартное небольшое разрешение, чтобы избежать проблем с памятью
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Видео запущено. Нажмите 'q' в окне видео, чтобы выйти.")

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Ошибка: Не удалось получить кадр")
        break

    cv2.imshow('Raspberry Pi Camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
