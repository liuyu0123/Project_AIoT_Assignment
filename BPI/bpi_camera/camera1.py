import cv2

# 打开摄像头
# cap = cv2.VideoCapture('/dev/vedio20') #linux
cap = cv2.VideoCapture(0) #windows

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # 读取一帧图像
    ret, frame = cap.read()
    if not ret:
        print("Can not receive frame (stream end?). Exiting ...")
        break

    # 显示图像
    cv2.imshow('frame', frame)

    # 按下 ‘q’ 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()