import cv2

# 定义视频编解码器
fourcc = cv2.VideoWriter_fourcc(*'XVID')

# 创建 VideoWriter 对象
# 我们将结果保存为 output.avi
# FPS 值设为 20.0，分辨率设为(640,480)
out = cv2.VideoWriter('7_output.avi', fourcc, 20.0, (320, 240))

# 打开默认摄像头
cap = cv2.VideoCapture(1)
cap.set(4, 240)
cap.set(3, 320)

for i in range(500):
    # 从摄像头读取帧
    ret, frame = cap.read()
    # print(frame.shape)
    if ret:
        # 写入帧
        out.write(frame)

        # 在窗口中显示结果
        cv2.imshow('Recording', frame)

        # 按下 'q' 键停止录制
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# 释放所有资源
cap.release()
out.release()
cv2.destroyAllWindows()
