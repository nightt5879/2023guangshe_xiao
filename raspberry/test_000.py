from control import gpio
import time
from YYJ import Vision
import cv2
# wb = cv2.xphoto.createSimpleWB()
Cam = Vision.Camera(0, 320, 240)
# Cam2 = Vision.Camera(1, 320, 240)

def PIDLineTracking(K, Kp, Ki, Kd,Line,SumMax):
    # 初始化摄像头

    # 初始化PID模块
    PWM_L = PWM_R = 0  # 赋初值
    i = 0
    print(time.time())
    while True:
        Cam.ReadImg(0, 320, 0, 150)
        # Cam2.ReadImg(0, 320, 0, 240)
        #de
        Centre, Sum, Dst = Cam.LineTracking(Cam.Img, Line)
        Cam.ShowImg(Cam.Img)
        # Cam2.ShowImg(Cam2.Img,"cam2")
        Cam.ShowImg(Dst, 'Dst')
        # print(Centre[100])
        # Now = Centre[Line]  # 现在的值
        # Future = Centre[Line - 10]  # “未来”要去到的值
        # D = Future - Now  # 差值
        sum = Sum[Line - 50] + Sum[Line - 40] + Sum[Line - 45]  # 黑色像素点的数量
        # Sum = Sum[Line] + Sum[Line - 20]  # 黑色像素点的数量
        # if Sum >= SumMax:
        #     break
        print(Centre[Line],sum)
        # i += 1
        # if i >= 100:
        #     break
        Cam.Delay(1)


if __name__ == '__main__':
    K = 0.5
    Kp = 2
    Ki = 0
    Kd = 0
    Line = 120
    SumMax = 350
    PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax)
    time.sleep(1)
    print("done")
    print(time.time())
    PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax)
    # while True:
    #     PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax)
    # for i in range (100):
    #     S = gpio.Screen()
    #     S.screen_display("hellodfdf" + str(i))
    #     time.sleep(0.5)
