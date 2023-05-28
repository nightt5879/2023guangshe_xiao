import Vision
import cv2

Cam =  Vision.Camera(0, 320, 240)
while True:
    Cam.ReadImg()
    Cam.LineTracking(Cam.img)
    Cam.ShowImg(Cam.img)
    ret, array, bin = Cam.GetOutLineAndRGB()
    print(ret)
    print(array)
    Cam.ShowImg(bin,'bin')
    Cam.Delay(1)