import cv2
from util import LCD_2inch4

#import RPi.GPIO as GPIO
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(18,GPIO.IN)

from PIL import Image, ImageFilter
disp = LCD_2inch4.LCD_2inch4()
disp.Init()
disp.clear()
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    image = image.resize((320, 240), Image.ANTIALIAS)
    image = image.filter(ImageFilter.SHARPEN)
    disp.ShowImage(image)
    
    #if GPIO.input(18) == 0:
    #    break

cap.release()
