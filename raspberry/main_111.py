from YYJ import GPIO_RPi
from YYJ import Vision
#陀螺仪需要的库
import smbus
import math
# 语音播放需要的库
import sys
import os
Path = '/home/pi/Desktop/F_NEW_Version/Drug/'   #存放录音文件的位置

import time

# 电源控制寄存器地址
power_regist = 0x6b

# I2C模块初始化
bus = smbus.SMBus(1)
# 外接I2C设备的地址
address = 0x68

#初始化L298N驱动
L= GPIO_RPi.L298N(18,16,35,37,33,31)
L.Stop()

#初始化OLED模块
OLED = GPIO_RPi.OLED_1306()

def OledStart():
    OLED.Show4Line('##########' ,'###任务开始###' ,'###任务开始###','###一挑三队###')
    
def Show( Str1, Str2, Str3, Str4):
    OLED.Show4Line('物品种类：' + Str1 ,'运送状态：' + Str2,'运送时间：' + Str3 + 's','完成次数：' + Str4)
    
def ShowDone():
    OLED.Show4Line('##########' ,'###任务完成###' ,'###任务完成###','###一挑三队###')

#初始化三个红外
InL = GPIO_RPi.IO(38, 'IN')
InR = GPIO_RPi.IO(40, 'IN')
InM = GPIO_RPi.IO(36, 'IN')

#初始化两个LED
Green = GPIO_RPi.IO(40, 'OUT')
Green.SetPWM(2)
White = GPIO_RPi.IO(38, 'OUT')
White.SetPWM(2)

#播放音频
def PlaySound(Name):
    os.system('mplayer %s' % Path + Name + '.aac')
        
# 读取一个字长度的数据(16位)
def readWord(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

# 将读取到的数据转换为原码 (有符号数本身是采用补码方式存储的)
def readWordReal(adr):
    val = readWord(adr)
    x = 0xffff
    # 首位为1 表示是负数
    if (val >= 0x8000):
        # 求原码
        return -((x - val)+1)
    else:
        return val

#获取Z轴转向加速度的数值
def GetZ():
    gyroZ = readWordReal(0x47)/131
    if abs(gyroZ) <= 1.5:  #防止零飘
        gyroZ = 0
    else:
        pass  #左边为正右边为负数
    return gyroZ
    

def Classify(array):
    #分类不同颜色的色快
    R = array[2]
    G = array[1]
    B = array[0]
    if R >= 80 and G <= 100 and B <= 100:
        PlaySound('1')
        Object = 1
    elif R <= 100 and G >= 100 and B >= 100:
        PlaySound('2')
        Object = 2
    elif R <= 100 and G >= 70 and B <= 100:
        PlaySound('3')
        Object = 3
    elif R <= 100 and G <= 100 and B >= 70:
        PlaySound('4')
        Object = 4
    else:
        Object = 0
    return Object


def TurnLeft(TurnTime = 7000):
    Flag = 0
    PlaySound('Left')
    #time.sleep(0.5) #给陀螺仪修正时间
    while True:
        Z = GetZ()
        Flag += Z
        if Flag >= TurnTime:
            break
        else:
            L.TurnLeft(40,40)
        time.sleep(0.01)
    L.Stop()
    print(Flag)
    print('Trun done')
    
def TurnRight(TurnTime = 7800):
    Flag = 0
    PlaySound('Right')
    #time.sleep(0.5) #给陀螺仪修正时间
    while True:
        Z = GetZ()
        Flag += Z
        if Flag <= -TurnTime:
            break
        else:
            L.TurnRight(40,40)
        time.sleep(0.01)
    L.Stop()
    print('Trun done')
    
def Forward():
    L.Forward(30,25)
    time.sleep(0.4)
    L.Stop()
    
def Turn180(TurnTime = 14500):
    ZAngle = 0
    #print('2:',ZAngle)
    while True:
        Z = GetZ()
        ZAngle += Z
        if ZAngle <= -TurnTime:
            break
        else:
            L.TurnRight(40,40)
        time.sleep(0.01)
    L.Stop()
    print('Turn 180 done')
#初始化PID模块
PID = GPIO_RPi.PID(0.2,1,0.005,2,160)

def PIDLineTracking(K, Kp, Ki, Kd, Line, Start, Max, SumMax):
    #初始化摄像头
    Cam =  Vision.Camera(0, 320, 240)
    #初始化PID模块
    PID = GPIO_RPi.PID(K,Kp,Ki,Kd,160)
    PWM_L = PWM_R = 0    #赋初值
    while True:
        Cam.ReadImg(0,320,20,150)
        Centre, Sum, Dst = Cam.LineTracking(Cam.Img,Line)
        Cam.ShowImg(Cam.Img)
        Cam.ShowImg(Dst,'Dst')
        Now = Centre[Line]   #现在的值
        Future = Centre[Line - 10]  #“未来”要去到的值
        D = Future - Now  #差值
        Sum = Sum[Line] + Sum[Line - 20]  #黑色像素点的数量
        PWM = PID.OneDin(Now)
        PWM_L = Start + PWM
        PWM_R = Start - PWM
        if PWM_L <= 0:
            PWM_L = 0
        elif PWM_L >= Max:
            PWM_L = Max
        if PWM_R <= 0:
            PWM_R = 0
        elif PWM_R >= Max:
            PWM_R = Max 
        L.Forward(PWM_L,PWM_R)
        
        if Sum >= SumMax:
            break
        Cam.Delay(1)
        print('PWM',PWM,'Sum',Sum,'D',Now-160)
    L.Forward(50,50)
    time.sleep(0.2)
    L.Stop()
    Cam.Release()
    print('Tracking done')
    
    
def PIDCorrect():
    '''
    Cam =  Vision.Camera(0, 320, 240)
    while True:
        Cam.ReadImg(0,320,20,150)
        Centre, Sum, Dst = Cam.LineTracking(Cam.Img)
        Cam.ShowImg(Cam.Img)
        Cam.ShowImg(Dst,'Dst')
        Error = Centre[40] - 160
        if Error >= 0:
            TurnRight(500)
        else:
            TurnLeft(500)
        break
        Cam.Delay(1)
    L.Stop()
    Cam.Release()
    print('Correct done')
    '''
    
def Start():
    Green.ChangePWM(50)   #LED闪烁
    Object = 0
    L.Stop()
    Cam =  Vision.Camera(0, 320, 240)
    while True:
        Cam.ReadImg(0,320,0,150)
        Cam.ShowImg(Cam.Img)
        array = Cam.Img[80,160]
        Object = Classify(array)
        Cam.Delay(1)
        if Object != 0:
            break
    Cam.Release()
    PlaySound('Sart')
    Green.ChangePWM(0)   #关闭
    print('Start done')
    return Object

def Job1():  #终点为十字路口
    PIDLineTracking(0.2, 2, 0, -0.1, 40, 70, 80, 300)
#     Forward()
    
def Job2():  #终点为停止点
    PIDLineTracking(0.2, 2, 0, -0.1, 40, 70, 80, 300)
#     Forward()
    Turn180()
    PlaySound('Finish')
    
StartFlag = 0  #用于开始的判断
Answer = 0 #物品的类别
Job = 0  #判断送了几次
OledStart()
StartTime = time.time()
while True:
    Middle = InM.Input()
    if StartFlag == 0:
        StartFlag = 1 #完成任务前不再进来
        Answer = Start()
        End = time.time()
        Show(str(Answer), '准备出发', str(round(End - StartTime,1)), str(Job))
        print('物品种类:',Answer)
    if Answer <= 2 and Middle == 1:
        Job1()
        if Answer == 1:
            TurnLeft()
            PIDLineTracking(0.2, 2, 0, 0, 40, 70, 80, 400)
            Turn180()
        elif Answer == 2:
            TurnRight()
            PIDLineTracking(0.2, 2, 0, 0, 40, 70, 80, 400)
            Turn180()
            #完成去到的任务
        White.ChangePWM(50)   #白灯开始闪烁
        End = time.time()
        Show(str(Answer), '准备返回', str(round(End - StartTime,1)), str(Job))
        while True:  #判断是否取下药品
            Middle = InM.Input()
            if Middle == 0:
                White.ChangePWM(0)  #关灯
                Job1()
                if Answer == 2:
                    TurnLeft()
                    Job2()
                else:
                    TurnRight()
                    Job2()
                Answer = 0 #重置物品种类
                StartFlag = 0  #重置开始旗帜
                Job += 1
                break
    elif Answer >= 3 and Middle == 1:
        Job1()
        Job1()  #去到第二个十字路口
        if Answer == 3:
            TurnLeft()
            PIDLineTracking(0.2, 2, 0, 0, 40, 70, 80, 400)
            Turn180()
        elif Answer == 4:
            TurnRight()
            PIDLineTracking(0.2, 2, 0, 0, 40, 70, 80, 400)
            Turn180()
            #完成去到的任务
        White.ChangePWM(50)   #白灯开始闪烁
        End = time.time()
        Show(str(Answer), '准备返回', str(round(End - StartTime,1)), str(Job))
        while True:  #判断是否取下药品
            Middle = InM.Input()
            if Middle == 0:
                White.ChangePWM(0)
                Job1()
                if Answer == 4:
                    TurnLeft()
                    Job1()
                    Job2()
                else:
                    TurnRight()
                    Job1()
                    Job2()
                Answer = 0 #重置物品种类
                StartFlag = 0  #重置开始旗帜
                Job += 1
                break
    if Job == 4:
        L.Stop()
        break

ShowDone()
PlaySound('JobDone')
print('job done easily')        
        
'''   
PIDLineTracking()
Forward()
TurnLeft()
PIDLineTracking()
print('第二阶段')
Turn180()
PIDLineTracking()
Forward()
TurnRight()
PIDLineTracking()
Turn180()
'''
    
