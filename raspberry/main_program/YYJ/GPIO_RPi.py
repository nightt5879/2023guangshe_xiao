import RPi.GPIO as GPIO

try:    #下面是OLED需要使用的库。防止有没安装这个库，但是又想用别的功能
    from luma.core.interface.serial import i2c, spi
    from luma.core.render import canvas
    from luma.oled.device import ssd1306, ssd1325, ssd1331, sh1106
    from PIL import ImageFont
except:
    pass


try:   #防止屏幕I2C不行
    serial = i2c(port=1, address=0x3C)
    device = ssd1306(serial)
except:
    pass

#各种路径
FontPath = '/home/pi/Desktop/F_NEW_Version/YYJ/Font_OLED.ttf'  #字体的绝对路径


class L298N:    #这个是
    """ 树莓派使用L298N驱动模块

    Attributes:
        self.PMW1, self.PWM2, self, IN_1, IN_2, IN_3, IN_4:这些是L298N是对应的逻辑电平输入与使能端口引脚
        self.P1, self.P2:用于树莓派上实例化PWM
        self.Frequency:PWM波的占空比，该类中默认两个电机输出PWM频率相同
        self.DutyCycle1, self.DutyCycle2:分别对应PWM1与PWM2的占空比

    PS:
        请对照L298N上的引脚使用，基本命名对应的，不对看注释与开发资料自己改！！！
        如果遇到插上后调用发现运动逻辑不符合，请查看自己的接线并灵活调整输入电平引脚设置！！！
    """
    def __init__(self, PWM1, PWM2, IN_1, IN_2, IN_3, IN_4):
        """ 初始化L298N

        Args:
            :param PWM1: 对应控制A电机的PWM1
            :param PWM2: 对应控制B电机的PWM2
            :param IN_1: A电机的逻辑 1
            :param IN_2: A电机的逻辑 2
            :param IN_3: B电机的逻辑 1
            :param IN_4: B电机的逻辑 2

        :return: None
        """
        #下面是基本引脚
        self.PWM1 = PWM1
        self.PWM2 = PWM2
        self.IN_1 = IN_1
        self.IN_2 = IN_2
        self.IN_3 = IN_3
        self.IN_4 = IN_4
        #下面是PWM波的频率，用于小车操控默认两个频率相同
        self.Frequency = 100
        #将这些引脚设置为输出模式
        GPIO.setmode(GPIO.BOARD)    #编码为物理引脚
        GPIO.setup(self.PWM1,GPIO.OUT) 
        GPIO.setup(self.PWM2,GPIO.OUT)
        GPIO.setup(self.IN_1,GPIO.OUT)
        GPIO.setup(self.IN_2,GPIO.OUT)
        GPIO.setup(self.IN_3,GPIO.OUT)
        GPIO.setup(self.IN_4,GPIO.OUT)
        #下面是两个PWM的实例化
        self.P1 = GPIO.PWM(self.PWM1, self.Frequency)
        self.P2 = GPIO.PWM(self.PWM2, self.Frequency)
        #下面是两个PWM波的占空比,默认为30
        self.DutyCycle1 = 30
        self.DutyCycle2 = 30
        self.P1.start(self.DutyCycle1)
        self.P2.start(self.DutyCycle2)

    '''
    下面是基本运动部分
    '''
    def Forward(self, DutyCycle1, DutyCycle2):
        """ 向前走

        电平 10 10 电机向前转动 （电平描述顺序是 IN1 IN2, IN3 IN4）后面不再赘述

        Args:
            :param: A电机的占空比
            :param: B电机的占空比

        :return: None
        """
        self.DutyCycle1 = DutyCycle1
        self.DutyCycle2 = DutyCycle2
        self.P1.ChangeDutyCycle(self.DutyCycle1)
        self.P2.ChangeDutyCycle(self.DutyCycle2)
        GPIO.output(self.IN_1, GPIO.HIGH)
        GPIO.output(self.IN_2, GPIO.LOW)
        GPIO.output(self.IN_3, GPIO.HIGH)
        GPIO.output(self.IN_4, GPIO.LOW)

    def Backward(self, DutyCycle1, DutyCycle2):
        """ 向后走

        电平 01 01 电机向后转动

        Args:
            :param: A电机的占空比
            :param: B电机的占空比
        :return: None
        """
        # 01 01 电机向后转动
        self.DutyCycle1 = DutyCycle1
        self.DutyCycle2 = DutyCycle2
        self.P1.ChangeDutyCycle(self.DutyCycle1)
        self.P2.ChangeDutyCycle(self.DutyCycle2)
        GPIO.output(self.IN_1, GPIO.LOW)
        GPIO.output(self.IN_2, GPIO.HIGH)
        GPIO.output(self.IN_3, GPIO.LOW)
        GPIO.output(self.IN_4, GPIO.HIGH)

    def TurnLeft(self, DutyCycle1, DutyCycle2):
        """ 左旋转

        电平 01 10 左边向后旋转，右边向前旋转

        Args:
            :param: A电机的占空比
            :param: B电机的占空比

        :return: None
        """
        self.DutyCycle1 = DutyCycle1
        self.DutyCycle2 = DutyCycle2
        self.P1.ChangeDutyCycle(self.DutyCycle1)
        self.P2.ChangeDutyCycle(self.DutyCycle2)
        GPIO.output(self.IN_1, GPIO.LOW)
        GPIO.output(self.IN_2, GPIO.HIGH)
        GPIO.output(self.IN_3, GPIO.HIGH)
        GPIO.output(self.IN_4, GPIO.LOW)

    def TurnRight(self, DutyCycle1, DutyCycle2):
        """ 右旋转

        电平 10 01 左边向前旋转，右边向后旋转

        Args:
            :param: A电机的占空比
            :param: B电机的占空比

        :return: None
        """
        self.DutyCycle1 = DutyCycle1
        self.DutyCycle2 = DutyCycle2
        self.P1.ChangeDutyCycle(self.DutyCycle1)
        self.P2.ChangeDutyCycle(self.DutyCycle2)
        GPIO.output(self.IN_1, GPIO.HIGH)
        GPIO.output(self.IN_2, GPIO.LOW)
        GPIO.output(self.IN_3, GPIO.LOW)
        GPIO.output(self.IN_4, GPIO.HIGH)

    def Stop(self):
        """ 停止

        电平 00 00 两个电机停止

        Args: None

        :return: None
        """
        GPIO.output(self.IN_1, GPIO.LOW)
        GPIO.output(self.IN_2, GPIO.LOW)
        GPIO.output(self.IN_3, GPIO.LOW)
        GPIO.output(self.IN_4, GPIO.LOW)

class OLED_1306:
    """ OLED_1306屏幕

    Attributes:
        self.serial: i2c的端口定义
        self.device: 确定型号的驱动代码
        self.Size: 字体大小
        self.Font: 加载字体使用
        self.Board: 是否显示边界，如果为 1 显示边界，为 0 不显示边界。默认显示边界

    PS：引脚提示VCC,GND,SCL,SDA分别接 4, 6, 5, 3 (BOARD编码）
    """
    def __init__(self):
        """ 初始化OLED屏幕

        Args: None

        :return: None
        """
        self.serial = i2c(port=1, address=0x3C)
        self.device = ssd1306(serial)
        self.Size = 15
        self.Font = ImageFont.truetype(FontPath, self.Size)  #放在了同级目录下
        self.Board = 1

    def Show(self, Str = '请输入字符串', X = 0, Y = 0):
        """ 在某行某列显示字符串

        Args:
            :param str:
            :param x:
            :param y:

        :return: None
        """
        if self.Board == 1 :    #显示字体
            with canvas(device) as draw:
                draw.rectangle(device.bounding_box, outline="white", fill="black")
        with canvas(device) as draw:
            draw.text((X, Y), Str, font=self.Font, fill="white")    #显示字体
    
    def Show4Line(self, Str1, Str2, Str3, Str4):
        """ 显示四行字符
        仅仅适用于默认的 15 号的字体，刚好以 15 为一行切割四行

        Args:
            :param Str1: 第一行显示的字符串
            :param Str2: 第二行显示的字符串
            :param Str3: 第三行显示的字符串
            :param Str4: 第四行显示的字符串
            
        :return： None
        """
        with canvas(device) as draw:
            draw.text((0, 0), Str1, font=self.Font, fill="white")    #显示字体
            draw.text((0, 15), Str2, font=self.Font, fill="white")    #显示字体
            draw.text((0, 30), Str3, font=self.Font, fill="white")    #显示字体
            draw.text((0, 45), Str4, font=self.Font, fill="white")    #显示字体

class IO:
    """ 树莓派上的IO口

    Attributes:
        self.Coding: 树莓派引脚的编码模式
        self.Pin: 定义的引脚位置
        self.Mode: 引脚为输入还是输出
    """
    def __init__(self, Pin, Mode, Coding = 'BOARD'):
        """ 初始化IO口

        Args:
            :param Pin: 引脚编号
            :param Mode: 输入与输出模式选择
            :param Coding: 编码方式选择

        :return: None
        """
        if Coding == 'BOARD':
            self.Coding = GPIO.BOARD
        elif Coding == 'BCM':
            self.Coding = GPIO.BCM
        elif Coding == 'wiringPi':
            self.Coding = GPIO.wiringPi
        else:
            print('请输入正确的编码格式')
        GPIO.setmode(self.Coding)    #设置编码格式
        #下面是判断模式
        self.Mode = Mode
        if self.Mode == 'IN':
            self.Mode = GPIO.IN  #输入模式
        elif self.Mode == 'OUT':
            self.Mode = GPIO.OUT
        else:
            print('请输入IN或OUT')
        self.Pin = Pin    #引脚编码
        GPIO.setup(self.Pin, self.Mode)  #设置引脚
        self.PWM = None


    def SetPin(self, Pin, Mode):
        """ 初始化IO口
        Args:
            :param Pin: 引脚位置
            :param Mode: 引脚模式（输入与输出）

        :return: None
        """
        self.Pin = Pin
        self.Mode = Mode
        GPIO.setmode(self.Coding)
        if self.Mode == 'IN':
            GPIO.setup(Pin, GPIO.IN)  #输入模式
        elif self.Mode == 'OUT':
            GPIO.setup(Pin, GPIO.IN)  # 输出模式
        else:
            print("请输入IN或OUT")

    def OutHigh(self):
        """ 输出高电平

        :return: None
        """
        GPIO.output(self.Pin, GPIO.HIGH)

    def OUTLow(self):
        """ 输出低电平

        :return: None
        """
        GPIO.output(self.Pin, GPIO.LOW)

    def Input(self):
        """ 得到输入电平

        :return: 返回输入的逻辑电平是 1 还是 0
        """
        if self.Mode == GPIO.IN:
            Logic = GPIO.input(self.Pin)
        else:
            print('IO口为输出模式不能调用输入')
            Logic = None
        return Logic

    def SetPWM(self, Frequency):
        """ 将引脚设为PWM输出

        Args:
            :param Frequency: 设置的PWM的频率
            
        :return: None
        """
        self.PWM = GPIO.PWM(self.Pin, Frequency)
        self.PWM.start(0)   #默认为  0 
        
    def ChangePWM(self, DustCycle):
        """ 改变PWM的占空比

        Args:
            :param DustCycle: 设置的PWM的占空比
            
        :return: None
        """
        self.PWM.ChangeDutyCycle(DustCycle)
        
class PID:
    """ PID控制

    Attributes:
        self.K, self.Kp, self.Ki, self.Kd: 分别对于PID的三个系数，最外面的K是对整体进行变换
        self.Expval: 期望值
        self.Nowval: 现在的值
        self.NowErr: 现在的误差，即 P
        self.SumErr: 累计误差 即 I
        self.LastErr: 上次的误差 用于求 D

    """
    def __init__(self, K, Kp, Ki, Kd, ExpVal ):
        """ 初始化PID
        输入几个关键的参数

        Args:
            :param K:  最外面的系数
            :param Kp: P的系数
            :param Ki: I的系数
            :param Kd: D的系数
            :param ExpVal: 期望值

        :return: None
        """
        self.K = K
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.ExpVal = ExpVal
        self.NowVal = 0
        self.NowErr = 0
        self.SumErr = 0
        self.LastErr = 0

    def OneDin(self, NowVal):
        """ 一维PID
        进行最经典的PID算法，适用于一维情况，例如电机需要转到某个转速。

        Args:
            :param NowVal: 现在值

        :return:
            PID: 计算出来的PID值
        """
        self.LastErr = self.NowErr
        self.NowVal = NowVal
        self.NowErr = self.NowVal - self.ExpVal
        self.SumErr += self.NowErr

        P = self.NowErr
        I = self.SumErr
        D = self.NowErr - self.LastErr

        PID = self.K * (self.Kp * P + self.Ki * I + self.Kd * D)
        return PID

    def TwoDin(self, NowVal, Differ):
        """ 二维PID
        对于二维的情况，可以根据自己输入的值来作为D。例如视觉寻线，
        这样输入的好处是可以"更加得到赛道的弯曲变化程度"。

        Args：
            :param NowVal: 现在的值
            :param Differ: 输入的 D 值

        :return:
            PID: 计算出来的PID值
        """
        self.NowVal = NowVal
        self.NowErr = self.NowVal - self.ExpVal
        self.SumErr += self.NowErr

        P = self.NowErr
        I = self.SumErr
        D = Differ

        PID = self.K * (self.Kp * P + self.Ki * I + self.Kd * D)
        return PID

    def Reset(self):
        """ 重置PID
        清除所有数据，用于再次新的调用，就不需要重复初始化（对于一个相同的期望与输入）

        Args: None

        :return: None
        """
        self.NowErr = 0
        self.SumErr = 0
        self.LastErr = 0
