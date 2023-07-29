import cv2
import numpy as np

class Camera:
    """ 摄像头处理类

    Attributes:
        self.Img: 读取到的图像，设为全局变量
        self.X: 摄像头分辨率的X
        self.Y: 摄像头分辨率的Y
        self.cam: 初始化的摄像头

    """
    def __init__(self, num = 0, X = 320, Y = 240, camera=None):
        """ 摄像头初始化

        Args:
            :param num: 选择摄像头，如果只有一个摄像头默认为 0
            :param X: 摄像头分辨率的 X
            :param Y: 摄像头分辨率的 Y

        :return: None
        """
        #读取的图像
        self.Img = None
        #摄像头分辨率
        self.X = X
        self.Y = Y
        #摄像头初始化
        self.cam = camera if camera else cv2.VideoCapture(num) # GYJ_高端修改 适应各种使用情况
        self.cam.set(4, self.Y)
        self.cam.set(3, self.X)

    def GetOutLine(self, Img, CannyMin = 10, CannyMax = 100, Wmin = 50, Wmax = 100, Hmin = 50, Hmax = 100):
        """ 检测轮廓与得到轮廓中心RGB值

        Args:
            :param Img: 输入图像
            :param CannyMin: canny检测的下限值
            :param CannyMax: canny检测的上限值
            :param Wmin: 轮廓限制的 最小 宽度值
            :param Wmax: 轮廓限制的 最大 宽度值
            :param Hmin: 轮廓限制的 最小 高度值
            :param Hmax: 轮廓检测的 最大 高度值

        :return:
            ret: 判断是否有检测到轮廓（有为 1，无为 0）
            bin: canny检测后的图像
        """
        Imge = cv2.GaussianBlur(Img, (5, 5), 0)   #高斯滤波 平滑处理
        bin = cv2.Canny(Imge, CannyMin, CannyMax, apertureSize=3)  #canny边缘检测  ps:这里是单通道的图像
        contours, _hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   #找轮廓 cv2.findContours
        #print("轮廓数量：%d" % len(contours))
        # 轮廓遍历
        ret = 0
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)  # 外接矩形
            if Wmin <= w <= Wmax and Hmin <= h <= Hmax:
                ret = 1
                #画出框选的图，在原有的外接矩形上做了缩放
                cv2.rectangle(Img, (int(x + w*3/8), int(y + h*3/8)), (int(x + w*5/8), int(y + h*5/8)), (0, 255, 0), 2)
            else:
                ret = 0
        return ret, bin
    
    def GetRGB(self, Img, Y = 120, X = 160, Sight = 1):
        """ 得到某点的RGB色值

        Args:
            :param Img: 输入图像
            :param Y: 要得到的点的Y坐标
            :param X: 要得到的点的X坐标
            :param Sight: 是否画该点的校准准星，默认开启。输入 0 关闭

        :return:
            RGB: 该点的RGB色值 格式[R,G,B]
        """
        #下面两行是画一个准星，给使用者一个校准的方法
        if Sight == 1:
            cv2.circle(self.Img, (Y, X), 20, (0, 0, 255), 0)
            cv2.circle(self.Img, (Y,X), 2, (255, 0, 0), 0)
        RGB = Img[Y,X]      #得到BGR的值
        RGB = RGB[::-1]    #反转一下数组，变成RGB
        return RGB

    def ReadImg(self, X0 = 0 , X1 = 10000, Y0 = 0 , Y1 = 10000):
        """ 读取图像

        Args:
            :param X0: 要裁剪的图像的 X0（左上角点）
            :param X1: 要裁剪的图像的 X1（右下角点） 设置成10000即是默认不裁剪图像
            :param Y0: 要裁剪的图像的 Y0(左上角点）
            :param Y1: 要裁剪的图像的 Y1（右下角点） 设置成10000即是默认不裁剪图像

        :return: None
        """
        self.ret, Img = self.cam.read()
        if self.ret:
            self.Img = Img[Y0:Y1, X0:X1]    #裁剪坐标为[Y0:Y1, X0:X1]
        else:
            print('摄像头有问题')

    def ShowImg(self, Img, WindowName = 'Img'):
        """ 显示图像
        窗口默认名是输入图像的名字
        
        Args:
            :param Img: 输入的图像
            :param WindowName: 窗口的名字

        :return: None
        """
        cv2.imshow(WindowName, Img)

    def gamma_correction(self, img, correction):
        img = np.power(img / float(np.max(img)), correction)
        return img * 255
    def LineTracking(self, Img, Line = 100, Dilate = 1, condition = 0, IterationErode = 10, IterationDilate = 2):
        """ 视觉寻线图像处理
        将图像处理后二值化，具体用numpy数组逐行处理

        Args：
            :param Img: 输入图像
            :param Line: 要在哪一行画线，作为基准线
            :param Dilate: 是否要进行膨胀操作，如果为 1 进行。默认进行
            :param condition:  选择查找的是那哪种像素带点。 0：黑色，255：白色。默认为 黑色 腐蚀次数
            :param IterationDilate: 膨胀操作的次数
            :param IterationErode: 腐蚀操作的次数

        :return:
            CentreDict: 保存对应某一行上中间对应的中点的字典。Key为行数，Value为对应的中心点坐标
            CentreLen: 保存对应某一行上有多少符合要求的像素点，Key为行数，Value为对应的符合要求像素点个数
            Dst: 进行一系列图像处理后的最终图像，可以调用看处理的图像有没有问题
        """
        LineSum = Img.shape[0]  #输入的图片有几行
        cv2.line(Img, (0, Line), (Img.shape[1] - 1, Line), (255, 0, 0))   #画基准线
        CentreDict = {}     #用于保存 某一行对应的中心点 的字典
        CentreLen = {}    #用于保存 某一行对应的满足要求的点的数量 的字典

        # Gray = cv2.cvtColor(Img, cv2.COLOR_BGR2GRAY)    #灰度化
        # Blur = cv2.GaussianBlur(Gray, (5, 5), 0)    #高斯滤波
        # _, Dst = cv2.threshold(Blur, 95, 255, cv2.THRESH_BINARY)
        # Retval, Dst = cv2.threshold(Blur, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)  #二值化
        # 下面是GPT写的
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(Img, cv2.COLOR_BGR2HSV)

        # 设置黑色的HSV阈值范围
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 80])
        # 使用HSV阈值过滤黑色
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.dilate(mask, kernel, iterations=3)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.bitwise_not(mask)  # 反转一下图像
        Dst = mask  # 我需要的就是遮罩之后的图像
        if Dilate == 1:
            Dst = cv2.dilate(Dst, None, iterations=IterationDilate)   #膨胀操作，默认不进行，可选参数
        Dst = cv2.erode(Dst, None, iterations=IterationErode)    #腐蚀操作
        for i in range(LineSum):    #逐行处理
            try:
                where = np.where(Dst[i] == condition)  # 找到这行的符合要求的(x)
                Left = where[0][1]  # 最左边符合要求的像素点
                Right = where[0][-2]  # 最右边符合要求的像素带你
                Centre = int((Left + Right) / 2)
                CentreLen[i] = len(where[0])
                CentreDict[i] = Centre
                #下面是画出边界与中间线
                cv2.circle(Img, (Left, i), 1, (0, 0, 255), 1)  # 画左边的点 红色 小圆
                cv2.circle(Img, (Right, i), 1, (0, 0, 255), 1)  # 画右边的点 红色 小圆
                cv2.circle(Img, (Centre, i), 1, (0, 255, 0), 1)  # 画中间的点 绿色 小圆
            except:
                CentreLen[i] = 0
                CentreDict[i] = 0
                pass
        return CentreDict, CentreLen, Dst
    
    def Delay(self, time=1):
        """ 读取延时

        Args:
            :param time: 延时的时间，单位为 ms

        :return: None
        """
        try:
            cv2.waitKey(time)
        except KeyboardInterrupt:
            pass
    def Release(self):
        """ 释放摄像头

        Args: None

        :return: None
        """
        self.cam.release()