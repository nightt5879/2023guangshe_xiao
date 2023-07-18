import RPi.GPIO as GPIO
import time


def button_with_wait():
    """
    按下按钮后，等待一段时间，如果没有再次按下按钮，就返回one_press 此时对应的是红队
    如果再次按下按钮，就返回two_press 此时对应的是蓝队
    :return: one_press or two_press
    """
    BUTTON_PIN = 21  # 按钮连接的GPIO口
    # 选择BCM模式
    GPIO.setmode(GPIO.BCM)
    # 设置GPIO口为输入
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    press_flag = 0
    press_flag_down = 0
    press_time = 0
    while True:
        # 如果按键被按下
        # if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        #     print('Button is not pressed')
        if press_flag == 1 and GPIO.input(BUTTON_PIN) == GPIO.HIGH:  # 按下松开后开始计时
            press_time += 1
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.1)  # 按键消除抖动
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                press_flag = 1
                if press_flag_down == 1:  # 第二次按下
                    print("two_press")
                    button = "two_press"
                    break
                # print('Button is pressed'
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH and press_flag == 1:
            time.sleep(0.1)
            if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
                press_flag_down = 1
        if press_time > 10:
            print("one_press")
            button = "one_press"
            break
        # print(press_time)
        # 暂停一段时间
        time.sleep(0.1)
    # GPIO.cleanup()  lcd需要使用不能清除GPIO口
    return button

if __name__ == '__main__':
    button_with_wait()
