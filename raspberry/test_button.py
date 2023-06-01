import RPi.GPIO as GPIO
import time
def button_input():
    BUTTON_PIN = 18  # 按钮连接的GPIO口
    # 选择BCM模式
    GPIO.setmode(GPIO.BCM)
    # 设置GPIO口为输入
    GPIO.setup(BUTTON_PIN, GPIO.IN,pull_up_down=GPIO.PUD_UP)

    press_flag = 0
    press_time = 0
    while True:
        # 如果按键被按下
        # if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        #     print('Button is not pressed')
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.1) # 按键消除抖动
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                press_flag = 1
                # print('Button is pressed')
                press_time += 1
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH and press_flag == 1:
            time.sleep(0.1)
            if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
                print("short_press")
                button = "short_press"
                break
        if press_time > 10:
            print("long_press")
            button = "long_press"
            break
        print(press_time)
        # 暂停一段时间
        time.sleep(0.1)
    GPIO.cleanup()
    return button

def button_with_wait():
    BUTTON_PIN = 18  # 按钮连接的GPIO口
    # 选择BCM模式
    GPIO.setmode(GPIO.BCM)
    # 设置GPIO口为输入
    GPIO.setup(BUTTON_PIN, GPIO.IN,pull_up_down=GPIO.PUD_UP)

    press_flag = 0
    press_flag_down = 0
    press_time = 0
    while True:
        # 如果按键被按下
        # if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        #     print('Button is not pressed')
        if press_flag == 1 and GPIO.input(BUTTON_PIN) == GPIO.HIGH:  #按下松开后开始计时
            press_time += 1
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.1) # 按键消除抖动
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                press_flag = 1
                if press_flag_down == 1:  #第二次按下
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
        print(press_time)
        # 暂停一段时间
        time.sleep(0.1)
    GPIO.cleanup()
    return button

a = button_input()
time.sleep(1)
b = button_with_wait()
print(a,b)
