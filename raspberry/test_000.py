# 导入Python的RPi.GPIO模块，该模块提供控制Raspberry Pi GPIO通道的能力。
import RPi.GPIO as GPIO

# 设置GPIO编号模式为BOARD模式，该模式下引脚是按照物理位置进行编号的。
GPIO.setmode(GPIO.BOARD)

# 假设我们将GPIO引脚12设为输入
pin_number = 12

# 设置GPIO引脚为输入模式，使用GPIO内置的上拉电阻。
# 这意味着当开关未被按下时，引脚将被读取为HIGH。
GPIO.setup(pin_number, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 无限循环，持续检查引脚状态
while True:
    # GPIO.input函数读取指定GPIO引脚的值，HIGH或者LOW。
    input_state = GPIO.input(pin_number)
    print(input_state)
    # 如果input_state为False，说明引脚状态为LOW，即开关被按下。
    if input_state == False:
        print('Button Pressed')
        # 使用time.sleep进行延迟，以防止同时检测到多次按钮按下。
        time.sleep(0.2)
