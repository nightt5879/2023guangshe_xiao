# The servo module, which is used to control the servo motor
# the class HalfCircleServo for the 180° servo and class FullCircleServo for the 360° servo

# import the necessary modules
# create three threads to control the servo and read the angle
import threading
import RPi.GPIO as GPIO
import time
import pygame


class Servo:
    """
    The servo class, which is used to control the servo motor

    :param pin: the servo's gpio pin
    """

    def __init__(self, pin):
        self.pwm = None
        self.pin = pin
        self.target = 0

    def set_mode(self):
        GPIO.setup(self.pin, GPIO.OUT, initial=False)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(0)
        
    @staticmethod
    def clamp_number(num, min_number, max_number):
        return max(min(num, max(min_number, max_number)), min(min_number, max_number))


class HalfCircleServo(Servo):
    """
    The half circle servo class, which is used to control the 180° servo motor, can directly set the angle

    :param pin: the servo's gpio pin
    """
    def __init__(self, pin):
        super().__init__(pin)
        self.angle = 0
        self.set_mode()

    def set_mode(self):
        super().set_mode()
        # initialize the servo's angle
        self.set_angle(self.angle)

    def set_angle(self, angle):
        angle = self.clamp_number(angle, 0, 180)
        if self.target != angle:
            self.target = angle
        self.pwm.ChangeDutyCycle(2.5 + 10 * angle / 180)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)
        # update the servo's angle
        self.angle = angle

    def get_angle(self):
        return self.angle
    
    
def control_servo(vertical_servo, rotate_angle, vertical_angle: int, horizontal_angle: int):
    vertical_servo.target = vertical_angle
    rotate_angle = horizontal_angle


if __name__ == '__main__':
    """
    the process of the create the thread function
    """
    def thread_nodding():
        while True:
            if servo1.target != servo1.angle:
                servo1.set_angle(servo1.target)
            time.sleep(0.5)


    def thread_rotating():
        global rotate_angle
        while True:
            if rotate_angle != servo2.angle + servo3.angle:
                rotate_angle = servo2.clamp_number(rotate_angle, 0, 360)
                if 0 <= rotate_angle <= 180:
                    # check the servo3's angle
                    if servo3.angle != 0:
                        servo3.set_angle(0)
                    servo2.set_angle(rotate_angle)
                elif 180 < rotate_angle <= 360:
                    servo2.set_angle(180)
                    print(rotate_angle)
                    servo3.set_angle(rotate_angle - 180)
            time.sleep(0.5)


    """
    the process of the initialization
    """
    # set the gpio mode to BCM
    GPIO.setmode(GPIO.BCM)

    # define the three servo
    servo1 = HalfCircleServo(4)
    servo2 = HalfCircleServo(17)
    servo3 = HalfCircleServo(27)

    # set the servo's rotate angle
    servo1.target = 90
    servo2.target = 0
    servo3.target = 0
    rotate_angle = 0

    # create two threads to control the servo and read the angle
    t1 = threading.Thread(target=thread_rotating)
    t2 = threading.Thread(target=thread_nodding)
    t1.start()
    t2.start()

    """
    the process of the control other parts and set the servo's angle
    """
    pygame.init()
    window = pygame.display.set_mode((100, 100))
    pygame.display.set_caption("keyboard_control")
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    servo1.target -= 10
                    print(servo1.target)
                elif event.key == pygame.K_DOWN:
                    servo1.target += 10
                    print(servo1.target)
                elif event.key == pygame.K_LEFT:
                    rotate_angle += 90
                    print(rotate_angle)
                elif event.key == pygame.K_RIGHT:
                    rotate_angle -= 90
                    print(rotate_angle)