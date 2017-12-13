import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(5, GPIO.OUT)
pwm_c = GPIO.PWM(5, 1000)

GPIO.setup(26, GPIO.OUT)
pwm_v = GPIO.PWM(26, 1000)

pwm_v.start(0)
pwm_c.start(0)

time.sleep(1)

pwm_v.start(50)
pwm_c.start(97)
print("after")

while True:
    coin_detect = int(input("Enter a number 0 or 1: "))
    if coin_detect == 0:
        pwm_v.ChangeDutyCycle(100)
        time.sleep(0.2)
        pwm_v.ChangeDutyCycle(50)
    elif coin_detect == 1:
        pwm_v.ChangeDutyCycle(0)
        time.sleep(0.2)
        pwm_v.ChangeDutyCycle(50)

GPIO.cleanup()
