import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

coin_detect = input("Enter a number 0 or 1: ")

GPIO.setup(18, GPIO.OUT)
pwm_c = GPIO.PWM(18, 1000)
pwm_c.start(50)

GPIO.setup(16, GPIO.OUT)
pwm_v = GPIO.PWM(16, 1000)
pwm_v.start(50)

# while coin_detect == 0 or coin_detect == 1:
if coin_detect == 0:
    pwm_v.ChangeDutyCycle(100)
    time.sleep(2)
    pwm_v.ChangeDutyCycle(50)
elif coin_detect == 1:
    pwm_v.ChangeDutyCycle(0)
    time.sleep(2)
    pwm_v.ChangeDutyCycle(50)

#GPIO.cleanup()
