import RPi.GPIO as GPIO
import time

SERVO_PIN = 
MOTOR_PIN = 

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

angle_pwm = GPIO.PWM(SERVO_PIN, _)
speed_pwm = GPIO.PWM(MOTOR_PIN, _)

angle_pwm.start()
speed_pwm.start(0)

def set_servo_angle(angle):
    duty =  2.5 + (angle / 180) * 10.0
    duty = max(2.5, min(12.5, duty))
    angle_pwm.ChangeDutyCycle(duty)

def set_motor_speed(speed):
    duty = 2.5 + (speed / 100) * 10.0
    duty = max(2.5, min(12.5, duty))
    speed_pwm.ChangeDutyCycle(duty)

try:
    while True:
        angle = int(input())
        speed = int(input())
        
        set_servo_angle(angle)
        set_motor_speed(speed)
except:
    pass
angle_pwm.stop()
speed_pwm.stop()
GPIO.cleanup()