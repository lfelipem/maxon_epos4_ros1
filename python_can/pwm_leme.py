import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
servo = GPIO.PWM(12,50)
servo.start(0)

duty = 2
while duty<=12:
    servo.ChangeDutyCycle(duty)
    time.sleep(1)
    duty = duty + 1


servo.ChangeDutyCycle(0)
servo.stop()
GPIO.cleanup()