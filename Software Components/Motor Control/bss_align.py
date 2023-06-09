import RPi.GPIO as GPIO
import time
import time
import board
import sys

from adafruit_motor import stepper
from adafruit_motorkit import MotorKit


## Variables
Trial= int(sys.argv[1])
M = 23


##Initialise Sensor
kit = MotorKit(i2c=board.I2C())

if Trial <= 2:
##RS->LS Scen 1
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
       
elif Trial <= 8:
##LS->RS Scen 2
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
        

