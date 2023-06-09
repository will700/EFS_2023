import RPi.GPIO as GPIO
import time
import sys


sensor = 16
T = 0
state = 0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensor,GPIO.IN)

def position_check():
   if GPIO.input(sensor):
    state = 0
   else: 
    state = 1         
                
    return state
