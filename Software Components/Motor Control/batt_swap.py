import RPi.GPIO as GPIO
import time
import time
import board
import sys

from adafruit_motor import stepper
from adafruit_motorkit import MotorKit


## Variables
C_P = int(sys.argv[1])
T_P = int(sys.argv[2])
Moves = abs(C_P-T_P)

##C_P,T_P
Scen = [[0,0,0,0,0,0,0,0,0],
        [0,0,5,5,5,5,2,2,2],
        [0,6,0,5,5,5,2,2,2],
        [0,6,6,0,5,5,2,2,2],
        [0,6,6,6,0,617,2,2,2],
        [0,6,6,6,6,0,2,2,2],
        [0,1,1,1,1,1,0,4,4],
        [0,1,1,1,1,1,3,0,4],
        [0,1,1,1,1,1,3,3,0]]
        
A_V = [0,0,0,0,0,0,2,1,0]

##Initialise Sensor
kit = MotorKit(i2c=board.I2C())

if Scen[C_P][T_P] == 1:
##RS->LS Scen 1
    M = 223*(T_P + A_V[C_P])
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
       
elif Scen[C_P][T_P] == 2:
##LS->RS Scen 2
    M = 223*(C_P + A_V[T_P])
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        time.sleep(0.01)

elif Scen[C_P][T_P] == 3:    
    M = 223*Moves
##RS->RS/Up Scen 3
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
       

elif Scen[C_P][T_P] == 4:    
    M = 223*Moves
##RS->RS/Down Scen 4
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
      
    
elif Scen[C_P][T_P] == 5: 
    M = 223*Moves
##LS->LS/UP Scen 5 
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
        
elif Scen[C_P][T_P]  == 6:    
    M = 223*Moves
##LS->LS/Down Scen 6
    for i in range(M):
        kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
 



        
