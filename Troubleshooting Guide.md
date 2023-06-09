# Troubleshooting Guide

## Mechanical system
The main issue that the system was facing at the point of the handover was motor jittering, causing the motor to jam leading to system failure.  This section is a troubleshooting guide for the motor and carousel assembly.

### Permenant Solution 
* Swapping the current motor with a better and overall, more reliable motor (more torque & speed) will permanently solve the issue. 
* Adding additional motors for redundancies would ensure continual function of the system even if one or more motor is down.

### Current Solution
* Possible reasons of failures are as follows, motor overheated, uneven carousel track surfaces, the hot glue melted due to motor overheat and friction, overwhelming friction leading to motor failure, motor gear got stuck between carousel gears.
* Motor overheated: wait for it to cooldown, add heatsink to the bottom of the motor.
* Uneven carousel track surfaces: Print the entirety of the bottom section of the rotating carousel piece/ add lubricant to the bottom of the carousel assembly. You will notice that for the current system we added additional guiding rails to minimize uneven surfaces and lots of lubrication (we used bike grease).
* Hot glue melted due to motor overheat and friction: Shut down the system, remove the hot glue on the motor gear, then re-add the hot glue onto the gear and wait for it to cool. This usually happens when the motor overheats or too much friction on the carousel assembly leading to the motor gear to slip from the shaft. Remember to switch off the motor when not in use, and let the system cool down between interval usages. OR get a better motor that does not rely on frictional forces acting on the shaft.
* Overwhelming friction leading to motor failure: Add more lubrication, fix the uneven surfaces with higher quality acryl manufacturing capabilities with smoother surfaces to reprint acryl components with uneven surfaces (rotating carousel). OR better motor, print entire bottom rotating carousel piece as a whole piece.
* Motor gear got stuck between carousel gears: the distance between the gear and the carousel assembly matters a lot here, try adjusting the position of the distance between the motor gear and the carousel gear to find the optimal distance to provide rotational forces while not jamming the system. OR design a better gear with better gear ratio.
* Bolt down the Battery Transfer Unit and lubricate it.

## Battery Cage & Module
* Possible reasons for failure, module fails to rotate and swap into the cage.
* Possible Fixes: Due to the Holybro drone legs not being fixated, it might cause the battery cage mounted below it to be tilted and unlevelled. By changing the position of the drone legs, the level of the cage can be adjusted. Also ensure that battery cage is tied to the bottom of the UAV correctly.
* Another fix, clip the sharp edges of the battery modules so it has smooth edges. This is to make sure that the module slots into the cage correctly.

## Alignment system:
* The state-of-the-art wooden alignment system (popsicle sticks) might be ensured that its alignment perpendicular to the system so the IR sensor can align the modules accurately. However, please replace it popsicle sticks with a better alignment model.

## Blade components:
This can be considered future work however due to the University’s workshop manufacturing capabilities the blade component was not manufactured to size. Shrink the height of the female blade component and extend the metallic nickel plates across to better ensure connectively. Also shrink the height of the male blade component platform to ensure when both components are connected, they are levelled.
