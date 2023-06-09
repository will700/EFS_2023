EFS Standard Operating Procedure (SOP)

# Setting Up the System
The EFS system comprises three main subsystems and one auxiliary subsystem. This section will outline the setup required for each before starting standard operation.

## Autonomous Flight
The autonomous flight subsystem encapsulates the onboard UAV hardware and software algorithms, both of which support flight and landing control. The physical setup information for the UAV is seen at https://github.com/EEEManchester/drone_build_tutorial/blob/main/4_Experiment_OnboardComputer_Setup.md. Setup for the PX4 autopilot unit and other hardware/software setup related to the UAV is at https://github.com/EEEManchester/drone_build_tutorial/blob/main/6_Vicon_Setup_Use.md.

There are three primary features that should be enabled by the UAV to work seamlessly with the EFS:

### Localisation
The VICON motion capture system is used to localise the UAV and BSS. However, only simulations of this have been completed. The physical and simulation setup instructions for VICON can be seen here: 

https://github.com/EEEManchester/drone_build_tutorial/blob/main/6_Vicon_Setup_Use.md (VICON Physical Setup)
https://github.com/EEEManchester/drone_simulation_tools (VICON Simulation Setup)

### AR Tag Detection
The AR tag detection is implemented using the April Tag (https://april.eecs.umich.edu/software/apriltag) family of markers. A ROS package for detecting and estimating April Tag pose is seen at http://wiki.ros.org/apriltag_ros. This package is used with the T3_AR.py script (seen in test scripts) and simulated in Gazebo. A physical test is also implemented using a Raspberry Pi 4b with the same package and script. The usb_cam ROS package (http://wiki.ros.org/usb_cam) is used to setup the physical R Pi camera and camera_callibration (http://wiki.ros.org/camera_calibration) is used to calibrate it and retrieve the camera_info file used in apriltag_ros.

### AR Tag Based Landing Control
A basic PID controller uses the x,y and z errors of the April tag pose estimation to generate the required linear velocity for accurate landing on the tag. This then fed into the PX4 controller which communicates this command to the motor. The T5_final.py script (seen in test scripts) is used to implement this. Keep in mind that it has only been simulated in Gazebo.

## Battery Swapping System (BSS)
The component information and assembly for the BSS is given in the final report and the CAD files section of the handover package. Assuming all components have been assembled and no changes have been made to the BSS as is the case after the handover, a few setup checks have to be performed before operation:

### System Integrity
A thorough check should be conducted to make sure all components are locked in properly and there are no misalignments. More specifically, the landing docks should be slotted in, battery modules should be in the respective storage slots, the motor should be locked in to the spokes of the carousel, and finally the whole rotating carousel should be of uniform level lest it jam during rotation.

### Lubricating the Ball Transfer Unit (BTU)
A core component responsible for the BSS rotation is the BTU. Hence, it needs to be lubricated well before system startup and consequently the rotation.

## Power Management
The primary setup for power management is to make sure the batteries are all connected to the battery modules through the connectivity electrodes and components. Lastly, all battery modules should contain a BMS PCB each while each slot on the carousel should contain a charger PCB.

## Communication Network
The instructions for setting up the communication network has been described in detail in the software section of this handover package.

# Safety precautions
A list of safety precautions related to the operation of the EFS has been described below:

* Flight cage with adequate protections should be used for any real-world UAV tests
## 	Ensure fire prevention kits are available in case of fire due to power components.
## Do not touch the BSS motor or carousel while in operation.
## 	All operators should know how to stop the system in case of emergency.
## Make sure there are no trailing cables and other tripping hazards.
## 	The UAV batteries should be risk assessed with fire proof storage bags.
## The motor heats up over longer durations of operation, do not touch.

# System start-up sequence
## 	Power on the R Pi 4b in the control unit, and the motor
## Power on the power supply
## 	Power on the base station PC and connect to the control unit R Pi 4b
## Instructions on possible motor commands can be found in the communication section

# System shut-down sequence
## 	Disconnect from the control unit R Pi 4b and power it off
## Switch off the power supply
## 	Wait till all systems have stopped and cooled
