# Autonomous Flight

Autonomous flight contains 5 test files and a launch file for ROS Noetic/Gazebo.

The launch file can be edited to launch the 5 test files.

These test files can be sequentially used to test the different autonomous flight functionalities.


# Motor control

The motor control files can be used to control the motor wirelessly using MQTT/Mosquitto. This requires two raspberry pis running linux (Rpi 4B+ was used). The RPi i2c bus needs configuring for motor control. Mosquitto MQTT needs installing for wireless communication.


## Configure I2C and install motorkit library 

The motorkit library needs i2c to communicate with the motor. This can be easily achieved using i2c tools and smbus libraries. Smbus or smbus2 may be required, so installing both is recommended.

``` sudo apt-get install -y python3-smbus ```

``` sudo apt-get install -y i2c-tools python3-pip ```

``` sudo pip3 install smbus2 ```

test i2c with:

``` sudo i2cdetect -y 1 ```

Install the motorkit library with:

``` sudo pip3 install adafruit-circuitpython-motorkit ```


## Mosquitto MQTT setup

``` sudo apt-get install mosquitto ```

``` wget http://repo.mosquitto.org/debian/mosquitto-repo.gpg.key ```

``` sudo apt-key add mosquitto-repo.gpg.key ```

``` cd /etc/apt/sources.list.d/ ```

``` sudo wget http://repo.mosquitto.org/debian/mosquitto-wheezy.list ```

``` sudo apt-get update ```

This is for broker:

``` sudo apt-get install mosquitto ```


## RPi 1 (connected to the motor driver board/motor):

BSS_master.py can be ran using (sudo must be used) in terminal:

``` sudo python3 bss_master.py ```

This will call the other motor control python files.

This RPi will then continue monitoring the "EFS/target_pos" MQTT topic. 



## RPi 2 (on UAV or near the system):

Transmit a signal to move the motor in terminal:

``` mosquitto_pub -t "EFS/target_pos" -m 5 ```

Where 5 is the target position.

