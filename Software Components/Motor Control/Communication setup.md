# Raspberry Pi configuration guide

A comprehensive guide to implement a number of functionalities on a Raspberry Pi 3A using ubuntu 20.04. Although this is configured to work with a Raspberry Pi 3A, much of it should work on different Pi's, especially a 4B.

## Table of contents  
  - [All Packages](#all-packages)
  - [Install Ubuntu](#install-ubuntu)
    * [Network Connection](#network-connection)
    * [Installing a Desktop](#installing-a-desktop)
    * [Configure static IP](#configure-static-ip)
    * [Configure Eduroam](#configure-eduroam)
    * [Reconfigure Wifi](#reconfigure-wifi)
  - [Install ROS](#install-ros)
    * [Make_catkin_ws](#make-catkin-ws)
    * [Install PX4](#install-px4)
    * [Install QGroundControl](#install-qgroundcontrol)
  - [Display live image on web server](#display-live-image-on-web-server)
  - [Mosquitto MQTT](#mosquitto-mqtt)
  - [USB camera setup](#usb-camera-setup)
    * [Basic camera capture](#basic-camera-setup)
    * [camera ROS config](#camera-ros-config)
    * [Display live image on web](#display-live-image-on-web)
  - [Install April Tag library](#install-april-tag-library)
  - [Configure I2C and install motorkit library](#configure-i2c-and-install-motorkit-library)

## Installing packages

Here is a list of the commands used to install all the packages used within this document. 

``` sudo apt-get install ros-indigo-noetic-view ```

``` sudo apt install ros-noetic-web-video-server ```

## Install Ubuntu

Installing Ubuntu, the operating system used by Raspberry Pi. ubuntu is used because each ROS version is designed to work with a specific Ubuntu version. The UAV uses ROS noetic, a stable version released in 2020, which is designed to work with Ubuntu 20.04. A good description of the installation can be found here: https://github.com/EEEManchester/drone_build_tutorial/blob/main/4_Experiment_OnboardComputer_Setup.md

Alternatively, follow these briefer steps:

boot using raspberry pi imager, ubuntu-server-20.0.4

It is important that the Raspberry Pi can easily connect to a network after the first boot, so that packages can be installed. One way to ensure this is to copy and paste network-config file from repository. Edit with correct SSID and passwork for network. A home network (that does not require installing a certificate, like eduroam) is recommended.

save + exit, then eject the SD card

### Network Connection

Check the internet connection with:

``` ping google.com ```

if error, continue

``` sudo nano /etc/resolv.conf ```

Add to bottom of file

``` nameserver 8.8.8.8 ```

``` ctrl + s ```, then ``` ctrl + x ```

reboot with ``` sudo reboot now ```

``` ping google.com ```

### Installing a Desktop

``` git clone https://github.com/wimpysworld/desktopify.git ```

``` cd desktopify ```

``` sudo ./desktopify --de ubuntu-mate ```

reboot with ``` sudo reboot now ```

### Configure static IP

It is beneficial to configure the Raspberry Pi to have a static IP on the network. This will mean the RPi's IP address will be the same each time it connects to the network, so can be easily accessed through SSH.

Useful link: https://www.tomshardware.com/how-to/static-ip-raspberry-pi

Current ipv4 address, used for SSH, STATIC IP:
``` hostname -I ```

Router ip address, ROUTER IP:
``` ip r ```

DNS ip address, DNS IP:
``` grep "nameserver" /etc/resolv.conf ```

``` sudo apt install dhcpcd5 ```

``` sudo nano /etc/dhcpcd.conf ```

Change to

``` 
interface [INTERFACE]
static_routers=[ROUTER IP]
static domain_name_servers=[DNS IP]
static ip_address=[STATIC IP ADDRESS YOU WANT]/24
```

For example

```
interface wlan0
static_routers=192.168.7.1
static domain_name_servers=192.168.1.1
static ip_address=192.168.7.121/24
```

reboot with ``` sudo reboot now ```

Can be accessed with SSH using ip_address

### Configure Eduroam

This requires a desktop. If you manage this without a desktop please let me know how.

Change datetime:

``` timedatectl set-ntp 0 ```

``` timedatectl set-time 21:45:53 ```

``` timedatectl set-time 2019-04-10 ```

``` timedatectl set-ntp 1 ```

Install python3

``` sudo apt update ```

``` sudo apt install python3 ```

Download installer from: https://www.itservices.manchester.ac.uk/wireless/eduroam/

Run the installer and enter username and password (user is a12345@manchester.ac.uk)

``` cd Downloads ```

``` python3 eduroam-linux-TUoM-manchester.ac.uk ```

``` chmod +x eduroam-linux-TUoM-manchester.ac.uk.py ```

Now should be able to connect as a usual network.

### Reconfigure Wifi

Manually add or change network connections.

``` sudo nano /etc/netplan/50-cloud-init.yaml ```

only use spaces

``` sudo netplan generate ```

``` sudo netplan apply ```

## Install ROS

``` sudo sh -c ‘echo “deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main” > /etc/apt/sources.list.d/ros-latest.list’ ```

``` sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 ```

``` sudo apt update ```
 
``` sudo apt install curl ```

``` sudo apt update ```

``` sudo apt install ros-noetic-ros-base ```

``` ls -al ```

``` sudo nano .bashrc ```

Add the following lines to the bottom of the document and save: ``` source /opt/ros/noetic/setup.bash ```

``` sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ```

``` sudo rosdep init ```

``` rosdep update ```

### Make catkin ws

This is the workspace used for development in ROS. More of these can be made for each project. Generally, a package or library is installed for one catkin, and then source useful packages when needed. For general development, use the main catkin_ws, and automatically source all packages each time a terminal window is opened.

``` sudo apt-get install ros-noetic-catkin python3-catkin-tools ```

``` mkdir -p ~/catkin_ws/src ```

``` cd ~/catkin_ws ```

Tell .bashrc that this is the catkin_ws you are using. Bashrc runs each time the terminal is opened. This will then configure packages with the correct catkin_ws address.

``` source ~/.bashrc ```

``` catkin_make ```

### Install PX4

PX4 is the autopilot software used by the drone.

``` cd src ```

``` git clone -b v1.12.3 https://github.com/PX4/PX4-Autopilot.git --recursive ```

``` cd PX4-AUTOPILOT ```

``` git describe ```

This should output: v1.12.3.

``` cd .. ```

PX4 General Dependencies:
``` bash ./PX4-Autopilot/Tools/setup/ubuntu.sh ```

Then reboot

``` cd catkin_ws/src/PX4-Autopilot ```

if no desktop:
``` DONT_RUN=1 make px4_sitl_default gazebo ```

or
if desktop installed:
``` DONT_RUN=0 make px4_sitl_default gazebo ```

This will then run a simulation of the drone.

### Install QGroundControl

QGroundControl is

https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

``` sudo usermod -a -G dialout $USER ```

``` sudo apt-get remove modemmanager -y ```

``` sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y ```

``` sudo apt install libqt5gui5 -y ```

``` sudo apt install libfuse2 -y ```

``` sudo add-apt-repository ppa:alexlarsson/flatpak ```

``` sudo apt update ```

``` sudo apt install flatpak ```

``` flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo ```

``` flatpak install flathub org.kde.Platform/aarch64/5.15-21.08 ```

``` flatpak remote-add --if-not-exists thopiekar.eu https://dl.thopiekar.eu/flatpak/_.flatpakrepo ```

``` flatpak install thopiekar.eu org.mavlink.qgroundcontrol ```


## Mosquitto MQTT

``` sudo apt-get install mosquitto ```

``` wget http://repo.mosquitto.org/debian/mosquitto-repo.gpg.key ```

``` sudo apt-key add mosquitto-repo.gpg.key ```

``` cd /etc/apt/sources.list.d/ ```

``` sudo wget http://repo.mosquitto.org/debian/mosquitto-wheezy.list ```

``` sudo apt-get update ```

This is for broker:

``` sudo apt-get install mosquitto ```



For client:
``` sudo apt-get install mosquitto-clients ```


Terminal 1:
``` mosquitto ```

Terminal 2:
``` mosquitto_sub -v -t 'test/topic' ```

Terminal 3:
``` mosquitto_pub -t 'test/topic' -m 'helloWorld' ```

If you get Error: Address already in use

``` ps -ef | grep mosquitto ```

``` sudo kill 11333 ```

To connect between devices:
``` mosquitto_pub -h <ipv4> -t 'test/topic' -m 'helloWorld' ```

On Windows 

``` cd C:\Program Files\mosquitto ```

``` mosquitto.exe -v ```

``` mosquitto_pub.exe -h <ipv4> -t "test/topic" -m "helloWorld" ```

``` mosquitto_sub.exe -h <ipv4> -v -t "test/topic" ```

``` sudo apt install python3-pip ```

``` pip install paho-mqtt ```


## USB camera setup

### Basic camera capture

This will install relevant libraries and allow single jpg capture.

``` sudo apt-get install fswebcam ```

``` mkdir usb_captures ```

``` cd usb_captures ```

``` fswebcam -r 1200x800 im_1.jpg ```

Open to verify capture. 


### camera ROS config

On leader (base station)

Use ``` hostname -I ``` to find leader and follower IPs.

sudo apt install net-tools

``` 
export ROS_IP='{{LEADER_IP}}'
export ROS_MASTER_URI="{{LEADER_IP}}:11311"
```

```
echo 'export ROS_IP="{{LEADER_IP}}"' >> ~/.bashrc
echo 'export ROS_MASTER_URI="http://{{LEADER_IP}}:11311"' >> ~/.bashrc
```

or add the lines 

```
export ROS_MASTER_URI="http://{{LEADER_IP}}:11311"
export ROS_IP="{{LEADER_IP}}"
```

to bashrc

check with 

vim .bashrc

type ```:q``` then ```enter``` to leave

source ~/.bashrc




On follower

```
export ROS_IP="{{FOLLOWER_IP}}"
export ROS_MASTER_URI="http://{{LEADER_IP}}:11311"
```

```
echo 'export ROS_IP="{{FOLLOWER_IP}}"' >> ~/.bashrc
echo 'export ROS_MASTER_URI="http://{{LEADER_IP}}:11311"' >> ~/.bashrc
```

or add the lines
```
export ROS_IP="{{FOLLOWER_IP}}"
export ROS_MASTER_URI="http://{{LEADER_IP}}:11311"
```
to bashrc

source .bashrc

check if connected with ```lsusb```

check videos

``` ls /dev | grep video* ```

install ros cam

``` sudo apt install ros-noetic-usb-cam ```

launch cam test file

``` cat /opt/ros/noetic/share/usb_cam/launch/usb_cam-test.launch ```

only usb_cam node will start, image_view will fail because no gui resources on rpi

This is just to verify launch file




start roscore on leader

``` roscore ```





then start usb_cam on follower
``` roslaunch usb_cam usb_cam-test.launch ```

may have to disable firewall

``` sudo apt install ufw ```

``` sudo ufw disable ```

``` ctrl + z ```
``` bg ``` to background

rostopic list

in gui could rqt_graph

on master configure 

``` sudo apt-get install ros-noetic-view ```

``` rosrun image_view image_view.image:=/usb_cam/image_raw ```

This should open the image viewing app.

### Display live image on web

Display the live image from the previous session in a web browser.

Install the web-video-server library

``` sudo apt install ros-noetic-web-video-server ```

cd ~/catkin_ws

``` catkin_make ```

Source, so that libraries can be used in this catkin.

``` source devel/setup.bash ```

``` cd src ```

``` catkin_create_pkg vidsrv std_msgs rospy roscpp ``` 

``` mkdir -p vidsrv/launch ``` 

``` nano vidsrv/launch/vidsrv.launch ```

Copy and paste the vidsrv.launch file that can be found in this Github directory.

``` cd .. ```

``` catkin_make ```

``` on master: roscore ``` 

on follower: ``` roslaunch vidsrv vidsrv.launch ```

on web browser: ``` RPI_IP:8080 ```

if multiple roscores are running you can kill with ``` killall -9 rosmaster ```


## Install April Tag library

April tag is a visual system widely used in robotics. We install the ROS package to configure our vision and co-ordinate transformation system.

In user directory execute:

``` cd ~/catkin_ws/src ```

``` git clone https://github.com/AprilRobotics/apriltag.git  ```

``` git clone https://github.com/AprilRobotics/apriltag_ros.git ```

``` cd .. ```

``` rosdep install --from-paths src --ignore-src -r –y ```

``` catkin_make_isolated ```

## Configure I2C and install motorkit library 

The motorkit library needs i2c to communicate with the motor. This can be easily achieved using i2c tools and smbus libraries. Smbus or smbus2 may be required, so installing both is recommended.

``` sudo apt-get install -y python3-smbus ```

``` sudo apt-get install -y i2c-tools python3-pip ```

``` sudo pip3 install smbus2 ```

test i2c with:

``` sudo i2cdetect -y 1 ```

Install the motorkit library with:

``` sudo pip3 install adafruit-circuitpython-motorkit ```

## Install mqtt-explorer

sudo apt update

sudo apt install snapd

sudo snap install mqtt-explorer

### install ros bridge

cd src

git clone https://github.com/groove-x/mqtt_bridge 

cd .. 

catkin_make_isolated

cd src

pip3 install -r dev-requirements.txt

### install mqtt client 

Start mosquitto broker

terminal 1: ``` mosquitto -v ```

edit /opt/ros/noetic/share/mqtt_client/launch/params.yaml

add to ros2mqtt:
``` 
ros_topic: /EFS/ros_topic_1
mqtt_topic: EFS/mqtt_topic_1
```
ensure .bashrc addresses are correct

run mosquitto -v

roscore

roslaunch mqtt_client standalone.launch



### Creating a publisher in ROS

sudo apt-get install --reinstall coreutils

cd ~/catkin_ws/src

catkin_create_pkg comms rospy

cd catkin_ws

catkin_make

cd catkin_ws/src/comms/src

add publisher.py file

for some reason this must be created with 

``` vim publisher.py ``` which can be saved and exited with ``` ctrl + c, :wq!, enter ```

chmod +x publisher.py

terminal 1: roscore

terminal 2: rosrun comms publisher.py

terminal 3: rostopic echo /EFS_ros_output


### Whole System

T1:

cd catkin_ws

mosquitto & roslaunch mqtt_client standalone.launch 

T2:

mosquitto_sub -t EFS

T3: 

python3 bss_efs.py & python3 bms_efs.py

T4:

cd catkin_ws

source devel/setup.bash

rosrun comms bridge_2.py

T5:

python3 uav_efs.py



``` sudo apt-get install rpi.gpio ```
