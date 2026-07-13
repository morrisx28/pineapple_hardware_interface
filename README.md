# Pineapple Hardware Interface
DDS hardware interface for CSL wheel-biped robot
## Overview
### Hardware
- Joint motors: Damiao DM8006 DM8009P
- Wheel motors: Damiao DM6006
- IMU: Xsens Mti320

## Requirement
- USB2CANFD firmware version (app v1.0.0.1)
- Damiao motor firmware version (v3 or v4)
- Mti-320 (IMU) baudrate: 115200 (must enable quatenion output at MT Manger software)

## Build
1. Follow this repo to install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2).
2. Set up IMU lib
    ```
    cd ~/pineapple_hardware_interface/src/xspublic
    make clean
    make
    sudo usermod -G dialout -a $USER
    ```
3. Set up motor lib
    ```
    sudo apt install libusb-1.0-0
    sudo nano /etc/udev/rules.d/99-usb.rules
    ```
    add following line
    ```
    SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    ```
    cd ~/pineapple_hardware_interface
    mkdir build
    cd build
    cmake ..
    make
    ```
## Useage

The robot variant is selected by a config file (default: `../config/config.yaml`, the wheeled robot).
All motor parameters (CAN IDs, motor types, offsets, directions, position limits) live in the config,
so the same binary runs every robot.

```
cd ~/pineapple_hardware_interface/build
sudo ./pineapple_hardware_interface                          # wheeled robot (config.yaml)
sudo ./pineapple_hardware_interface ../config/config_v3.yaml  # v3 wheeled robot
sudo ./pineapple_hardware_interface ../config/config_arm.yaml # 6-DOF arm
```
