# MushrBus is forked from the [MuSHR](https://mushr.io/) Project

Multi-agent System for non-Holonomic Racing or MuSHR platform is designed for indoor vehicles; this fork takes it outdoors with the ultimate goal of using full-scale micro-vehicles. Indoor systems such as overhead cameras are not permitted. GNSS will be added.
This repository will contain code to implement a system on the Jetson Nano and flash an SD card.

The MuSHR Bus is a scale model of the modular bus: http://viewer.zmags.com/publication/a58eb17c#/a58eb17c/26. The lead bus vehicle is driven manually, but all following vehicles are completely automated. The behavior to be shown includes:
1) Accelerating from a parked state to join onto the rear of the bus platoon
2) Driving bumper-to-bumper when platooned with vehicles exerting minimal force on each other
3) Separating from and leaving the platoon at the destination bus stop

## Installation

The commands present in this guide assume that the target jetson nano has access to the internet, and has the base image installed as described [here](https://github.com/YoungJonathanP/MicroAV/tree/212c8c7034e3d3ae0a8a8b40605f7b38db1b0efb)
The instructions here supersede other installation instructions.

This repository uses a series of [Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

To clone with all files:
```
git clone --recurse-submodules https://github.com/elcano/MushrBus.git
```

Run the primary install script with administrator privileges
```
source MushrBus/MicroAV/microav_install_script.sh
```

All individual pieces will need to be moved into the **catkin workspace** and then built. Use this support command or move them manually.
```
cp -r ~/MushrBus/mushr_ackermann_drive ~/MushrBus/jetson-module ~/MushrBus/jetson-tracker ~/catkin_ws/src
```
Move into the catkin workspace
```
cd ~/catkin_ws/
```
Build the catkin projects we just added
```
catkin_make
```
Run the setup.sh
```
source devel/setup.sh
```

### Note
Install scripts and vehicle image setup found in [MicroAV](https://github.com/YoungJonathanP/MicroAV/tree/212c8c7034e3d3ae0a8a8b40605f7b38db1b0efb) folder