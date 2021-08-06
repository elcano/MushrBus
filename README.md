# MushrBus is forked from the [MuSHR](https://mushr.io/) Project

Multi-agent System for non-Holonomic Racing or MuSHR platform is desiged for indoor vehicles; this fork takes it outdoors with the ultimate goal of using full-scale micro-vehicles. Indoor systems such as overhead cameras are not permitted. GNSS will be added.
This repository will contain code to implement a system on the Jetson Nano and flash an SD card.

The MuSHR Bus is a scale model of the modular bus: http://viewer.zmags.com/publication/a58eb17c#/a58eb17c/26. The lead bus vehicle is driven manually, but all following vehicles are completely automated. The behavior to be shown includes:
1) Accelerating from a parked state to join onto the rear of the bus platoon
2) Driving bumper-to-bumper when platooned with vehicles exerting minimal force on each other
3) Seoarating from and leaving the platoon at the destination bus stop

## Installation

This repository uses a series of [Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). To clone with all files:
```
$ git clone --recuse-submodules https://github.com/elcano/MushrBus.git
```

### Note
Install scripts and vehicle image setup found in **MicroAV** folder