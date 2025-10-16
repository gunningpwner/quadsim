# Overview
[Insert cool name here] is a bare bones flight control software that I wrote from scratch as a platform to prototype and experiment with guidance and controls algorithms.
Hardware related stuff is in the firmware folder
Currently unfinished SITL using Gazebo is in gazebo folder
Flight Algorithms are in libs/flight_controller

# Components
The firmware currently only supports the components and sensors I've used to build my quadcopter. It will probably work with others but has only been tested with the below:  
[Xilo Stax V3 F4 Flight Controller](https://www.getfpv.com/xilo-stax-v3-f4-flight-controller-30x30.html)  
[RadioMaster ExpressLRS RP4TD](https://www.getfpv.com/radiomaster-expresslrs-2-4ghz-receiver-w-antennas-rp4td.html)  
[Flywoo GOKU GM10 GPS and Compass](https://www.getfpv.com/flywoo-goku-gm10-mini-v3-gps-w-compass.html) (TODO)  
Any ESC that supports DSHOT600

# Building
Firmware can be built using platformio. I haven't tested the build platform on another machine so I can't guarantee it will all work.
