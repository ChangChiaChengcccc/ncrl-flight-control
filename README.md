# NCRL Flight Controller

## Guide

Check NCRL Flight Controller [documentation page](https://c-shengwen-tw.gitbook.io/ncrl-flight-controller/).

## License

The source code of **ncrl-flight-control** is released under GPLv3 license, for commercial inquiries please contact Teng-Hu Cheng (tenghu@g2.nctu.edu.tw).

# Version(developed by ChiaCheng)
---new---
NCRL Flight Controller was revised to fit a single quadrotor with fault motors.
1. the motors can mimic the fault motors by function: fake_mr_geometry_ctrl_thrust_allocation at 
ncrl-flight-control/src/core/controllers/multirotor_geometry/multirotor_geometry_ctrl.c
2. get the estimated motors efficiency from upboard and reallocate the motors thrust at
ncrl-flight-control/src/core/controllers/multirotor_geometry/multirotor_geometry_ctrl.c
3. new debug link function for check motors efficiency.
  
