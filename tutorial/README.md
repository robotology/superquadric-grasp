# superquadric-grasp tutorial
#### Overview

- [ How to compile](#how-to-compile)
- [ How to communicate with the module](#how-to-communicate-with-the-superquadric-grasp)
- [ How to run the `superquadric-grasp` + `tutorial`](#how-to-run-the-superquadric-grasp--tutorial)
- [Setting up before running on the robot](#setting-up-before-running-on-the-robot)
- [How to run the overall pipeline](#how-to-run-the-overall-pipeline)

## How to compile
```sh
mkdir build; cd build
ccmake ..
make install
```

[`Go to the top`](#superquadric-grasp-tutorial)
## How to communicate with the superquadric-grasp 
Once the user has a superquadric modeling the object as a `yarp::Vector sol` (for example, by querying the [superquadric-model](https://github.com/robotology/superquadric-model)), he can query the superquadric-grasp for
getting a suitable grasping pose for the hand `std::string hand` through the `thrift services`:
```cpp
graspRpc.open("/testing-graspmodule/superq:rpc");

Bottle cmd, reply;
cmd.addString("get_grasping_pose");

Bottle &b1=cmd.addList();
Bottle &b2=b1.addList();
b2.addString("dimensions");
Bottle &b3=b2.addList();
b3.addDouble(sol[0]); b3.addDouble(sol[1]); b3.addDouble(sol[2]);

Bottle &b5=b1.addList();
b5.addString("exponents");
Bottle &b6=b5.addList();
b6.addDouble(sol[3]); b6.addDouble(sol[4]);

Bottle &b8=b1.addList();
b8.addString("center");
Bottle &b9=b8.addList();
b9.addDouble(sol[5]); b9.addDouble(sol[6]); b9.addDouble(sol[7]);

Bottle &b11=b1.addList();
b11.addString("orientation");
Bottle &b12=b11.addList();
Vector orient=dcm2axis(euler2dcm(sol.subVector(8,10)));
b12.addDouble(orient[0]); b12.addDouble(orient[1]); b12.addDouble(orient[2]); b12.addDouble(orient[3]);

cmd.addString(hand);

graspRpc.write(cmd, reply);
```
connecting to the corresponding port:
```sh
yarp connect /testing-graspmodule/superq:rpc /superquadric-grasp/rpc
```
For instance, in case `hand=right` solution of the grasping pose computation will be received in the format:
```
(solution_right (s0 s1 s2 s3 s4 s5)) (pose_right (p0 p1 p2 p3 p4 p5 p6)) 
```
where `solution` is a 6D vector representing the hand ellipsoid pose (with Euler angles) and `pose` is a 7D vector representing the robot hand pose
for grasping the object (with axis-angle). 
The variable `hand` can be set equal to `both`, in case we want the solutions for both the arms.

It's possible also to ask the module to compute an `approaching trajectory`, consisting of one waypoint pose, obtained by shifting the final
pose along the `x` and `z` axes of the robot hand frame.
In this case, the solution will also include the quantity:
```
(trajectory_right ((t0 t1 t2 t3 t4 t5 t6) (t01 t11 t12 t13 t14 t15 t16 t17))
```

If the user wants the robot to grasp the object with one of the hands, the user should type:
```cpp
Bottle cmd, reply;

cmd.addString("move");
cmd.addString(hand_to_move);

graspRpc.write(cmd, reply);
```
where `hand_to_move` can be `left` or `right`.

[`Go to the top`](#superquadric-grasp-tutorial)
## How to run the `superquadric-grasp` + `tutorial`
This tutorial is designed to be tested on the `simulator`, since a fake superquadric is used (the tutorial upload an example of superquadric from a configuration file).
In order to run the tutorial:
1. Launch a yarpserver on your computer:
```sh
yarpserver --write
```
2. Launch the [`iCub_SIM`](http://wiki.icub.org/wiki/Simulator_README).
3. Launch the basic modules  for the simulator (`yarprobotinterface`, `iKinGazeCtrl`, `iKinCartsianSolver`- for both right and left arm-).
4. Launch the `superquadric-grasp` and a `yarpviewer`, using [this xml](https://github.com/robotology/superquadric-grasp/blob/master/app/scripts/superquadric-grasp.xml.template).
**Note** The _Tactile Control Library_ cannot be executed on the simulator, since it communicates with low level ports of the robot. For this reason, pay attention to specify `grasp   off` in the [configuration file](https://github.com/robotology/superquadric-grasp/blob/master/app/conf/config.ini#L30). In this scenario, the robot will only reach the pose without closing the fingers.
5. Launch the tutorial with [this xml](https://github.com/robotology/superquadric-grasp/blob/master/tutorial/app/script/testing-graspmodule.xml.template).
6. Connect everything.

The simulator will perform a fake grasping pose computation and execution.


Enjoy! :smiley:

[`Go to the top`](#superquadric-grasp-tutorial)
## Setting up before running on the robot

In order to achieve the desired performance when running the module on the robot, it is strongly recommended to perform the following steps:
#### Calibrate the robot following the `fine calibration` procedure:
- for the [arms](http://wiki.icub.org/wiki/ArmFineCalibration);
- for the [head](http://wiki.icub.org/wiki/HeadFineCalibration);
- for the [torso](http://wiki.icub.org/wiki/TorsoFineCalibration).

#### Check the hand-eye calibration 
Our approach is an open loop approach. For this reason, the grasping goal can be properly reached if the stereo-vision system (see [here](https://github.com/robotology/superquadric-model/tree/master/tutorial#calibrate-the-stereo-vision-through-the-sfm-module) for information on how to calibrate it) and robot kinematics are properly calibrated (previous section). If the offsets between the two hinder the robot from correctly reaching for the desired pose even after the calibration, the superquadric-grasp module allows the definition of some empirical offsets, in order to compensate for that: `shift_right` and `shift_left`. They are 3D Vectors representing the required offsets for correctly reaching the pose along the *x,y,z* axes of the robot reference frame.

In order to overcome the need for a fine hand-eye calibration, we are currently integrating our work with the visual servoing approach described [here](https://github.com/robotology/visual-tracking-control).

[`Go to the top`](#superquadric-grasp-tutorial)

## How to run the overall pipeline
This module is designed in order to be executed together the [`superquadric-model module`](https://github.com/robotology/superquadric-model). A **complete tutorial** on how to execute them together and perform on the robot a **complete modeling and grasping task** is provided in [`this repository`](https://github.com/robotology/superquadric-grasp-example).

[`Go to the top`](#superquadric-grasp-tutorial)
