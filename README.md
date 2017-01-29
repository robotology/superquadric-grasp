# superquadric-grasping
Framework for computing a good grasping pose, by exploiting superquadric models for the object and the robot hand.

## Theoretical background
The theoretical background will be available soon in the following paper:

G. Vezzani, U. Pattacini and L. Natale, "A grasping approach based on superquadric models", _accepted at IEEE ICRA 2016_, Sngapore.

## Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [IPOPT](https://projects.coin-or.org/Ipopt)
- [OpenCV](http://opencv.org/)
- [IOL](https://github.com/robotology/iol)
- [superquadric-model](https://github.com/robotology/superquadric-model)

## How to compile
In `Linux systems` code can be compiled as follows:
```
git clone https://github.com/giuliavezzani/superquadric-grasping.git
cd superquadric-grasping
mkdir build; cd build
ccmake ..
make install
```

##Module pipeline
The module computes a <b> pose reachable</b> by the robot hand and  make it <b>grasp and lift the object</b>.
The complete pipeline is the following:

1. Given the <b>object model</b>, as a _superquadric_, and the <b> selected hand</b>, whose **graspable volume** is represented by an _ellipsoid_, a reachable pose  is computed. 
    - The  **object model** can be provided from a _configuration file_ or by quering the [superquadric-model](https://github.com/robotology/superquadric-model), that computes _online_ a superquadric fitting the object of interest point cloud. 
    - The user can select not only one hand, but also **both**. In this case, two poses will be computed, one for the left and one for the right hand.
    
2. The computed pose/<b>poses</b> is/are <b>shown</b> together with the superquadric representing the volume graspable by the hand, overlapped to the superquadric modelling the object.
3. Then the user can  **ask the robot to grasp and lift the object**. If the poses for both of the arms are computed, the user can choose the best one.
4. The <b>trajectory</b> is shown and the robot <b>reaches</b> the desired pose.
5. The robot <b>grasps</b> and <b>lifts</b> the object.
6. Finally the hand goes back to the initial pose.

The described pipelin can be summarized by the following image:

<img src="https://github.com/giuliavezzani/superquadric-grasping/blob/master/misc/superquadric-grasping.png">

## Documentation
Online code documentation will be available soon.

## License
Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_
and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.

[![DOI](https://zenodo.org/badge/54572419.svg)](https://zenodo.org/badge/latestdoi/54572419)



