# superquadric-grasping
Framework for computing a good grasping pose, by exploiting superquadric models for the object and the robot hand.

## Module description

##Description
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [IPOPT](https://projects.coin-or.org/Ipopt)
- [OpenCV](http://opencv.org/)
- [IOL](https://github.com/robotology/iol)
- [superquadric-model](https://github.com/robotology/superquadric-model)

##Module pipeline
The module computes a reachable pose for the robot and say to the robot to grasp and lift the object.
The complete pipeline is the following:

1. Given the <b>object model</b>, as a _superquadric_, and the <b>hand</b>, as an _ellipsoid_, that we want to use, a reachable pose for the selected hand is computed. 
    <b>NB</b>: 
    - The  object model can be provided from a _configuration file_ or by quering the [superquadric-model](https://github.com/robotology/superquadric-model), that computes _online_ a superquadric fitting the object of interest point cloud. 
    - The user can select not only _one hand_, but also _both_ of them. In this case, two poses will be computed, one for the left and one for the right.
2. The computed pose/<b>poses</b> is/are <b>shown<\b> together with the superquadric representing the volume graspable by the hand, overlapped to the superquadric modelling the object.
3. Then the user can choose if to move the robot and, if both the poses are computed, with hand the robot has to use.
4. The <b>trajectory</b> is shown and the robot <b>reach</b> the desired pose.
5. The robot <b>grasp</b> and <b>lift</b> the object.
6. Finally the hand goes back to the initial pose.

This pipeline can be modified by the user by specifying if he wants the robot to grasp and lift the object, or only to compute the pose, .. and so on.

## License
Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_
and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.
