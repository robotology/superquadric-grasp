# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift
/**
* Property
*
* IDL structure to set/show advanced parameters.
*/

/**
* superquadricGrasping_IDL
*
* IDL Interface to \ref superquadric-grasping services.
*/

struct Vector
{
} (
   yarp.name = "yarp::sig::Vector"
   yarp.includefile="yarp/sig/Vector.h"
  )

struct Property
{
} (
   yarp.name = "yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
  )

service superquadricGrasping_IDL
{
    /**
    * Start entire pipeline: pose computation and
    * grasping
    *@return true/false on success/failure
    */
    bool start();

    /**
    * Come back home
    *@return true/false on success/failure
    */
    bool go_home(1:string hand);

    /**
    * Say if you want the robot to lift the object or not
    *@param yes or no 
    *@return true/false on success/failure
    */
    bool lift_object(1:string lift_or_not);

    /**
    * Select the kind of grasping you want the robot to perform:
    *@param power or precision, for respectively power or precision grasp
    *@return true/false on success/failure
    */
    bool grasping_method(1:string lift_or_not);

    /**
    * Choose the hand to use to grasp the object
    *@param hand name (left or right)
    *@return true/false on success/failure
    */
    bool choose_hand(1:string hand);

    /**
    * Stop all robot movements but not the module
    *@return true/false on success/failure
    */
    bool stop();

    /**
    * Compute, show and send pose
    *@return computed poses
    */
    Vector compute_pose();

    /** Set parameters of trajectory computation and 
    * poses reaching
    *@params a Property object containing the parameters you want to change
    *@return true/false on success/failure
    */
    bool set_trajectory_options(1:Property options);
}
    
