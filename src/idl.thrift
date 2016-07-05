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
    * Say if the robot is going to lift the object or not 
    *@return yes or no
    */
    string get_lift_object(); 

    /**
    * Remove computed poses 
    *@return true/false on success/failure
    */
    bool clear_poses();

    /**
    * Select the kind of grasping you want the robot to perform:
    *@param power or precision, for respectively power or precision grasp
    *@return true/false on success/failure
    */
    bool grasping_method(1:string lift_or_not);

    /**
    * Say the kind of selected grasping:
    *@return power or precision
    */
    string get_grasping_method();

    /**
    * Say if enabled depth2kin calibration
    *@return yes or no
    */
    string get_calibrate_cam();

    /**
    * Enable or not depth2kin calibration
    *@param yes or no
    *@return true/false on success/failure
    */
    bool calibrate_cam(1:string calib_or_not);

    /**
    * Choose the hand to use to grasp the object
    *@param hand name (left or right)
    *@return true/false on success/failure
    */
    bool choose_hand(1:string hand);

    /**
    * Get the chosen hand 
    *@return left or right
    */
    string get_chosen_hand();

    /**
    * Change hand displacement for grasping
    *@param displacement value (as a Vector)
    *@return true/false on success/failure
    */
    bool hand_displacement(1:Vector hand);

    /**
    * Get hand displacement for grasping
    *@return the vector of displacement
    */
    list<double> get_hand_displacement();

    /**
    * Stop all robot movements but not the module
    *@return true/false on success/failure
    */
    bool stop();

    /**
    * Compute, show and send pose
    *@return computed poses
    */
    list<double> compute_pose();

    /** Set parameters of trajectory computation and 
    * poses reaching
    *@params a Property object containing the parameters you want to change
    *@return true/false on success/failure
    */
    bool set_trajectory_options(1:Property options);
}
    
