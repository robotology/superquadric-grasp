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
struct Property
{
} (
   yarp.name = "yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
  )

/**
* Vector
*
* IDL structure to set/show advanced parameters.
*/
struct Vector
{
} (
   yarp.name = "yarp::sig::Vector"
   yarp.includefile="yarp/sig/Vector.h"
  )

/**
* superquadricGrasp_IDL
*
* IDL Interface to \ref superquadric-grasp services.
*/

service superquadricGrasp_IDL
{
    /**
    * Remove computed poses.
    *@return true/false on success/failure.
    */
    bool clear_poses();

    /**
    * Choose the hand to use to grasp the object.
    *@param hand name (left or right).
    *@return true/false on success/failure.
    */
    bool set_hand(1:string hand);

    /**
    * Get the chosen hand. 
    *@return left or right.
    */
    string get_hand();

    /**
    * Set if to save or not the computed poses 
    * and trajectory.
    *@param entry can be "on" or "off".
    *@return true/false on success/failure.
    */
    bool set_save_poses(1:string entry);

    /**
    * Get if the saving process is on or off.
    *@return "on" or "off".
    */
    string get_save_poses();

    /**
    * Set the  parameters of the module. The user 
    * must pay attention in changing them.
    * @param options is a Property containing the 
    * parameters the user want to change.
    * @param field is a string specifying which
    * can of parameter we are going to change.
    * Field can be: "pose", "trajectory", "optimization",
    * "visualization" or "execution".
    * You can set the  parameters typing, for instance: 
    * command:  set_options ((n_pointshand <points-value>)
    * (hand_displacement_x <displacement-value>)) pose.
    * @return true/false on success/failure.
    */
    bool set_options(1:Property options, 2: string field);


    /**
    * Get the  parameters of the module. The user must
    * pay attention in changing them.
    * @param field can be "pose", "trajectory",
    * "optimization", "statistics", "visualization" or "execution".
    * depending on which parameters we are interested in.
    * @return the Property including all the  parameter values.
    */
    Property get_options(1: string field);

    /** Return the estimated grasping poses given
    * an estimated superquadric.
    *@param estimated_superq is a Property containing
    * the superquadric.
    *@param hand is the hand for which we want
    * to solve the grasping problem (right, left or both).
    *@return a property containing the solution.
    * Note: the estimated superquadric must be 
    * provide in the following format:  (dimensions (x0 x1 x2)) 
    * (exponents (x3 x4)) (center (x5 x6 x7)) (orientation (x8 x9 x10 x11))
    * where x0, x1,x2 are the semi axes of the superquadric,
    * x3, x4 are the responsible for the shape, x5 x6 x7 are the coordinates
    * of the superquadric center and x8 x9 x10 x11
    * are the axis-angle representation of the superquadric orientation.
    * The solution is given in the form: (pose_right (h0 h1 h2 h3 h4 h5 h6))
    * (trajectory_right (t0 t1 t2 t3 t4 t5) ... ) for the right hand,
    * and the same for the left hand (according to the  value of the
    * string hand are input parameter. The quantity "pose_right" is the pose
    * computed for the robot hand (x0,x1,x2,  are the 3D coordinates of the end-effector
    * and x3,x4,x5 are the Euler angles representing the end-effector orientation)
    * The quantity "trajectory_right"  includes all the waypoint of the
    * computed trajectory, in the form center of the end-effector
    * (t0,t1,t2)+ orientation (Euler angles, t3,t4,t5).
    */
    Property get_grasping_pose(1: Property estimated_superq, 2: string hand);

    /**
    * Set if the visualization has to be enabled.
    *@return  true/false on success/failure.
    */
    bool set_visualization(1:string e)

    /**
    * Get if visualization is enabled.
    *@return "on" or "off".
    */
    string get_visualization()

    /**
    * Move the right or the left arm (according to the
    * string e).
    *@return "on" or "off" if e is right or left.
    */
    bool move(1: string e)

    /**
    * Move the right or the left arm  back to home position (according to the
    * string e).
    *@return "on" or "off" if e is right or left.
    */
    bool go_home(1: string e)

    /**
    * Grasp object 
    *@param e is the selected hand
    *@return "on" or "off" if e is right or left.
    */
    bool grasp_object(1: string e)

    /**
    * Release object 
    *@param e is the selected hand
    *@return "on" or "off" if e is right or left.
    */
    bool open(1: string e)

    /**
    * Put hand in relaxed pose
    *@param e is the selected hand
    *@return "on" or "off".
    */
    bool rest(1: string e)

    /**
    * move and wait 
    *@param e is the selected hand
    *@return "true" when finished movement.
    */
    bool move_and_wait(1:string e)

    /**
    * pointing hand pose 
    *@param e is the selected hand
    *@return "true" when finished movement.
    */
    bool pointing(1:string e)

}
    
