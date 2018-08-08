/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.cub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __EXECUTION_H__
#define __EXECUTION_H__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IVisualServoing.h>

#ifdef USE_FINGERS_POSITION_CONTROL
    #include "FingersPositionControl/HandController.h"
#else
    #include "TactileControl/HandController.h"
#endif

/**
  * This class implements the arm movements for reaching
  * the desired pose for grasping the object and for closing the
  * fingers and stably grasping the object by using tactile feedback.
  */
/*******************************************************************************/
class GraspExecution
{
protected:

    // Parameters for setting up the experiment
    /** Robot name: icub or icubSim**/
    std::string robot;
    /** Hand to be moved for grasping the object**/
    std::string hand_to_move;
    /** Hand to be enabled with the code**/
    std::string left_or_right;

    // Trajectories and home positions
    /** Entire trajectory (final pose and waypoint) for the right hand**/
    std::deque<yarp::sig::Vector> trajectory_right;
    /** Entire trajectory (final pose and waypoint) for the left hand**/
    std::deque<yarp::sig::Vector> trajectory_left;
    /** Entire trajectory selected for the execution**/
    std::deque<yarp::sig::Vector> trajectory;
    /** 3D shift for the final right pose along x, y, and z axes of the right hand reference frame for compensating with eye-kinematics offsets**/
    yarp::sig::Vector shift_right;
    /** 3D shift for the final left pose along x, y, and z axes of the left hand reference frame for compensating with eye-kinematics offsets**/
    yarp::sig::Vector shift_left;
    /** Home pose (7D) for the right hand**/
    yarp::sig::Vector home_right;
    /** Home pose (7D) for the left hand**/
    yarp::sig::Vector home_left;
    /** Basket pose (7D) for the right hand**/
    yarp::sig::Vector basket_right;
    /** Basket pose (7D) for the left hand**/
    yarp::sig::Vector basket_left;
    /** Stiff values for the right hand**/
    yarp::sig::Vector stiff_right;
    /** Stiff values for the left hand**/
    yarp::sig::Vector stiff_left;
    /** Damp values for the right hand**/
    yarp::sig::Vector damp_right;
    /** Damp values for the left hand**/
    yarp::sig::Vector damp_left;


    yarp::os::Mutex mutex;

    // Cartesian
    yarp::dev::ICartesianControl *icart_right;
    yarp::dev::ICartesianControl *icart_left;

    yarp::dev::PolyDriver robotDevice_right;
    yarp::dev::PolyDriver  robotDevice_left;
    yarp::dev::PolyDriver driverImped_right;
    yarp::dev::PolyDriver  driverImped_left;

    // driver and interfaces for controlling the torso
    yarp::dev::PolyDriver             driverTorso;
    yarp::dev::IControlMode          *imodTorso;
    yarp::dev::IEncoders             *iencTorso;
    yarp::dev::IPositionControl      *iposTorso;


    yarp::dev::IEncoders *enc;

    /** Backup context for right cartesian**/
    int context_right;
    /** Backup context for left cartesian**/
    int context_left;

    int i;
    /** Boolean variable to grasp or not the object**/
    bool grasp;
    std::string lobj;
    /** How much the robot should lift the object **/
    double lift_z;
    /** Boolean variable to lift or not the object**/
    bool lift_object;
    /** String variable for the visual servoing controller during the final pose reaching**/
    bool visual_serv;
    /** String variable for enabling the compliant mode**/
    bool compliant;
    /** String variable for using visual servoing with direct kinematics, instead of hand pose estimate**/
    bool use_direct_kin;
    std::string five_fingers;
    /** Maximum pitch value for avoiding movements to close to the table while reaching some particular poses**/
    double torso_pitch_max;
    /** Time for reaching each waypoint of the trajectory**/
    double traj_time;
    /** Reaching tolerance for the cartesian**/
    double traj_tol;

    // Port for reading forces from wholeBodyDynamics
    /** Threshold of the maximum allowed force measured at the endeffector. If the measured value is larger, the movement stops.**/
    double force_threshold;
    /** Port connected to wholeBodyDynamics module in order to receive force estimates**/
    yarp::os::RpcClient portWholeBodyRpc;
    /** Bottle containg force estimate for the right hand**/
    yarp::os::BufferedPort<yarp::os::Bottle> portForces_right;
    /** Bottle containg force estimate for the left hand**/
    yarp::os::BufferedPort<yarp::os::Bottle> portForces_left;

public:

    /** Boolean variable for movements*/
    bool reached;
    /** Boolean variable for movements*/
    bool reached_tot;

    /** Pixel tolerance for visual servoing*/
    double pixel_tol;
    /** Polydriver for visual servoing server*/
    yarp::dev::PolyDriver drv_server_vs;
    /** Interface for visual servoing for right hand*/
    yarp::dev::IVisualServoing *visual_servoing_right;

    /** Context file name for grasping library*/
    std::string lib_context;
    /** Context file name for grasping library*/
    std::string lib_filename;

    /** Property with solution */
    const yarp::os::Property &complete_sol;
    /** Property with options */
    yarp::os::Property movement_par;

    /** Tactile control library for right/left hands */
#ifdef USE_FINGERS_POSITION_CONTROL
    fingersPositionControl::HandController handContr_right;
    fingersPositionControl::HandController handContr_left;
#else
    tactileControl::HandController handContr_right;
    tactileControl::HandController handContr_left;
#endif

    /*******************************************************************************/
    GraspExecution(yarp::os::Property &movement_par, const yarp::os::Property &complete_sol,
                   bool _grasp, std::string _lib_context, std::string _lib_filename);

    /** Configure function
    * @return true/false on success/failure
    */
    /*******************************************************************************/
    bool configure();

    /** Reach a single waypoint of the trajectory
    * @param i is the index of the waypoint
    * @param hand is the name of the selected hand for moving
    * @return true/false on success/failure
    */
    /*******************************************************************************/
    bool reachWaypoint(int i, std::string &hand);

    /** Execute the entire trajectory
    * @ hand is the selected hand for moving
    * @return true/false on success/failure
    */
    /*******************************************************************************/
    bool executeTrajectory(std::string &hand);

    /** Ask the robot arm to go back to home position
     * @param hand is the name of the selected hand
     * @return true/false on success/failure
     */
    /*******************************************************************************/
    bool goHome(const std::string &hand);

    /** Ask the robot arm to go to the basket on its side
     * @param hand is the name of the selected hand
     * @return true/false on success/failure
     */
    /*******************************************************************************/
    bool goToBasket(const std::string &hand);

    /** Configure Cartesian controller for moving the selected arm
    *  @param which_hand is the hand for which we want to open the cartesian
    *  @return true/false on success/failure
    */
    /*******************************************************************************/
    bool configCartesian(const std::string &which_hand);

    /** Configure compliant mode for safe interaction with the table
    *  @param which_hand is the hand for which we want to set the compliant mode
    *  @return true/false on success/failure
    */
    /*******************************************************************************/
    bool configCompliant(const std::string &which_hand);

    /** Configure tactile control library for grasping the object
    *  @return treu/false on success/failure
    */
    /*******************************************************************************/
    bool configGrasp();

    /** Configure torso for returnin it in correct home position
    *  @return treu/false on success/failure
    */
    /*******************************************************************************/
    bool configTorso();

    /** Set all pose parameters for grasp execution
    *  @param newOptions is the property with the new options to be set
    *  @param first_time take into account if the options have been set once or not
    */
    /*******************************************************************************/
    void setPosePar(const yarp::os::Property &newOptions, bool first_time);

    /** Get all pose parameters for grasp execution
    *  @return a Property with all the grasp execution options
    */
    /*******************************************************************************/
    yarp::os::Property getPosePar();

    /** Acquire poses to be reached from the graspComputation class
    * @param poses are the poses computed from graspComputation class
    */
    /*******************************************************************************/
    void getPoses(const yarp::os::Property &poses);

    /** Release resources */
    /*******************************************************************************/
    bool release();

    /** Stop movements */
    /*******************************************************************************/
    bool stop();

    /** Lift object for testing the stability of the pose
    * @param traj is a deque of Vectors including the trajectory waypoint
    */
    /*******************************************************************************/
    void liftObject(std::deque<yarp::sig::Vector> &traj);

    /** Grasp the object with the tactile control library
    * @param hand is the hand selected for moving and grasping
    * @return true when the grasp is completed
    */
    /*******************************************************************************/
    bool graspObject(const std::string &hand);

    /** Open the hand that kis grasping the object with the tactile control library
    *  @param hand is the hand selected for moving and grasping
    *  @return true when the hand is open
    */
    /*******************************************************************************/
    bool releaseObject(const std::string &hand);

    /** Reach a single waypoint of the trajectory with the visual-servoing controller
    * @param i is the index of the waypoint
    * @param hand is the name of the selected hand for moving
    * @return true/false on success/failure
    */
    /*******************************************************************************/
    bool reachWithVisual(int i, std::string &hand);

    /** Configure visual-servoing controller
    * @return true/false on success/failure
    */
    /*******************************************************************************/
    bool configVisualServoing();

    /** Calibrate WholeBodyDynamics before executing trajectory
    * @return true/false on success/failure
    */
    /*******************************************************************************/
    bool calibrateWholeBody();
};

#endif
