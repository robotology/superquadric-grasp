/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __MODULE_H__
#define __MODULE_H__

#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "superquadric.h"
#include "graspComputation.h"
#include "graspVisualization.h"
#include "graspExecution.h"

#include "src/superquadricGrasp_IDL.h"

/**
  * This class handles the grasping pose computation and
  * visualization, together with the interaction with the user.
  * In particular, it launches the visualization thread and implements a state machine
  * for handling the grasping pose computation and excution.
  */
/*******************************************************************************/
class GraspingModule: public yarp::os::RFModule,
                      public superquadricGrasp_IDL
{
protected:

    yarp::os::ResourceFinder *rf;

    // Strings for setting the experiment scenario
	/** Robot name: icub or icubSim**/
    std::string robot;
	/** Hand to be enabled with the code**/
    std::string left_or_right;
	/** Path where the context is imported**/
    std::string homeContextPath;

    // Pose and trajectory computed for the arms
	/** Robot hand pose computed by the solver for the left hand**/
    yarp::sig::Vector poseL;
	/** Hand ellipsoid pose computed by the solver for the left hand**/
	yarp::sig::Vector solL;
	/** Robot hand pose computed by the solver for the right hand**/
    yarp::sig::Vector poseR;
	/** Hand ellipsoid pose computed by the solver for the right hand**/
	yarp::sig::Vector solR;
	/** Entire trajectory (final pose and waypoint) for the right hand**/
    std::deque<yarp::sig::Vector> trajectory_right;
	/** Entire trajectory (final pose and waypoint) for the left hand**/
    std::deque<yarp::sig::Vector> trajectory_left;

	/** Variable for saving gaze context**/
    int context_gaze;
	/** Rate of the visualization thread **/
    int rate_vis;
	/** Print level for the Ipopt optimization problem**/
    int print_level;
    double t,t0, t_grasp, t_vis;
    std::deque<double> times_vis;

    // Optimization parameters
	/** Tolerance of the Ipopt optimization problem**/
    double tol;
	/** Maximum iteration allowed for the Ipopt optimization problem**/
    int max_iter;
	/** Number of points sampled on the hand ellipsoid for the Ipopt optimization problem**/
    int n_pointshand;
	/** Acceptable iter of the Ipopt optimization problem**/
    int acceptable_iter;
	/** Constraint tolerance of the Ipopt optimization problem**/
    double constr_viol_tol;
	/** Mu strategy of the Ipopt optimization problem**/
    std::string mu_strategy;
	/** NLP scaling method of the Ipopt optimization problem**/
    std::string nlp_scaling_method;
	/** Max cpu time allowed for the Ipopt optimization problem**/
    double max_cpu_time;

    // Geometric parameters for pose computation
	/** Direction for generating the waypoint for the approach: it could be on x and z axes ("xz") or only z axis ("z") of the hand reference frame.**/
    std::string dir;
	/** Object superquadric **/
    yarp::sig::Vector object;
	/** Hand ellipsoid for the first hand enabled**/
    yarp::sig::Vector hand;
	/** Hand ellipsoid for the second hand enabled**/
	yarp::sig::Vector hand1;
	/** Distance for shifting the waypoint along x axis of the hand reference frame**/
    double distance;
	/** Distance for shifting the waypoint along z axis of the hand reference frame**/
	double distance1;
	/** Distance of the robot pose with respect to the hand ellipsoid along x axis of the hand reference frame**/
    yarp::sig::Vector displacement;
	/** Parameters of the implicit function describing the plane on which the object is located in the root reference frame**/
    yarp::sig::Vector plane;

	/** Rpc port for services**/
    yarp::os::RpcServer portRpc;

    // Gaze parameters
	/** Eye camera selected for visualization**/
    std::string eye;
	/** Vergence value for blocking the eyes**/
    double block_eye;
	/** Neck joint value for looking to the table**/
    double block_neck;
    yarp::sig::Matrix K,H;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;

    // Variables for state machine and enabling/disabling options
    std::string fing;
	/** Boolean variable to lift or not the object**/
    bool lift_object;
    std::string lobj;
	/** Boolean variable used for going to the next step of the state machine**/
    bool go_on;
	/** Boolean variable to grasp or not the object**/
    bool grasp;
	/** Boolean variable taking into account if the movement has been executed**/
    bool executed;
	/** Boolean variable taking into account if the movement has been executed (for different checks)**/
    bool executed_var;
	/** Boolean variable taking into account if the home pose has been reached**/
    bool reached_home;
	/** Boolean variable taking into account if the basket pose has been reached**/
    bool reached_basket;
	/** String variable for showing (on) or not (off) the hand ellipsoid on the viewer**/
    std::string show_hand;
	/** String variable for showing only the final pose(on) or also the trajectory (off) on the viewer**/
    std::string show_only_pose;
	/** String variable for fixating at the object (on) or not (off) during movements**/
    std::string look_object;
	/** Boolean variable for enabling visualization**/
    bool visualization;
    /** Boolean variable for enabling movements**/
    bool demo;
	/** Boolean variable for switching between online and offline mode**/
    bool mode_online;
	/** String variable for saving the solutions**/
    bool save_poses;
    bool also_traj;
	/** Threshold of the maximum allowed force measured at the endeffector. If the measured value is larger, the movement stops.**/
    double force_threshold;
	/** String variable for enabling the compliant mode**/
    std::string compliant;
	/** String variable for the visual servoing controller during the final pose reaching**/
    std::string visual_servoing;
	/** String variable for using visual servoing with direct kinematics, instead of hand pose estimate**/
    std::string use_direct_kin;

	/** Pixel tolerance for visual servoing**/
    double pixel_tol;
	/** How much the robot should lift the object **/
    double lift_z;
	/** Maximum pitch value for avoiding movements to close to the table while reaching some particular poses**/
    double torso_pitch_max;
	/** Time for reaching each waypoint of the trajectory**/
    double traj_time;
	/** Reaching tolerance for the cartesian**/
	double traj_tol;
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
	/** Hand selected for grasping the object**/
    std::string hand_to_move;

    // Information for saving results
	/** File name of Ipopt output for right hand**/
    std::string nameFileOut_right;
	/** File name containing trajectory for right hand**/
	std::string nameFileTrajectory_right;
	/** File name of Ipopt output for left hand**/
    std::string nameFileOut_left;
	/** File name containing trajectory for left hand**/
	std::string nameFileTrajectory_left;

	/** Context name for the module for closing the fingers**/
    std::string lib_context;
	/** File name of the module used for closing the fingers**/
	std::string lib_filename;

    yarp::os::Mutex mutex;

    // Different threads
	/** Class computing the grasping pose**/
    GraspComputation *graspComp;
	/** Class showing the grasping pose on the viewer**/
    GraspVisualization *graspVis;
	/** Class executing the trajectory**/
    GraspExecution *graspExec;

    // Properties with all the class parameters
	/** Parameters of the visualization class**/
    yarp::os::Property vis_par;
	/** Parameters for pose computation**/
    yarp::os::Property pose_par;
	/** Parameters for trajectory computation**/
    yarp::os::Property traj_par;
	/** Parameters for grasping the object**/
    yarp::os::Property grasp_par;
	/** Parameters for the Ipopt optimization problem**/
    yarp::os::Property ipopt_par;
	/** Parameters for executing the movement**/
    yarp::os::Property movement_par;
	/** Complete solution computed**/
    yarp::os::Property complete_sol;

	/** Quality of pose right**/
    double quality_right;
	/** Quality of pose left**/
	double quality_left;

public:
    /************************************************************************/
    bool attach(yarp::os::RpcServer &source);

    /** Return if visualization is on or off
    * @return "on" or "off"
    */
    /************************************************************************/
    std::string get_visualization();

    /** Set visualization option on or off
    * @param e can be "on" or "off" for setting the visualization on or off
    * @return true/false on success/failure
    */
    /************************************************************************/
    bool set_visualization(const std::string &e);

    /** Return the computed grasping poses
    * @param superquadric is a property with the object superquadric
    * @param hand is the name of the hand for which we want to compute the grasping pose
    * @return a property with the grasping pose solution
    */
    /************************************************************************/
    std::string get_best_hand();

    /************************************************************************/
    yarp::os::Property get_grasping_pose(const yarp::os::Property &superquadric, const std::string &hand);

    /** Get options of the field of interest
    * @param is a string with the field of options of interest
    * @return a Property with all the options
    */
    /************************************************************************/
    yarp::os::Property get_options(const std::string &field);

    /** Set options of the field of interest
    * @param newOptions is a proeprty with the new options to be set
    * @param field is the field of the options to be set
    * @return true/false on success/failure
    */
    /************************************************************************/
    bool set_options(const yarp::os::Property &newOptions, const std::string &field);

    /** Return which hand has been enabled
    * @return the name of the hand enabled
    */
    /**********************************************************************/
    std::string get_hand();

    /** Set hand enabled
    * @param e is the name of the selected hand
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool set_hand(const std::string &e);

    /** Return if poses are saved or not
    * @param entry can be "on" or "off"
    * @return true of false
    */
    /************************************************************************/
    bool set_save_poses(const std::string &entry);

    /** Set if poses are saved or not
    * @return "on" or "off" is the poses are saved
    */
    /************************************************************************/
    std::string get_save_poses();

    /** Fill property with the grasping solutions
    * @param sol is the solution to be saved in a property
    * @return the Property with the solution in the proper form
    */
    /************************************************************************/
    yarp::os::Property fillProperty(const yarp::sig::Vector &sol);

    /** Delete computed poses
    * @return true/false
    */
    /************************************************************************/
    bool clear_poses();

    /** Move the selected arm
    * @param entry is the name of the hand to be moved
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool move(const std::string &entry);

    /** Configure basics options
    * @param rf is the resource finder with all the options from config files
    * @return true/false on success/failure
    */
    /************************************************************************/
    bool configBasics(yarp::os::ResourceFinder &rf);

    /** Close function of the RF module
    * @return true/false on success/failure
    */
    /************************************************************************/
    bool close();

    /** Interrupt function of the RF module
    * @return true/false on success/failure
    */
    /************************************************************************/
    bool interruptModule();

    /** Update function of the RF module
    * @return true/false on success/failure
    */
    /************************************************************************/
    bool updateModule();

    /** Get period function of the RF module
    * @return the thread period
    */
    /************************************************************************/
    double getPeriod();

    /** Configure options for visualization
    * @param rf is the resource finder with all the optins
    * @return true/false on success/failure
    */
    /***********************************************************************/
    bool configViewer(yarp::os::ResourceFinder &rf);

    /** Configure options for pose computation
    * @param rf is the resource finder with all the optins
    * @return true/false on success/failure
    */
    /***********************************************************************/
    bool configPose(yarp::os::ResourceFinder &rf);

    /** Configure options for arm movements
    * @param rf is the resource finder with all the optins
    * @return true/false on success/failure
    */
    /***********************************************************************/
    bool configMovements(yarp::os::ResourceFinder &rf);

    /** Configure options for grasping
    * @param rf is the resource finder with all the optins
    * @return true/false on success/failure
    */
    /***********************************************************************/
    bool configGrasp(yarp::os::ResourceFinder &rf);

    /** Configure function of RF module */
    /***********************************************************************/
    bool configure(yarp::os::ResourceFinder &rf);

    /** Read object model from text file for simulation tests
    * @param name_obj is the name of the object
    * @param x is the superquadric lot be read
    * @param dimensions is the dimension of the superquadric (11)
    * @param rf is the resource finder
    * @return true/false on success/failure
    */
    /************************************************************************/
    bool readSuperq(const std::string &name_obj, yarp::sig::Vector &x, const int &dimension, yarp::os::ResourceFinder *rf);

    /** Save solutions
    * @param sol is a property with the solution to be saved
    */
    /************************************************************************/
    void saveSol(const yarp::os::Property &sol);

    /* Ask the robot to look in the center
    * @return true/false on success/failure
`   */
    /**********************************************************************/
    bool look_center();

    /* Ask the robot to look the object again
    * @return true/false on success/failure
`   */
    /**********************************************************************/
    bool look_obj();

    /** Go back to home position
    * @param entry is the name of the hand that is moving
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool go_home(const std::string &entry);

    /** Go back to the basket on the robot side
    * @param entry is the name of the hand that is moving
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool go_to_basket(const std::string &entry);

    /**
    * Check if the motion has been completed
    *@return true/false on success/failure
    */
    /**********************************************************************/
    bool check_motion();

    /**
    * Check if the motion back to home has been completed
    *@return true/false on success/failure
    */
    /**********************************************************************/
    bool check_home();

    /**
    * Check if the motion to the basket has been completed
    *@return true/false on success/failure
    */
    /**********************************************************************/
    bool check_basket();


};

#endif
