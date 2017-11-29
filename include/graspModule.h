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
    std::string robot;
    std::string left_or_right;
    std::string homeContextPath;

    // Pose and trajectory computed for the arms
    yarp::sig::Vector poseL, solL;
    yarp::sig::Vector poseR, solR;
    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;

    int context_gaze;
    int rate_vis;
    int print_level;
    double t,t0, t_grasp, t_vis;
    std::deque<double> times_vis;

    // Optimization parameters
    double tol;
    int max_iter;
    int n_pointshand;
    int acceptable_iter;
    double constr_viol_tol;
    std::string mu_strategy;
    std::string nlp_scaling_method;
    double max_cpu_time;

    // Geometric parameters for pose computation
    std::string dir;
    yarp::sig::Vector object;
    yarp::sig::Vector hand, hand1;
    double distance, distance1;
    yarp::sig::Vector displacement;
    yarp::sig::Vector plane;

    yarp::os::RpcServer portRpc;

    // Gaze parameters
    std::string eye;
    double block_eye;
    double block_neck;
    yarp::sig::Matrix K,H;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;

    // Variables for state machine and enabling/disabling options
    std::string fing;
    bool lift_object;
    std::string lobj;
    bool go_on;
    bool grasp;
    bool executed;
    bool executed_var;
    bool reached_home;
    bool reached_basket;
    std::string show_hand;
    std::string show_only_pose;
    std::string look_object;
    bool visualization;
    bool mode_online;
    bool save_poses;
    bool also_traj;
    double force_threshold;
    std::string compliant;
    std::string visual_servoing;
    std::string use_direct_kin;

    double pixel_tol;
    double lift_z;
    double torso_pitch_max;
    double traj_time, traj_tol;
    yarp::sig::Vector shift_right, shift_left;
    yarp::sig::Vector home_right;
    yarp::sig::Vector home_left;
    yarp::sig::Vector basket_right;
    yarp::sig::Vector basket_left;
    yarp::sig::Vector stiff_right;
    yarp::sig::Vector stiff_left;
    yarp::sig::Vector damp_right;
    yarp::sig::Vector damp_left;
    std::string hand_to_move;

    // Information for saving results
    std::string nameFileOut_right, nameFileTrajectory_right;
    std::string nameFileOut_left, nameFileTrajectory_left;

    std::string lib_context, lib_filename;

    yarp::os::Mutex mutex;

    // Different threads
    GraspComputation *graspComp;
    GraspVisualization *graspVis;
    GraspExecution *graspExec;

    // Properties with all the class parameters
    yarp::os::Property vis_par;
    yarp::os::Property pose_par;
    yarp::os::Property traj_par;
    yarp::os::Property grasp_par;
    yarp::os::Property ipopt_par;
    yarp::os::Property movement_par;
    yarp::os::Property complete_sol;

    double quality_right, quality_left;

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
