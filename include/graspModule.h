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
    yarp::sig::Matrix K,H;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;

    // Variables for state machine and enabling/disabling options
    bool go_on;
    bool grasp;
    bool executed;
    std::string show_hand;
    std::string show_only_pose;
    std::string look_object;
    bool visualization;
    bool mode_online;    
    bool save_poses;

    double lift_z;
    double torso_pitch_max;
    double traj_time, traj_tol;
    yarp::sig::Vector shift;
    yarp::sig::Vector home_right;
    yarp::sig::Vector home_left;
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

public:
    /************************************************************************/
    bool attach(yarp::os::RpcServer &source);

    /* Return if visualization is on or off */
    /************************************************************************/
    std::string get_visualization();

    /* Set visualization option on or off */
    /************************************************************************/
    bool set_visualization(const std::string &e);

    /* Return the computed grasping poses */
    /************************************************************************/
    yarp::os::Property get_grasping_pose(const yarp::os::Property &superquadric, const std::string &hand);

    /* Get options of the field of interest */
    /************************************************************************/
    yarp::os::Property get_options(const std::string &field);

    /* Set options of the field of interest */
    /************************************************************************/
    bool set_options(const yarp::os::Property &newOptions, const std::string &field);

    /* Return which hand has been enabled */
    /**********************************************************************/
    std::string get_hand();

    /* Set hand enabled */
    /**********************************************************************/
    bool set_hand(const std::string &e);

    /* Return if poses are saved or not */
    /************************************************************************/
    bool set_save_poses(const std::string &entry);

    /* Set if poses are saved or not */
    /************************************************************************/
    std::string get_save_poses();

    /* Fill property with the grasping solutions */
    /************************************************************************/
    yarp::os::Property fillProperty(const yarp::sig::Vector &sol);

    /* Delete computed poses */
    /************************************************************************/
    bool clear_poses();

    /* Move the selected arm */
    /**********************************************************************/
    bool move(const std::string &entry);

    /* Configure basics options */
    /************************************************************************/
    bool configBasics(yarp::os::ResourceFinder &rf);

    /* Close function of the RF module */
    /************************************************************************/
    bool close();

    /* Interrupt function of the RF module */
    /************************************************************************/
    bool interruptModule();

    /* Update function of the RF module */
    /************************************************************************/
    bool updateModule();

    /* Get period function of the RF module */
    /************************************************************************/
    double getPeriod();

    /* Configure options for visualization */
    /***********************************************************************/
    bool configViewer(yarp::os::ResourceFinder &rf);

    /* Configure options for pose computation */
    /***********************************************************************/
    bool configPose(yarp::os::ResourceFinder &rf);

    /* Configure options for arm movements */
    /***********************************************************************/
    bool configMovements(yarp::os::ResourceFinder &rf);

    /* Configure options for grasping */
    /***********************************************************************/
    bool configGrasp(yarp::os::ResourceFinder &rf);

    /* Configure function of RF module */
    /***********************************************************************/
    bool configure(yarp::os::ResourceFinder &rf);

    /* Read object model from text file for simulation tests */
    /************************************************************************/
    bool readSuperq(const std::string &name_obj, yarp::sig::Vector &x, const int &dimension, yarp::os::ResourceFinder *rf);

    /* Save solutions */
    /************************************************************************/
    void saveSol(const yarp::os::Property &sol);

    /* Go back to home position */
    /**********************************************************************/
    bool go_home(const std::string &entry);

};

#endif
