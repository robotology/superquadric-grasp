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

    std::string robot;
    std::string left_or_right;
    std::string homeContextPath;

    yarp::sig::Vector poseL, solL;
    yarp::sig::Vector poseR, solR;
    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;

    int context_gaze;
    int rate_vis;
    int print_level;
    double t,t0, t_grasp, t_vis;
    std::deque<double> times_vis;

    double tol;
    int max_iter;
    int n_pointshand;
    int acceptable_iter;
    double constr_viol_tol;
    std::string mu_strategy;
    std::string nlp_scaling_method;
    double max_cpu_time;
    
    std::string dir;
    yarp::sig::Vector object;
    yarp::sig::Vector hand, hand1;
    double distance, distance1;    
    yarp::sig::Vector displacement;
    yarp::sig::Vector plane;

    yarp::os::RpcServer portRpc;

    std::string eye;
    yarp::sig::Matrix K,H;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;

    std::string fing;
    bool go_on;
    bool grasp;    
    bool executed;
    std::string show_hand;
    std::string show_only_pose;
    std::string look_object;
    bool visualization;
    bool mode_online;    
    bool save_poses;
    bool also_traj;
    std::string visual_servoing;
    std::string use_direct_kin;

    double pixel_tol;
    double lift_z;
    double torso_pitch_max;
    double traj_time, traj_tol;
    yarp::sig::Vector shift_right, shift_left;
    yarp::sig::Vector home_right;
    yarp::sig::Vector home_left;
    std::string hand_to_move;

    std::string nameFileOut_right, nameFileTrajectory_right;
    std::string nameFileOut_left, nameFileTrajectory_left;

    std::string lib_context, lib_filename;

    yarp::os::Mutex mutex;

    GraspComputation *graspComp;
    GraspVisualization *graspVis;
    GraspExecution *graspExec;


    yarp::os::Property vis_par;
    yarp::os::Property pose_par;
    yarp::os::Property traj_par;
    yarp::os::Property grasp_par;
    yarp::os::Property ipopt_par;
    yarp::os::Property movement_par;
    yarp::os::Property complete_sol;

    std::string rotation;

public:
    /************************************************************************/
    bool attach(yarp::os::RpcServer &source);

    /************************************************************************/
    std::string get_visualization();

    /************************************************************************/
    bool set_visualization(const std::string &e);

    /************************************************************************/
    std::string get_best_hand();

    /************************************************************************/
    yarp::os::Property get_grasping_pose(const yarp::os::Property &superquadric, const std::string &hand);

    /************************************************************************/
    yarp::os::Property get_options(const std::string &field);

    /************************************************************************/
    bool set_options(const yarp::os::Property &newOptions, const std::string &field);

    /**********************************************************************/
    std::string get_hand();

    /**********************************************************************/
    bool set_hand(const std::string &e);

    /************************************************************************/
    bool set_save_poses(const std::string &entry);

    /************************************************************************/
    std::string get_save_poses();

    /************************************************************************/
    yarp::os::Property fillProperty(const yarp::sig::Vector &sol);

    /************************************************************************/
    bool clear_poses();

    /**********************************************************************/
    bool move(const std::string &entry);

    /************************************************************************/
    bool configBasics(yarp::os::ResourceFinder &rf);

    /************************************************************************/
    bool close();

    /************************************************************************/
    bool interruptModule();

    /************************************************************************/
    bool updateModule();

    /************************************************************************/
    double getPeriod();

    /***********************************************************************/
    bool configViewer(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configPose(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configMovements(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configGrasp(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configure(yarp::os::ResourceFinder &rf);

    /************************************************************************/
    bool readSuperq(const std::string &name_obj, yarp::sig::Vector &x, const int &dimension, yarp::os::ResourceFinder *rf);

    /************************************************************************/
    void saveSol(const yarp::os::Property &sol);

    /**********************************************************************/
    bool go_home(const std::string &entry);

};

#endif
