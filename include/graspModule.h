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

#include <opencv2/opencv.hpp>

#include "superquadric.h"
#include "graspComputation.h"
#include "graspVisualization.h"
//#include "graspExecution.h"

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

    int rate, rate_vis;
    double t,t0, t_grasp, t_vis;
    std::deque<double> times_grasp;
    std::deque<double> times_vis;

    double tol;
    int max_iter;
    int n_pointshand;
    int acceptable_iter;
    int object_provided;
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
    yarp::os::BufferedPort<yarp::os::Property > portPose;

    std::string eye;
    yarp::sig::Matrix K,H;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;

    bool go_on;
    bool visualization;
    bool mode_online;    
    bool save_poses;

    std::string chosen_hand;
    std::string nameFileOut_right, nameFileTrajectory_right;
    std::string nameFileOut_left, nameFileTrajectory_left;

    yarp::os::Mutex mutex;

    GraspComputation *graspComp;
    GraspVisualization *graspVis;

    yarp::os::Property pose_par;
    yarp::os::Property traj_par;
    yarp::os::Property ipopt_par;
    yarp::os::Property complete_sol;

public:
    /************************************************************************/
    bool attach(yarp::os::RpcServer &source);

    /************************************************************************/
    std::string get_visualization();

    /************************************************************************/
    bool set_visualization(const std::string &e);

    /************************************************************************/
    yarp::os::Property get_grasping_pose(const yarp::os::Property &superquadric, const std::string &hand);

    /************************************************************************/
    yarp::os::Property get_options(const std::string &field);

    /************************************************************************/
    bool set_options(const yarp::os::Property &newOptions, const std::string &field);

    /************************************************************************/
    bool set_save_poses(const std::string &entry);

    /************************************************************************/
    std::string get_save_poses();

    /************************************************************************/
    yarp::os::Property fillProperty(const yarp::sig::Vector &sol);

    /************************************************************************/
    bool clear_poses();

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
    bool configure(yarp::os::ResourceFinder &rf);

    /************************************************************************/
    bool readSuperq(const std::string &name_obj, yarp::sig::Vector &x, const int &dimension, yarp::os::ResourceFinder *rf);

    /************************************************************************/
    void saveSol(const yarp::os::Property &sol);


};

#endif
