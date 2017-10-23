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

#ifndef __COMPUTATION_H__
#define __COMPUTATION_H__

#include <yarp/dev/all.h>

#include "superquadric.h"

/*******************************************************************************/
class GraspComputation
{
protected:

    std::string left_right;

    double tol, constr_viol_tol;
    int max_iter, acceptable_iter, object_provided;
    std::string mu_strategy,nlp_scaling_method;
    double max_cpu_time;

    int n_pointshand;
    double distance, distance1;
    std::string dir;
    yarp::sig::Vector displacement;
    yarp::sig::Vector plane;

    yarp::os::Property ipopt_par;
    yarp::os::Property pose_par;
    yarp::os::Property trajectory_par;

    bool go_on;
    double t0, t_grasp;

    yarp::os::Mutex mutex;

    yarp::os::ResourceFinder *rf;

    int print_level;

public:
    
    yarp::sig::Vector &hand, &hand1;
    yarp::os::Property &complete_sol;
    const yarp::sig::Vector &object;
    const yarp::sig::Vector &obstacle;
    int count_file_old;
    int count_file;
    bool multiple_superq;
    std::string best_hand;
    double final_value_R, final_value_L, cos_zr, cos_zl;

    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;
    yarp::sig::Vector poseR, solR;
    yarp::sig::Vector poseL, solL;

    double &quality_right, &quality_left;

    /*******************************************************************************/
    GraspComputation(const yarp::os::Property &_ipopt_par, const yarp::os::Property &_pose_par,
                     const yarp::os::Property &_trajectory_par, const std::string &_left_or_right,
                     yarp::sig::Vector &_hand, yarp::sig::Vector &_hand1, yarp::os::ResourceFinder *_rf,
                     yarp::os::Property &_complete_sol, const yarp::sig::Vector &_object, const yarp::sig::Vector &_obstacle, double &_quality_right, double &_quality_left, bool &_multiple_superq);

    /***********************************************************************/
    void setIpoptPar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    yarp::os::Property getIpoptPar();

    /***********************************************************************/
    void setPosePar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    yarp::os::Property getPosePar();

    /***********************************************************************/
    void setTrajectoryPar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    yarp::os::Property getTrajectoryPar();

    /***********************************************************************/
    bool init();

    /***********************************************************************/
    void run();

    /***********************************************************************/
    bool computePose(yarp::sig::Vector &hand, const std::string &left_or_right);

    /***********************************************************************/
    bool computeTrajectory(const std::string &chosen_hand, const std::string &direction);

    /***********************************************************************/
    void getSolution(const std::string &hand);

    /***********************************************************************/
    double getTime();

    /**********************************************************************/
    yarp::os::Property fillProperty(const std::string &hand);

    /**********************************************************************/
    void setPar(const std::string &tag, const std::string &value);

    /***********************************************************************/
    void bestPose();
};

#endif
