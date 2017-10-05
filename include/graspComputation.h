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

    std::string left_or_right;

    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;
    yarp::sig::Vector poseR, solR;
    yarp::sig::Vector poseL, solL;

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

public:
    
    yarp::sig::Vector &hand, &hand1;
    yarp::os::Property &complete_sol;
    const yarp::sig::Vector &object;

    /*******************************************************************************/
    GraspComputation(const yarp::os::Property &_ipopt_par, const yarp::os::Property &_pose_par,
                     const yarp::os::Property &_trajectory_par, const std::string &_left_or_right,
                     yarp::sig::Vector &_hand, yarp::sig::Vector &_hand1, yarp::os::ResourceFinder *_rf,
                     yarp::os::Property &_complete_sol, const yarp::sig::Vector &_object);

    /* Set parameters for computing the solution with ipopt */
    /***********************************************************************/
    void setIpoptPar(const yarp::os::Property &newOptions, bool first_time);

    /* Get parameters used for computing the solution with ipopt */
    /***********************************************************************/
    yarp::os::Property getIpoptPar();

    /* Set parameters for correctly compute the grasping pose */
    /***********************************************************************/
    void setPosePar(const yarp::os::Property &newOptions, bool first_time);

    /* Get parameters for correctly compute the grasping pose */
    /***********************************************************************/
    yarp::os::Property getPosePar();

    /* Set parameters for correctly compute the trajectory for approaching the desired pose */
    /***********************************************************************/
    void setTrajectoryPar(const yarp::os::Property &newOptions, bool first_time);

    /* Get parameters used for correctly compute the trajectory for approaching the desired pose */
    /***********************************************************************/
    yarp::os::Property getTrajectoryPar();

    /* Init function */
    /***********************************************************************/
    bool init();

    /* Run function */
    /***********************************************************************/
    void run();

    /* Compute a given pose for the selected hand */
    /***********************************************************************/
    bool computePose(yarp::sig::Vector &hand, const std::string &left_or_right);

    /* Compute the trajectory for the selected hand */
    /***********************************************************************/
    bool computeTrajectory(const std::string &chosen_hand, const std::string &direction);

    /* Extract the solution from ipopt interface */
    /***********************************************************************/
    void getSolution(const std::string &hand);

    /* Return computation time for getting the pose*/
    /***********************************************************************/
    double getTime();

    /* Properly fill a property with the computed solution*/
    /**********************************************************************/
    yarp::os::Property fillProperty(const std::string &hand);

    /* Set a a parameter equal to a value*/
    /**********************************************************************/
    void setPar(const std::string &tag, const std::string &value);
};

#endif
