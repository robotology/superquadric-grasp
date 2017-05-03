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
class GraspComputation : public yarp::os::RateThread
{
protected:

    std::string left_or_right;

    std::deque<yarp::sig::Vector> trajectory;
    yarp::sig::Vector poseR, solR;
    yarp::sig::Vector poseL, solL;
    yarp::sig::Vector pose_tmp, pose_tmp2;

    double tol, constr_viol_tol;
    int max_iter, acceptable_iter, object_provided;
    std::string mu_strategy,nlp_scaling_method;
    double max_cpu_time;

    yarp::sig::Vector hand, hand1;
    int n_pointshand;
    double distance, distance1;
    std::string dir;
    yarp::sig::Vector displacement;
    yarp::sig::Vector shift;

    yarp::os::Property ipopt_par;
    yarp::os::Property pose_par;
    yarp::os::Property trajectory_par;

    bool go_on;
    bool one_shot;
    bool compute_traj;
    double t0, t_grasp;

    yarp::os::Mutex mutex;

    yarp::os::ResourceFinder *rf;

    yarp::os::BufferedPort<yarp::os::Property> objectPort;

public:

    yarp::sig::Vector object;
    /*******************************************************************************/
    GraspComputation(const int rate, const yarp::os::Property &_ipopt_par, const yarp::os::Property &_pose_par,
                     const yarp::os::Property &_trajectory_par, const std::string &_left_or_right,
                     const yarp::sig::Vector &_hand, const yarp::sig::Vector &_hand1, yarp::os::ResourceFinder *_rf);

    /***********************************************************************/
    void setIpoptPar(const yarp::os::Property &newOptions);

    /***********************************************************************/
    yarp::os::Property getIpoptPar();

    /***********************************************************************/
    void setPosePar(const yarp::os::Property &newOptions);

    /***********************************************************************/
    yarp::os::Property getPosePar();

    /***********************************************************************/
    void setTrajectoryPar(const yarp::os::Property &newOptions);

    /***********************************************************************/
    yarp::os::Property getTrajectoryPar();

    /***********************************************************************/
    virtual bool threadInit();

    /***********************************************************************/
    virtual void run();

    /***********************************************************************/
    virtual void threadRelease();

    /***********************************************************************/
    void getSuperq();

    /***********************************************************************/
    bool computePose(yarp::sig::Vector &hand, const std::string &left_or_right);

    /***********************************************************************/
    bool computeTrajectory(const std::string &chosen_hand, const std::string &direction);

    /***********************************************************************/
    yarp::os::Property getSolution();

    /***********************************************************************/
    double getTime();

    /**********************************************************************/
    yarp::os::Property fillProperty();

};




#endif
