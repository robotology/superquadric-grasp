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

#include "TactileControl/HandController.h"

/*******************************************************************************/
class GraspExecution
{
protected:

    std::string robot;
    std::string hand_to_move;
    std::string left_or_right;

    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;
    std::deque<yarp::sig::Vector> trajectory;
    yarp::sig::Vector shift;
    yarp::sig::Vector home_right, home_left;

    yarp::os::Mutex mutex;

    yarp::dev::ICartesianControl *icart_right;
    yarp::dev::ICartesianControl *icart_left;

    yarp::dev::PolyDriver robotDevice_right;
    yarp::dev::PolyDriver robotDevice_left;

    yarp::dev::IEncoders *enc;

    int context_right;
    int context_left;

    int i;
    bool grasp;    
    double lift_z;
    double torso_pitch_max;
    double traj_time,traj_tol;

public:

    bool reached;
    bool reached_tot;

    std::string lib_context;
    std::string lib_filename;

    const yarp::os::Property &complete_sol;
    yarp::os::Property movement_par;

    tactileControl:: HandController handContr_right;
    tactileControl:: HandController handContr_left;


    /*******************************************************************************/
    GraspExecution(yarp::os::Property &movement_par, const yarp::os::Property &complete_sol,
                   bool _grasp, std::string _lib_context, std::string _lib_filename);

    /*******************************************************************************/
    bool configure();

    /*******************************************************************************/
    bool reachWaypoint(int i, std::string &hand);

    /*******************************************************************************/
    bool executeTrajectory(std::string &hand);

    /*******************************************************************************/
    bool goHome(const std::string &hand);

    /*******************************************************************************/
    bool configCartesian(const std::string &which_hand);

    /*******************************************************************************/
    bool configGrasp();

    /*******************************************************************************/
    void setPosePar(const yarp::os::Property &newOptions, bool first_time);

    /*******************************************************************************/
    yarp::os::Property getPosePar();

    /*******************************************************************************/
    void getPoses(const yarp::os::Property &poses);

    /*******************************************************************************/
    bool release();

    /*******************************************************************************/
    bool stop();

    /*******************************************************************************/
    void liftObject(std::deque<yarp::sig::Vector> &traj, int index);

    /*******************************************************************************/
    bool graspObject(const std::string &hand);

    /*******************************************************************************/
    bool releaseObject(const std::string &hand);
};

#endif
