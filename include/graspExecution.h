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

    yarp::dev::IEncoders *ienc_right;
    yarp::dev::IControlMode2 *imod_right;
    yarp::dev::IPositionControl2 *ipos_right;
    yarp::dev::IEncoders *ienc_left;
    yarp::dev::IControlMode2 *imod_left;
    yarp::dev::IPositionControl2 *ipos_left;

    yarp::dev::PolyDriver driver_right;
    yarp::dev::PolyDriver driver_left;

    int i;
    bool grasp;    
    double lift_z;

    double angle_paddle;
    double angle_thumb;

public:

    bool reached;
    bool reached_tot;

    const yarp::os::Property &complete_sol;
    yarp::os::Property movement_par;

    yarp::os::RpcClient reachRightPort;
    yarp::os:: RpcClient reachLeftPort;
    yarp::os::BufferedPort<yarp::sig::Vector> stateRightPort;
    yarp::os::BufferedPort<yarp::sig::Vector> stateLeftPort;

    /*******************************************************************************/
    GraspExecution(yarp::os::Property &movement_par, const yarp::os::Property &complete_sol, bool _grasp);

    /*******************************************************************************/
    bool configure();

    /*******************************************************************************/
    bool reachWaypoint(int i, std::string &hand);

    /*******************************************************************************/
    bool executeTrajectory(std::string &hand);

    /*******************************************************************************/
    bool goHome(const std::string &hand);

    /*******************************************************************************/
    bool configController(const std::string &which_hand);

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

    /*******************************************************************************/
    bool reachWithVisual(int i, std::string &hand);

    /*******************************************************************************/
    bool configVisualServoing();
};

#endif
