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

#include <iCub/action/actionPrimitives.h>

#include "superquadric.h"
//#include "graspComputation.h"
//#include "graspVisualization.h"
//#include "graspExecution.h"

#include "src/superquadricGrasp_IDL.h"


#define AFFACTIONPRIMITIVESLAYER    iCub::action::ActionPrimitivesLayer1

/*******************************************************************************/
class GraspingModule: public yarp::os::RFModule,
                      public superquadricGrasp_IDL
{
protected:

    AFFACTIONPRIMITIVESLAYER *action;
    AFFACTIONPRIMITIVESLAYER *action2;
    yarp::os::RpcServer                 portRpc;

    yarp::sig::Vector graspOrienR;
    yarp::sig::Vector graspDispR;
    yarp::sig::Vector dOffsR;
    yarp::sig::Vector dLiftR;
    yarp::sig::Vector home_xR;

    yarp::sig::Vector graspOrienL;
    yarp::sig::Vector graspDispL;
    yarp::sig::Vector dOffsL;
    yarp::sig::Vector dLiftL;
    yarp::sig::Vector home_xL;

    bool firstRun;

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::PolyDriver robotDevice2;
    yarp::dev::PolyDriver robotDevice3;
    yarp::dev::PolyDriver robotDevice4;

    yarp::dev::ICartesianControl *icart_arm;
    yarp::dev::ICartesianControl *icart_arm2;
    yarp::dev::IEncoders *enc;

    std::string robot;
    std::string left_or_right;

    yarp::os::ResourceFinder *rf;

    std::deque<yarp::sig::Vector> trajectory;
    yarp::sig::Vector poseR, solR;
    yarp::sig::Vector poseL, solL;
    yarp::sig::Vector pose_tmp, pose_tmp2;
    double t,t0;

    double tol, constr_viol_tol;
    int max_iter, acceptable_iter, object_provided;
    std::string mu_strategy,nlp_scaling_method;

    yarp::sig::Vector object;
    yarp::sig::Vector hand, hand1;
    int n_pointshand;
    double distance, distance1;
    std::string dir;
    yarp::sig::Vector displacement;
    yarp::sig::Vector shift;

    yarp::os::RpcClient portSuperqRpc;
    yarp::os::RpcClient portCalibCamRpc;
    std::string superq_name;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgOut;
    std::string eye;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;
    yarp::sig::Matrix K,H;

    bool go_on;
    bool online;
    bool move;
    bool go_move;
    bool viewer;
    bool stop_var;
    bool reached_pose;
    bool grasped_object;
    bool lifted_object;
    bool came_back;
    bool conf_dev_called;
    bool chosen_pose;
    bool conf_act_called;
    bool calib_cam;
    bool lift;

    std::string nameFileOut_right, nameFileSolution_right, nameFileTrajectory;
    std::string nameFileOut_left, nameFileSolution_left;
    std::string chosen_hand;

    yarp::os::Mutex mutex;

public:
    /************************************************************************/
    bool attach(yarp::os::RpcServer &source);

    /************************************************************************/
    bool set_tag_file(const std::string &tag_file);

    /************************************************************************/
    std::string get_tag_file();

    /**********************************************************************/
    std::string get_visualization();

    /**********************************************************************/
    bool set_visualization(const std::string &e);

    /************************************************************************/
    std::string get_movement();

    /************************************************************************/
    bool set_movement(const std::string &entry);

    /************************************************************************/
    bool start();

    /**********************************************************************/
    yarp::os::Property get_grasping_pose(const yarp::os::Property &superquadric);

    /**********************************************************************/
    yarp::os::Property fillProperty(const yarp::sig::Vector &sol);

    /************************************************************************/
    bool stop();

    /************************************************************************/
    bool go_home(const std::string &hand);

    /************************************************************************/
    bool clear_poses();

    /************************************************************************/
    bool set_hand(const std::string &str_hand);

    /************************************************************************/
    std::string get_hand();

    /**********************************************************************/
    yarp::os::Property get_options(const std::string &field);

    /**********************************************************************/
    bool set_options(const yarp::os::Property &newOptions, const std::string &field);

    /***********************************************************************/
    bool set_save_pose(const std::string &entry);

    /***********************************************************************/
    std::string get_save_pose();

    /************************************************************************/
    GraspingModule();

    /************************************************************************/
    void getArmDependentOptions(yarp::os::Bottle &b, yarp::sig::Vector &_gOrien, yarp::sig::Vector &_gDisp,
                                yarp::sig::Vector &_dOffs, yarp::sig::Vector &_dLift, yarp::sig::Vector &_home_x);

    /****************************************************************/
    bool configBasics(yarp::os::ResourceFinder &rf);

    /****************************************************************/
    bool close();

    /****************************************************************/
    bool interruptModule();

    /****************************************************************/
    bool updateModule();

    /****************************************************************/
    double getPeriod();

    /****************************************************************/
    bool configDevices(yarp::os::ResourceFinder &rf, const std::string &arm);

    /****************************************************************/
    bool configAction(yarp::os::ResourceFinder &rf, const std::string &l_o_r);

    /***********************************************************************/
    bool configViewer(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configPose(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configure(yarp::os::ResourceFinder &rf);

    /****************************************************************/
    bool readSuperq(const std::string &name_obj, yarp::sig::Vector &x, const int &dimension, yarp::os::ResourceFinder *rf);


};

#endif
