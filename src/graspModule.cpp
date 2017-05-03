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

#include <csignal>
#include <cmath>
#include <limits>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <deque>

#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/perception/models.h>
#include <iCub/perception/sensors.h>
#include <iCub/perception/tactileFingers.h>

#include "graspModule.h"
#include "superquadric.h"
#include "src/superquadricGrasp_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;
using namespace iCub::iKin;

/************************************************************************/
bool GraspingModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/************************************************************************/
string GraspingModule::get_movement()
{
   // go_on=true;
    if (go_move==true)
        return "yes";
    else
        return "no";
}

/************************************************************************/
bool GraspingModule::set_movement(const string &entry)
{
    if (entry=="yes")
        go_move=true;
    else
        go_move=false;
    return true;
}

/************************************************************************/
bool GraspingModule::start()
{
   // go_on=true;
    stop_var=false;
    return true;
}

/************************************************************************/
bool GraspingModule::stop()
{
    stop_var=true;
    return true;
}

/************************************************************************/
bool GraspingModule::go_home(const string &hand)
{
    bool f;
    f=false;
    // DA DEFINIRE
    return f;
}

/************************************************************************/
bool GraspingModule::set_tag_file(const string &entry)
{
    superq_name=entry;
    object.resize(11,0.0);
    return true;
}

/************************************************************************/
string GraspingModule::get_tag_file()
{
    return superq_name;
}

/************************************************************************/
bool GraspingModule::clear_poses()
{
    poseR.resize(6,0.0);
    poseL.resize(6,0.0);
    object.resize(11,0.0);
    chosen_pose=false;
    //go_on=true;
    stop_var=false;

    return true;
}

/****************************************************************/
bool GraspingModule::configBasics(ResourceFinder &rf)
{
    robot=rf.find("robot").asString().c_str();
    if(rf.find("robot").isNull())
        robot="iCubsim";

    left_or_right=rf.find("which_hand").asString().c_str();
    if(rf.find("which_hand").isNull())
        left_or_right="right";

    // Vediamo quali servono e quali no
    conf_dev_called=false;
    chosen_pose==true;
    conf_act_called=false;
    go_on=false;
    go_move=false;
    stop_var=false;

    lift=rf.find("lift").asString().c_str();
    if(rf.find("lift").isNull())
        lift=true;

    portRpc.open("/superquadric-grasp/rpc");
    attach(portRpc);

    displacement.resize(3,0.0);
    dir=rf.check("approaching_direction", Value("z")).asString();

    rate=rf.check("rate", Value(100)).asInt();
    mode_online=(rf.check("mode_online", Value("on")).asString()=="on");

    return true;
}

/****************************************************************/
bool GraspingModule::close()
{
    graspComp->stop();
    delete graspComp;

    if (portRpc.asPort().isOpen())
        portRpc.close();

    if (portSuperqRpc.asPort().isOpen())
        portSuperqRpc.close();

    if (viewer==true)
    {
        GazeCtrl.close();

        if (!portImgIn.isClosed())
            portImgIn.close();

        if (!portImgOut.isClosed())
            portImgOut.close();
    }

    return true;
}

/****************************************************************/
bool GraspingModule::interruptModule()
{
    if (viewer)
        portImgIn.interrupt();

    return true;
}

/****************************************************************/
bool GraspingModule::updateModule()
{    
    LockGuard lg(mutex);

    if (mode_online)
    {
        complete_sol=graspComp->getSolution();

        yInfo()<<" [GraspModule]: Complete solution "<<complete_sol.toString();

        if (times_grasp.size()<10)
            times_grasp.push_back(graspComp->getTime());
        else if (times_grasp.size()==10)
        {
            for (size_t i=0; i<times_grasp.size(); i++)
            {
                t_grasp+=times_grasp[i];
            }
            t_grasp=t_grasp/times_grasp.size();
            times_grasp.push_back(0.0);
        }
        else
        times_grasp.clear();
    }
    else
    {
        graspComp->threadInit();
        graspComp->object=object;
        graspComp->step();

        complete_sol=graspComp->getSolution();

        yInfo()<<" [GraspModule]: Complete solution "<<complete_sol.toString();
        return false;

    }
   return true;
}

/****************************************************************/
double GraspingModule::getPeriod()
{
    return 0.1;
}

/***********************************************************************/
bool GraspingModule::configViewer(ResourceFinder &rf)
{
    portImgIn.open("/superquadric-grasp/img:i");
    portImgOut.open("/superquadric-grasp/img:o");

    eye=rf.check("eye", Value("left")).asString();

    Property optionG;
    optionG.put("device","gazecontrollerclient");
    optionG.put("remote","/iKinGazeCtrl");
    optionG.put("local","/superquadric-grasp/gaze");

    GazeCtrl.open(optionG);
    igaze=NULL;

    if (GazeCtrl.isValid())
    {
        GazeCtrl.view(igaze);
    }
    else
        return false;

    Bottle info;
    igaze->getInfo(info);
    K.resize(3,4);
    K.zero();

    Bottle *intr_par;

    if (eye=="left")
        intr_par=info.find("camera_intrinsics_left").asList();
    else
        intr_par=info.find("camera_intrinsics_right").asList();

    K(0,0)=intr_par->get(0).asDouble();
    K(0,1)=intr_par->get(1).asDouble();
    K(0,2)=intr_par->get(2).asDouble();
    K(1,1)=intr_par->get(5).asDouble();
    K(1,2)=intr_par->get(6).asDouble();
    K(2,2)=1;

    return true;
}

/***********************************************************************/
bool GraspingModule::configPose(ResourceFinder &rf)
{
    this->rf=&rf;

    poseR.resize(6,0.0);
    poseL.resize(6,0.0);

    mode_online=(rf.check("online", Value("no")).asString()=="yes");
    n_pointshand=rf.check("pointshand", Value(48)).asInt();
    distance=rf.check("distance", Value(0.13)).asDouble();
    distance1=rf.check("distance1", Value(0.05)).asDouble();
    superq_name=rf.check("superq_name", Value("Sponge")).asString();
    max_cpu_time=rf.check("max_cpu_time", Value(5.0)).asDouble();
    shift.resize(3,0.0);
    shift[0]=-0.0;
    shift[1]=-0.0;
    shift[2]=0.0;

    portSuperqRpc.open("/superquadric-grasp/superq:rpc");

    if (!mode_online)
    {
        readSuperq("object",object,11,this->rf);
    }
    else
        object.resize(11,0.0);

    readSuperq("hand",hand,11,this->rf);


    cout<<"left or oright "<<left_or_right<<endl;
    if (left_or_right=="both")
    {
        readSuperq("hand1",hand1,11,this->rf);
    }

    nameFileOut_right=rf.find("nameFileOut_right").asString().c_str();
    if(rf.find("nameFileOut_right").isNull())
       nameFileOut_right="test_right";

    nameFileSolution_right=rf.find("nameFileSolution_right").asString();
    if(rf.find("nameFileSolution_right").isNull())
       nameFileSolution_right="solution_right.txt";

    nameFileTrajectory=rf.find("nameFileTrajectory_right").asString().c_str();
    if(rf.find("nameFileTrajectory_right").isNull())
       nameFileTrajectory="test-trajectory_right.txt";

    nameFileOut_left=rf.find("nameFileOut_left").asString().c_str();
    if(rf.find("nameFileOut_left").isNull())
       nameFileOut_left="test_left";

    nameFileSolution_left=rf.find("nameFileSolution_left").asString();
    if(rf.find("nameFileSolution_left").isNull())
       nameFileSolution_left="solution_left.txt";

    tol=rf.check("tol", Value(1e-3)).asDouble();
    constr_viol_tol=rf.check("constr_tol", Value(1e-2)).asDouble();

    acceptable_iter=rf.check("acceptable_iter", Value(0)).asInt();

    max_iter=rf.check("max_iter", Value(1e8)).asInt();

    mu_strategy=rf.find("mu_strategy").asString().c_str();
    if(rf.find("mu_strategy").isNull())
       mu_strategy="monotone";

    nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
    if(rf.find("nlp_scaling_method").isNull())
       nlp_scaling_method="none";

    ipopt_par.put("max_cpu_time",max_cpu_time);
    ipopt_par.put("tol",tol);
    ipopt_par.put("constr_viol_tol",constr_viol_tol);
    ipopt_par.put("max_iter",max_iter);
    ipopt_par.put("acceptable_iter",acceptable_iter);
    ipopt_par.put("IPOPT_mu_strategy",mu_strategy);
    ipopt_par.put("IPOPT_nlp_scaling_method",nlp_scaling_method);

    pose_par.put("n_pointshand",n_pointshand);
    pose_par.put("hand_displacement_x",displacement[0]);
    pose_par.put("hand_displacement_y",displacement[1]);
    pose_par.put("hand_displacement_z",displacement[2]);

    traj_par.put("distance_on_x",distance);
    traj_par.put("distance_on_z",distance1);
    traj_par.put("approaching_direction",dir);

    return true;
}

/***********************************************************************/
bool GraspingModule::configure(ResourceFinder &rf)
{
    bool config;
    this->rf=&rf;

    configBasics(rf);

    config=configPose(rf);

    graspComp= new GraspComputation(rate, ipopt_par, pose_par, traj_par, left_or_right, hand, hand1, this->rf);

    if (mode_online)
    {
        bool thread_started=graspComp->start();

        if (thread_started)
            yInfo()<<"[GraspComputation]: Thread started!";
        else
            yError()<<"[GraspComputation]: Problems in starting the thread!";
    }

    viewer=(rf.check("viewer", Value("no")).asString()=="yes");

    if (viewer)
        config=configViewer(rf);

    if (config==false)
        return false;

    move=(rf.check("movement", Value("off")).asString()=="on");

    return true;
}

/****************************************************************/
bool GraspingModule::readSuperq(const string &name_obj, Vector &x, const int &dimension, ResourceFinder *rf)
{
    if (Bottle *b=rf->find(name_obj.c_str()).asList())
    {
        if (b->size()>=dimension)
        {
            for(size_t i=0; i<b->size();i++)
                x.push_back(b->get(i).asDouble());
        }
        return true;
    }
}







