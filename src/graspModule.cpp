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
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <deque>

#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>

#include "graspModule.h"
#include "superquadric.h"
#include "src/superquadricGrasp_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

/************************************************************************/
bool GraspingModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/************************************************************************/
bool GraspingModule::clear_poses()
{
    poseR.resize(6,0.0);
    poseL.resize(6,0.0);
    object.resize(11,0.0);

    return true;
}

/****************************************************************/
bool GraspingModule::configBasics(ResourceFinder &rf)
{
    homeContextPath=rf.getHomeContextPath().c_str();

    robot=rf.find("robot").asString().c_str();
    if(rf.find("robot").isNull())
        robot="iCubsim";

    left_or_right=rf.find("which_hand").asString().c_str();
    if(rf.find("which_hand").isNull())
        left_or_right="right";

    portRpc.open("/superquadric-grasp/rpc");
    attach(portRpc);

    dir=rf.check("approaching_direction", Value("z")).asString();
    rate=rf.check("rate", Value(100)).asInt();
    mode_online=(rf.check("mode_online", Value("on")).asString()=="on");
    save_poses=(rf.check("save_poses", Value("on")).asString()=="on");
    viewer=(rf.check("viewer", Value("no")).asString()=="yes");

    go_on=false;

    return true;
}

/****************************************************************/
bool GraspingModule::close()
{
    graspComp->stop();
    delete graspComp;

    if (portRpc.asPort().isOpen())
        portRpc.close();

    if (portPose.isClosed())
        portPose.close();

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
        Property &complete_sol=portPose.prepare();

        complete_sol=graspComp->getSolution(left_or_right);

        yInfo()<<" [GraspingModule]: Complete solution "<<complete_sol.toString();

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

        portPose.write();
    }
    else
    {
        graspComp->threadInit();
        graspComp->setPar("one_shot", "on");
        graspComp->object=object;
        graspComp->step();

        complete_sol=graspComp->getSolution(left_or_right);
        t_grasp=graspComp->getTime();

        yInfo()<<" [GraspingModule]: Complete solution: "<<complete_sol.toString();

        if (save_poses)
            saveSol(complete_sol);

        return true;
    }

    if (save_poses)
        saveSol(complete_sol);

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

    n_pointshand=rf.check("pointshand", Value(48)).asInt();
    distance=rf.check("distance", Value(0.13)).asDouble();
    distance1=rf.check("distance1", Value(0.05)).asDouble();
    max_cpu_time=rf.check("max_cpu_time", Value(5.0)).asDouble();

    if (!mode_online)
    {
        readSuperq("object",object,11,this->rf);
    }
    else
        object.resize(11,0.0);

    readSuperq("hand",hand,11,this->rf);

    if (left_or_right=="both")
    {
        readSuperq("hand1",hand1,11,this->rf);
    }

    readSuperq("displacement",displacement,3,this->rf);
    readSuperq("plane",plane,4,this->rf);

    nameFileOut_right=rf.find("nameFileOut_right").asString().c_str();
    if(rf.find("nameFileOut_right").isNull())
       nameFileOut_right="test_right";

    nameFileTrajectory_right=rf.find("nameFileTrajectory_right").asString().c_str();
    if(rf.find("nameFileTrajectory_right").isNull())
       nameFileTrajectory_right="test-trajectory_right.txt";

    nameFileOut_left=rf.find("nameFileOut_left").asString().c_str();
    if(rf.find("nameFileOut_left").isNull())
       nameFileOut_left="test_left";

    nameFileTrajectory_left=rf.find("nameFileTrajectory_left").asString().c_str();
    if(rf.find("nameFileTrajectory_left").isNull())
       nameFileTrajectory_left="test-trajectory_left.txt";

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
    Bottle planed;
    Bottle &pd=planed.addList();
    pd.addDouble(displacement[0]); pd.addDouble(displacement[1]);
    pd.addDouble(displacement[2]);
    pose_par.put("hand_displacement",planed.get(0));
    Bottle planeb;
    Bottle &p2=planeb.addList();
    p2.addDouble(plane[0]); p2.addDouble(plane[1]);
    p2.addDouble(plane[2]); p2.addDouble(plane[3]);
    pose_par.put("plane", planeb.get(0));


    traj_par.put("distance_on_x",distance);
    traj_par.put("distance_on_z",distance1);
    traj_par.put("approaching_direction",dir);

    poseR.resize(6,0.0);
    poseL.resize(6,0.0);

    portPose.open("/superquadric-grasp/pose:o");

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

    if (viewer)
        config=configViewer(rf);

    if (config==false)
        return false;

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


/**********************************************************************/
string GraspingModule::get_save_poses()
{
    if (save_poses)
    {
        return "on";
    }
    else
    {
        return "off";
    }
}

/**********************************************************************/
bool GraspingModule::set_save_poses(const string &entry)
{
    if ((entry=="on") || (entry=="off"))
    {
        save_poses=(entry=="on");
    }
}

/**********************************************************************/
void GraspingModule::saveSol(const Property &poses)
{
    Bottle &pose=poses.findGroup("pose_right");
    if (!pose.isNull())
    {
        Bottle *p=pose.get(1).asList();

        for (size_t i=0; i<p->size(); i++)
            poseR[i]=p->get(i).asDouble();
    }
    else
        yError()<<"[GraspingModule]: No pose right found!";

    Bottle &pose1=poses.findGroup("pose_left");
    if (!pose1.isNull())
    {
        Bottle *p=pose1.get(1).asList();

        for (size_t i=0; i<p->size(); i++)
            poseL[i]=p->get(i).asDouble();
    }
    else
        yError()<<"[GraspingModule]: No pose left found!";

    if (left_or_right=="right" || left_or_right=="both")
    {
        ofstream fout((homeContextPath+"/"+nameFileOut_right).c_str());

        yDebug()<<"[GraspingModule]: Saving solution for right hand in "<<(homeContextPath+"/"+nameFileOut_right).c_str();

        if(fout.is_open())
        {
            fout<<"Initial right hand volume pose: "<<endl<<"["<<hand.toString(3,3).c_str()<<"]"<<endl<<endl;

            fout<<"Final right hand pose: "<<endl<<"["<<poseR.toString(3,3)<<"]"<<endl<<endl;

            fout<<"Average time for computation: "<<endl<<t_grasp<<endl<<endl;

            fout<<"object: "<<endl<<"["<<object.toString(3,3)<<"]"<<endl<<endl;
        }
    }

    if (left_or_right=="left" || left_or_right=="both")
    {
        ofstream fout((homeContextPath+"/"+nameFileOut_left).c_str());

        yDebug()<<"[GraspingModule]: Saving solution for left hand in "<<(homeContextPath+"/"+nameFileOut_left).c_str();

        if(fout.is_open())
        {
            if (left_or_right=="left")
                fout<<"Initial left hand volume pose: "<<endl<<"["<<hand.toString(3,3).c_str()<<"]"<<endl<<endl;
            else
                fout<<"Initial left hand volume pose: "<<endl<<"["<<hand1.toString(3,3).c_str()<<"]"<<endl<<endl;

            fout<<"Final left hand pose: "<<endl<<"["<<poseL.toString(3,3)<<"]"<<endl<<endl;

            fout<<"Average time for computation: "<<endl<<t_grasp<<endl<<endl;

            fout<<"object: "<<endl<<"["<<object.toString(3,3)<<"]"<<endl<<endl;
        }
    }

    Vector tmp(6,0.0);
    Bottle &pose2=poses.findGroup("trajectory_right");

    if (!pose2.isNull())
    {
        Bottle *p=pose2.get(1).asList();
        for (size_t i=0; i<p->size(); i++)
        {
            Bottle *p1=p->get(i).asList();


            for (size_t j=0; j<p1->size(); j++)
            {
                tmp[i]=p1->get(j).asDouble();
            }
            trajectory_right.push_back(tmp);
        }
    }
    else
        yError()<<"[GraspingModule]: No trajectory right found!";

    Bottle &pose3=poses.findGroup("trajectory_left");

    if (!pose3.isNull())
    {
        Bottle *p=pose3.get(1).asList();
        for (size_t i=0; i<p->size(); i++)
        {
            Bottle *p1=p->get(i).asList();


            for (size_t j=0; j<p1->size(); j++)
            {
                tmp[i]=p1->get(j).asDouble();
            }
            trajectory_left.push_back(tmp);
        }
    }
    else
        yError()<<"[GraspingModule]: No trajectory left found!";

    if (left_or_right=="right" || left_or_right=="both")
    {
        ofstream fout((homeContextPath+"/"+nameFileTrajectory_right).c_str());

        yDebug()<<"[GraspingModule]: Saving trajectory for right hand in "<<(homeContextPath+nameFileOut_left).c_str();

        if(fout.is_open())
        {
            fout<<"Trajectory for right hand: "<<endl;
            for (size_t i=0; i<trajectory_right.size(); i++)
            {
                fout<<"["<<trajectory_right[i].toString(3,3).c_str()<<"]"<<endl<<endl;
            }
        }
    }

    if (left_or_right=="left" || left_or_right=="both")
    {
        ofstream fout((homeContextPath+"/"+nameFileTrajectory_left).c_str());

        yDebug()<<"[GraspingModule]: Saving trajectory for left hand in "<<(homeContextPath+nameFileOut_left).c_str();

        if(fout.is_open())
        {
            fout<<"Trajectory for left hand: "<<endl;
            for (size_t i=0; i<trajectory_left.size(); i++)
            {
                fout<<"["<<trajectory_left[i].toString(3,3).c_str()<<"]"<<endl<<endl;
            }
        }
    }
}

/**********************************************************************/
Property GraspingModule::get_grasping_pose(const Property &estimated_superq, const string &hand)
{
    Property pose;

    Bottle *dim=estimated_superq.find("dimensions").asList();

    if (!estimated_superq.find("dimensions").isNull())
    {
        object[0]=dim->get(0).asDouble(); object[1]=dim->get(1).asDouble(); object[2]=dim->get(2).asDouble();
    }

    Bottle *shape=estimated_superq.find("exponents").asList();

    if (!estimated_superq.find("exponents").isNull())
    {
        object[3]=shape->get(0).asDouble(); object[4]=shape->get(1).asDouble();
    }

    Bottle *exp=estimated_superq.find("exponents").asList();

    if (!estimated_superq.find("exponents").isNull())
    {
        object[3]=exp->get(0).asDouble(); object[4]=exp->get(1).asDouble();
    }

    Bottle *center=estimated_superq.find("center").asList();

    if (!estimated_superq.find("center").isNull())
    {
        object[5]=center->get(0).asDouble(); object[6]=center->get(1).asDouble(); object[7]=center->get(2).asDouble();
    }

    Bottle *orientation=estimated_superq.find("orientation").asList();

    if (!estimated_superq.find("orientation").isNull())
    {
        Vector axis(4,0.0);
        axis[0]=orientation->get(0).asDouble(); axis[1]=orientation->get(1).asDouble(); axis[2]=orientation->get(2).asDouble(); axis[3]=orientation->get(3).asDouble();
        object.setSubvector(8,dcm2euler(axis2dcm(axis)));
    }


    graspComp->object=object;
    graspComp->setPar("left_or_right", hand);

    pose=graspComp->getSolution(hand);

    graspComp->setPar("left_or_right", left_or_right);
    graspComp->object.resize(11,0.0);

    return pose;
}

/**********************************************************************/
bool GraspingModule::set_options(const Property &newOptions, const string &field)
{
    if (field=="pose")
        graspComp->setPosePar(newOptions);
    else if (field=="trajectory")
        graspComp->setTrajectoryPar(newOptions);
    else if (field=="optimization")
        graspComp->setIpoptPar(newOptions);
    else
        return false;

    return true;
}

/**********************************************************************/
Property GraspingModule::get_options(const string &field)
{
    Property advOptions;
    if (field=="pose")
        advOptions=graspComp->getPosePar();
    else if (field=="trajectory")
        advOptions=graspComp->getTrajectoryPar();
    else if (field=="optimization")
        advOptions=graspComp->getIpoptPar();
    else if (field=="statistics")
    {
        advOptions.put("average_computation_time", t_grasp);
    }

    return advOptions;
}








