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
    LockGuard lg(mutex);

    poseR.resize(6,0.0);
    poseL.resize(6,0.0);
    object.resize(11,0.0);
    complete_sol.clear();

    return true;
}

/**********************************************************************/
Property GraspingModule::get_grasping_pose(const Property &estimated_superq, const string &hand_str)
{
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

    readSuperq("hand",hand,11,this->rf);

    if (left_or_right=="both")
    {
        readSuperq("hand1",hand1,11,this->rf);
    }

    graspComp->setPar("left_or_right", hand_str);
    graspComp->run();
    graspComp->getSolution(hand_str);

    yInfo()<<" [GraspingModule]: Complete solution "<<complete_sol.toString();

    t_grasp=graspComp->getTime();

    graspVis->left_or_right=hand_str;

    readSuperq("hand",graspVis->hand,11,this->rf);

    executed_var=false;

    if (look_object=="on")
        graspVis->look_object=true;

    return complete_sol;
}

/**********************************************************************/
string GraspingModule::get_visualization()
{
    if (visualization)
        return "on";
    else
        return "off";
}

/**********************************************************************/
string GraspingModule::get_best_hand()
{
    return graspComp->best_hand;
}

/**********************************************************************/
bool GraspingModule::set_visualization(const string &e)
{
    if ((e=="on") || (e=="off"))
    {
        LockGuard lg(mutex);

        if ((visualization==false) && (e=="on"))
        {
            graspVis->resume();

            visualization=true;

        }
        else if ((visualization==true) && (e=="off"))
        {
            graspVis->suspend();
            visualization=false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
string GraspingModule::get_hand()
{
    return left_or_right;
}

/**********************************************************************/
bool GraspingModule::set_hand(const string &e)
{
    if ((e=="right") || (e=="left") || (e=="both"))
    {
        LockGuard lg(mutex);

        left_or_right=e;
        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
bool GraspingModule::set_options(const Property &newOptions, const string &field)
{
    if (field=="pose")
        graspComp->setPosePar(newOptions, false);
    else if (field=="trajectory")
        graspComp->setTrajectoryPar(newOptions, false);
    else if (field=="optimization")
        graspComp->setIpoptPar(newOptions, false);
    else if (field=="execution")
        graspExec->setPosePar(newOptions, false);
    else if (field=="visualization")
        graspVis->setPar(newOptions,false);
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
    else if (field=="execution")
        advOptions=graspExec->getPosePar();
    else if (field=="visualization")
        advOptions=graspVis->getPar();
    else if (field=="statistics")
    {
        advOptions.put("average_computation_time", t_grasp);
        advOptions.put("average_visualization_time", t_vis);
    }

    return advOptions;
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
    LockGuard lg(mutex);

    if ((entry=="on") || (entry=="off"))
    {
        save_poses=(entry=="on");
    }
}

/**********************************************************************/
bool GraspingModule::move(const string &entry)
{
    LockGuard lg(mutex);

    yDebug()<<"command received";

    if ((entry=="right") || (entry=="left"))
    {
        hand_to_move=entry;
        executed=false;
        executed_var=false;

        return true;
    }

    return false;
}

/**********************************************************************/
bool GraspingModule::check_motion()
{
    return executed_var;
}

/**********************************************************************/
bool GraspingModule::check_home()
{
    return reached_home;
}

/**********************************************************************/
bool GraspingModule::check_basket()
{
    return reached_basket;
}

/**********************************************************************/
bool GraspingModule::look_center()
{
    Vector center(3,0.0);
    center[0]=-0.35;

    graspVis->igaze->setTrackingMode(false);

    return graspVis->igaze->lookAtFixationPoint(center);
}

/**********************************************************************/
bool GraspingModule::look_obj()
{
    graspVis->look_object=true;

    return true;
}

/**********************************************************************/
bool GraspingModule::go_home(const string &entry)
{
    LockGuard lg(mutex);
    reached_home=false;

    if ((entry=="right") || (entry=="left"))
    {
        executed=true;
        graspExec->reached=true;
        graspExec->reached_tot=true;
        graspExec->stop();
        reached_home=graspExec->goHome(entry);

        return true;
    }
    else if (entry=="both")
    {
        executed=true;
        graspExec->reached=true;
        graspExec->reached_tot=true;
        graspExec->stop();
        graspExec->goHome("right");
        reached_home=graspExec->goHome("left");

        return true;
    }

    return false;
}

/**********************************************************************/
bool GraspingModule::go_to_basket(const string &entry)
{
    LockGuard lg(mutex);
    reached_basket=false;

    if ((entry=="right") || (entry=="left"))
    {
        executed=true;
        graspExec->reached=true;
        graspExec->reached_tot=true;
        graspExec->stop();
        reached_basket=graspExec->goToBasket(entry);

        return true;
    }
    else if (entry=="both")
    {
        executed=true;
        graspExec->reached=true;
        graspExec->reached_tot=true;
        graspExec->stop();
        graspExec->goToBasket("right");
        reached_basket=graspExec->goToBasket("left");

        return true;
    }

    return false;
}


/****************************************************************/
bool GraspingModule::configBasics(ResourceFinder &rf)
{
    homeContextPath=rf.getHomeContextPath().c_str();

    robot=rf.find("robot").asString().c_str();
    if(rf.find("robot").isNull())
        robot="icubSim";

    left_or_right=rf.find("which_hand").asString().c_str();

    if(rf.find("which_hand").isNull())
        left_or_right="right";

    portRpc.open("/superquadric-grasp/rpc");
    attach(portRpc);

    dir=rf.check("approaching_direction", Value("z")).asString();
    rate_vis=rf.check("rate_vis", Value(100)).asInt();
    mode_online=(rf.check("mode_online", Value("on")).asString()=="on");
    save_poses=(rf.check("save_poses", Value("on")).asString()=="on");
    also_traj=(rf.check("also_traj", Value("off")).asString()=="on");
    visualization=(rf.check("visualization", Value("off")).asString()=="on");
    grasp=(rf.check("grasp", Value("off")).asString()=="on");
    visual_servoing=rf.check("visual_servoing", Value("off")).asString();   
    compliant=rf.check("compliant", Value("off")).asString();
    use_direct_kin=rf.check("use_direct_kin", Value("off")).asString();
    print_level=rf.check("print_level", Value(0)).asInt();

    go_on=false;

    return true;
}

/****************************************************************/
bool GraspingModule::configMovements(ResourceFinder &rf)
{
    traj_time=rf.check("trajectory_time", Value(1.0)).asDouble();
    traj_tol=rf.check("trajectory_tol", Value(0.01)).asDouble();
    pixel_tol=rf.check("pixel_tol", Value(15)).asDouble();
    force_threshold=rf.check("force_threshold", Value(6.0)).asDouble();
    lift_z=rf.check("lift_z", Value(0.15)).asDouble();
    torso_pitch_max=rf.check("torso_pitch_max", Value(15.0)).asDouble();
    fing=rf.check("five_fingers", Value("off")).asString();
    lobj=rf.check("lift_object", Value("off")).asString();

    readSuperq("shift_right",shift_right,3,this->rf);
    readSuperq("shift_left",shift_left,3,this->rf);
    readSuperq("home_right",home_right,7,this->rf);
    readSuperq("home_left",home_left,7,this->rf);
    readSuperq("basket_right",basket_right,7,this->rf);
    readSuperq("basket_left",basket_left,7,this->rf);
    readSuperq("stiff_right",stiff_right,5,this->rf);
    readSuperq("stiff_left",stiff_left,5,this->rf);
    readSuperq("damp_right",damp_right,5,this->rf);
    readSuperq("damp_left",damp_left,5,this->rf);


    movement_par.put("robot",robot);
    movement_par.put("hand",left_or_right);
    movement_par.put("five_fingers",fing);
    movement_par.put("five_fingers",fing);
    movement_par.put("lift_object",lobj);

    movement_par.put("traj_time",traj_time);
    movement_par.put("force_threshold",force_threshold);
    movement_par.put("traj_tol",traj_tol);
    movement_par.put("lift_z", lift_z);
    movement_par.put("torso_pitch_max", torso_pitch_max);
    movement_par.put("visual_servoing", visual_servoing);
    movement_par.put("compliant", compliant);
    movement_par.put("use_direct_kin", use_direct_kin);
    movement_par.put("pixel_tol", pixel_tol);

    Bottle shift_r;
    Bottle &bottle_shift_r=shift_r.addList();
    bottle_shift_r.addDouble(shift_right[0]); bottle_shift_r.addDouble(shift_right[1]);
    bottle_shift_r.addDouble(shift_right[2]);
    movement_par.put("shift_right",shift_r.get(0));

    Bottle shift_l;
    Bottle &bottle_shift_l=shift_l.addList();
    bottle_shift_l.addDouble(shift_left[0]); bottle_shift_l.addDouble(shift_left[1]);
    bottle_shift_l.addDouble(shift_left[2]);
    movement_par.put("shift_left",shift_l.get(0));

    Bottle home_r;
    Bottle &bottle_home_r=home_r.addList();
    bottle_home_r.addDouble(home_right[0]); bottle_home_r.addDouble(home_right[1]);
    bottle_home_r.addDouble(home_right[2]); bottle_home_r.addDouble(home_right[3]);
    bottle_home_r.addDouble(home_right[4]); bottle_home_r.addDouble(home_right[5]);bottle_home_r.addDouble(home_right[6]);
    movement_par.put("home_right", home_r.get(0));

    Bottle home_l;
    Bottle &bottle_home_l=home_l.addList();
    bottle_home_l.addDouble(home_left[0]); bottle_home_l.addDouble(home_left[1]);
    bottle_home_l.addDouble(home_left[2]); bottle_home_l.addDouble(home_left[3]);
    bottle_home_l.addDouble(home_left[4]); bottle_home_l.addDouble(home_left[5]);bottle_home_l.addDouble(home_left[6]);
    movement_par.put("home_left", home_l.get(0));

    Bottle bask_r;
    Bottle &bottle_bask_r=bask_r.addList();
    bottle_bask_r.addDouble(basket_right[0]); bottle_bask_r.addDouble(basket_right[1]);
    bottle_bask_r.addDouble(basket_right[2]); bottle_bask_r.addDouble(basket_right[3]);
    bottle_bask_r.addDouble(basket_right[4]); bottle_bask_r.addDouble(basket_right[5]);bottle_bask_r.addDouble(basket_right[6]);
    movement_par.put("basket_right", bask_r.get(0));

    Bottle bask_l;
    Bottle &bottle_bask_l=bask_l.addList();
    bottle_bask_l.addDouble(basket_left[0]); bottle_bask_l.addDouble(basket_left[1]);
    bottle_bask_l.addDouble(basket_left[2]); bottle_bask_l.addDouble(basket_left[3]);
    bottle_bask_l.addDouble(basket_left[4]); bottle_bask_l.addDouble(basket_left[5]);bottle_bask_l.addDouble(basket_left[6]);
    movement_par.put("basket_left", bask_l.get(0));

    Bottle stiff_r;
    Bottle &bottle_stiff_r=stiff_r.addList();
    bottle_stiff_r.addDouble(stiff_right[0]); bottle_stiff_r.addDouble(stiff_right[1]);
    bottle_stiff_r.addDouble(stiff_right[2]); bottle_stiff_r.addDouble(stiff_right[3]);
    bottle_stiff_r.addDouble(stiff_right[4]);
    movement_par.put("stiff_right", stiff_r.get(0));

    Bottle stiff_l;
    Bottle &bottle_stiff_l=stiff_l.addList();
    bottle_stiff_l.addDouble(stiff_left[0]); bottle_stiff_l.addDouble(stiff_left[1]);
    bottle_stiff_l.addDouble(stiff_left[2]); bottle_stiff_l.addDouble(stiff_left[3]);
    bottle_stiff_l.addDouble(stiff_left[4]);
    movement_par.put("stiff_left", stiff_l.get(0));

    Bottle damp_r;
    Bottle &bottle_damp_r=damp_r.addList();
    bottle_damp_r.addDouble(damp_right[0]); bottle_damp_r.addDouble(damp_right[1]);
    bottle_damp_r.addDouble(damp_right[2]); bottle_damp_r.addDouble(damp_right[3]);
    bottle_damp_r.addDouble(damp_right[4]);
    movement_par.put("damp_right", damp_r.get(0));

    Bottle damp_l;
    Bottle &bottle_damp_l=damp_l.addList();
    bottle_damp_l.addDouble(damp_left[0]); bottle_damp_l.addDouble(damp_left[1]);
    bottle_damp_l.addDouble(damp_left[2]); bottle_damp_l.addDouble(damp_left[3]);
    bottle_damp_l.addDouble(damp_left[4]);
    movement_par.put("damp_left", damp_l.get(0));

    executed=true;
    hand_to_move="right";

    yInfo()<<"[GraspingModule] lift_z:        "<<lift_z;
    yInfo()<<"[GraspingModule] force_threshold: "<<force_threshold;
    yInfo()<<"[GraspingModule] shift_right:   "<<shift_right.toString(3,3);
    yInfo()<<"[GraspingModule] shift_left:    "<<shift_left.toString(3,3);
    yInfo()<<"[GraspingModule] home_right:    "<<home_right.toString(3,3);
    yInfo()<<"[GraspingModule] home_left:     "<<home_left.toString(3,3);
    yInfo()<<"[GraspingModule] basket_right:  "<<basket_right.toString(3,3);
    yInfo()<<"[GraspingModule] basket_left:   "<<basket_left.toString(3,3);
    yInfo()<<"[GraspingModule] stiff_right:   "<<stiff_right.toString(3,3);
    yInfo()<<"[GraspingModule] stiff_left:    "<<stiff_left.toString(3,3);
    yInfo()<<"[GraspingModule] damp_right:    "<<damp_right.toString(3,3);
    yInfo()<<"[GraspingModule] damp_left:     "<<damp_left.toString(3,3);

    return true;
}

/****************************************************************/
bool GraspingModule::configGrasp(ResourceFinder &rf)
{
    lib_context=rf.check("lib_context", Value("superquadric-grasp")).asString();
    lib_filename=rf.check("lib_filename", Value("confTactileControlLib")).asString();

    return true;
}

/****************************************************************/
bool GraspingModule::close()
{
    delete graspComp;

    graspExec->release();
    delete graspExec;

    if (visualization)
        graspVis->stop();

    delete graspVis;

    if (portRpc.asPort().isOpen())
        portRpc.close();

    igaze->stopControl();

    igaze->restoreContext(context_gaze);

    GazeCtrl.close();

    return true;
}

/****************************************************************/
bool GraspingModule::interruptModule()
{
    return true;
}

/****************************************************************/
bool GraspingModule::updateModule()
{
    if (visualization)
    {
        if (times_vis.size()<10)
            times_vis.push_back(graspVis->getTime());
        else if (times_vis.size()==10)
        {
            for (size_t i=0; i<times_vis.size(); i++)
            {
                t_vis+=times_vis[i];
            }
            t_vis=t_vis/times_vis.size();
            times_vis.push_back(0.0);
        }
        else
            times_vis.clear();
    }

    if (executed==false)
    {
        graspExec->getPoses(complete_sol);
        executed_var=executed=graspExec->executeTrajectory(hand_to_move);

        yDebug()<<"[GraspingModule] Movement executed :"<<executed_var;
    }

    if (save_poses && (graspComp->count_file == graspComp->count_file_old))
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
    block_eye=rf.check("block_eye", Value(5.0)).asDouble();
    block_neck=rf.check("block_neck", Value(0.0)).asDouble();
    eye=rf.check("eye", Value("left")).asString();
    show_hand=rf.check("show_hand", Value("on")).asString();
    look_object=rf.check("look_object", Value("on")).asString();
    show_only_pose=rf.check("show_only_pose", Value("on")).asString();

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

    igaze->storeContext(&context_gaze);

    igaze->setSaccadesMode(false);

    yDebug()<<"[GraspingModule]: Blocking eyes..."<<block_eye;
    igaze->blockEyes(block_eye);
    yDebug()<<"[GraspingModule]: Done: "<<igaze->waitMotionDone(2.0);

    yDebug()<<"[GraspingModule]: Blocking roll..."<<block_neck;
    igaze->blockNeckRoll(block_neck);
    yDebug()<<"[GraspingModule]: Done: "<<igaze->waitMotionDone(2.0);

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


    vis_par.put("look_object",look_object);
    vis_par.put("show_hand", show_hand);

    return true;
}

/***********************************************************************/
bool GraspingModule::configPose(ResourceFinder &rf)
{
    this->rf=&rf;

    n_pointshand=rf.check("pointshand", Value(48)).asInt();
    distance=rf.check("distance_on_x", Value(0.13)).asDouble();
    distance1=rf.check("distance_on_z", Value(0.05)).asDouble();
    max_cpu_time=rf.check("max_cpu_time", Value(5.0)).asDouble();

    object.resize(11,0.0);

    readSuperq("hand",hand,11,this->rf);

    if (left_or_right=="both")
    {
        readSuperq("hand1",hand1,11,this->rf);
    }

    readSuperq("displacement",displacement,3,this->rf);
    readSuperq("plane",plane,4,this->rf);

    if (plane.size()==0 && displacement.size()==0)
    {
        yError()<<"[GraspingModule]: No plane and displacement provided!";
        return false;
    }

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
    ipopt_par.put("print_level",print_level);

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

    return true;
}

/***********************************************************************/
bool GraspingModule::configure(ResourceFinder &rf)
{
    bool config;
    this->rf=&rf;

    configBasics(rf);

    config=configPose(rf);

    graspComp= new GraspComputation(ipopt_par, pose_par, traj_par, left_or_right, hand, hand1, this->rf, complete_sol, object, quality_right, quality_left);

    graspComp->init();

    config=configViewer(rf);

    if (config==false)
        return false;

    graspVis= new GraspVisualization(rate_vis,eye,igaze,executed_var, K, left_or_right, complete_sol, object, hand, hand1, vis_par, quality_right, quality_left);

    if (visualization)
    {
        bool thread_started=graspVis->start();

        if (thread_started)
            yInfo()<<"[GraspingModule]: Visualization hread started!";
        else
            yError()<<"[GraspingModule]: Problems in starting the visualization thread!";
    }
    else
    {
        graspVis->start();
        graspVis->suspend();
    }

    configMovements(rf);

    configGrasp(rf);

    graspExec= new GraspExecution(movement_par, complete_sol, grasp, lib_context, lib_filename);

    config=graspExec->configure();

    executed_var=false;


    if (config==false)
        return false;

    return true;
}

/****************************************************************/
bool GraspingModule::readSuperq(const string &name_obj, Vector &x, const int &dimension, ResourceFinder *rf)
{
    x.clear();
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
void GraspingModule::saveSol(const Property &poses)
{
    stringstream ss;
    ss << graspComp->count_file;
    string count_file=ss.str();

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

    if ((left_or_right=="right" || left_or_right=="both") && (norm(poseR)!=0.0))
    {
        ofstream fout((homeContextPath+"/"+nameFileOut_right+ "_"+count_file+".txt").c_str());

        yDebug()<<"[GraspingModule]: Saving solution for right hand in "<<(homeContextPath+"/"+nameFileOut_right+ "_"+count_file+".txt").c_str();

        if(fout.is_open())
        {
            fout<<"Initial right hand volume pose: "<<endl<<"["<<hand.toString(3,3).c_str()<<"]"<<endl<<endl;

            fout<<"Final right hand pose: "<<endl<<"["<<poseR.toString(3,3)<<"]"<<endl<<endl;

            fout<<"Average time for computation: "<<endl<<t_grasp<<endl<<endl;

            fout<<"object: "<<endl<<"["<<object.toString(3,3)<<"]"<<endl<<endl;
        }  
    }

    if ((left_or_right=="left" || left_or_right=="both") && (norm(poseL)!=0.0))
    {
        ofstream fout((homeContextPath+"/"+nameFileOut_left+ "_"+count_file+".txt").c_str());

        yDebug()<<"[GraspingModule]: Saving solution for left hand in "<<(homeContextPath+"/"+nameFileOut_left+ "_"+count_file+".txt").c_str();

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

    if ((left_or_right=="right" || left_or_right=="both") && (norm(poseR)!=0.0) && (also_traj==true))
    {
        ofstream fout((homeContextPath+"/"+nameFileTrajectory_right+ "_"+count_file+".txt").c_str());

        yDebug()<<"[GraspingModule]: Saving trajectory for right hand in "<<(homeContextPath+nameFileOut_left+ "_"+count_file+".txt").c_str();

        if(fout.is_open())
        {
            fout<<"Trajectory for right hand: "<<endl;
            for (size_t i=0; i<trajectory_right.size(); i++)
            {
                fout<<"["<<trajectory_right[i].toString(3,3).c_str()<<"]"<<endl<<endl;
            }
        }
    }

    if ((left_or_right=="left" || left_or_right=="both") && (norm(poseL)!=0.0) && (also_traj==true))
    {
        ofstream fout((homeContextPath+"/"+nameFileTrajectory_left+ "_"+count_file+".txt").c_str());

        yDebug()<<"[GraspingModule]: Saving trajectory for left hand in "<<(homeContextPath+nameFileOut_left+ "_"+count_file+".txt").c_str();

        if(fout.is_open())
        {
            fout<<"Trajectory for left hand: "<<endl;
            for (size_t i=0; i<trajectory_left.size(); i++)
            {
                fout<<"["<<trajectory_left[i].toString(3,3).c_str()<<"]"<<endl<<endl;
            }
        }
    }

    graspComp->count_file_old++;
}













