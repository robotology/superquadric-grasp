/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <csignal>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <deque>
#include <vector>

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
    complete_sols.clear();

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

    object_vis=object;

    obstacles.clear();
    obstacles_vis=obstacles;

    multiple_superq=false;
    best_scenario=-1;

    readSuperq("hand",hand,11,this->rf);

    if (left_or_right=="both")
    {
        readSuperq("hand1",hand1,11,this->rf);
    }

    graspComp->setPar("left_or_right", hand_str);
    graspComp->run();
    graspComp->getSolution(hand_str);

    cost_vis_r.clear();
    cost_vis_l.clear();
    cost_vis_r.push_back(graspComp->cost_right);
    cost_vis_l.push_back(graspComp->cost_left);

    yInfo()<<" [GraspingModule]: Complete solution "<<complete_sol.toString();

    t_grasp=graspComp->getTime();

    if (visualization)
        graspVis->left_or_right=hand_str;

    executed_var=false;

    if (look_object=="on" && visualization==true)
        graspVis->look_object=true;

    complete_sols.clear();
    complete_sols.push_back(complete_sol);

    return complete_sol;
}

/**********************************************************************/
Property GraspingModule::get_grasping_pose_multiple(const Property &estimated_superq, const Property &obstacle_ext, const string &hand)
{
    Vector obstacle(11,0.0);
    complete_sols.clear();
    solutions.clear();

    best_scenario=-1;

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

    object_vis=object;

    Bottle *obs=obstacle_ext.find("obstacles").asList();
    obstacles.clear();
    obstacles_vis.clear();
    if (obs!=nullptr)
    {
        for (size_t i =0; i<obs->size(); i++)
        {
            Bottle *obstacle_sing=obs->get(i).asList();

            obstacle[0]=obstacle_sing->get(0).asDouble(); obstacle[1]=obstacle_sing->get(1).asDouble(); obstacle[2]=obstacle_sing->get(2).asDouble();
            obstacle[3]=obstacle_sing->get(3).asDouble(); obstacle[4]=obstacle_sing->get(4).asDouble(); obstacle[5]=obstacle_sing->get(5).asDouble();
            obstacle[6]=obstacle_sing->get(6).asDouble(); obstacle[7]=obstacle_sing->get(7).asDouble();
            Vector axis(4,0.0);
            axis[0]=obstacle_sing->get(8).asDouble(); axis[1]=obstacle_sing->get(9).asDouble(); axis[2]=obstacle_sing->get(10).asDouble(); axis[3]=obstacle_sing->get(11).asDouble();
            obstacle.setSubvector(8,dcm2euler(axis2dcm(axis)));

            obstacles.push_back(obstacle);
            obstacles_vis.push_back(obstacle);
        }

        if (obs->size()>0)
            multiple_superq=true;
        else
            multiple_superq=false;
    }
    else
        multiple_superq=false;


    deque<double> cost;

    Vector obj_aux=object;
    deque<Vector> obs_aux;
    obs_aux=obstacles;

    readSuperq("hand",graspComp->hand,11,this->rf);

    if (left_or_right=="both")
    {
        readSuperq("hand1",graspComp->hand1,11,this->rf);
    }

    t0=Time::now();

    graspComp->setPar("left_or_right", hand);
    graspComp->run();
    graspComp->getSolution(hand);

    yInfo()<<" [GraspingModule]: Complete solution scenario 0 ";
    cout<<complete_sol.toString();
    cout<<endl<<endl;

    solutions.push_back(complete_sol);

    best_hands.clear();
    best_hands.push_back(graspComp->best_hand);

    cost_vis_r.clear();
    cost_vis_l.clear();

    cost_vis_r.push_back(graspComp->cost_right);
    cost_vis_l.push_back(graspComp->cost_left);


    if (graspComp->best_hand=="right")
        cost.push_back(graspComp->cost_right);
    else
        cost.push_back(graspComp->cost_left);

    double obst_size=obstacles.size();

    for (size_t i=0; i<obst_size; i++)
    {
        readSuperq("hand",graspComp->hand,11,this->rf);

        if (left_or_right=="both")
        {
            readSuperq("hand1",graspComp->hand1,11,this->rf);
        }

        object=obs_aux[i];
        obstacles.clear();
        obstacles.push_back(obj_aux);

        for (size_t j=0; j<obst_size; j++)
        {
            if (j!=i)
                obstacles.push_back(obs_aux[j]);
        }
        graspComp->run();
        graspComp->getSolution(hand);

        yInfo()<<" [GraspingModule]: Complete solution scenario"<<i+1;
        cout<<complete_sol.toString();
        cout<<endl<<endl;
        solutions.push_back(complete_sol);

        cost_vis_r.push_back(graspComp->cost_right);
        cost_vis_l.push_back(graspComp->cost_left);

        if (graspComp->best_hand=="right")
            cost.push_back(graspComp->cost_right);
        else
            cost.push_back(graspComp->cost_left);

        best_hands.push_back(graspComp->best_hand);
    }

    t_grasp=graspComp->getTime();

    if (visualization)
        graspVis->left_or_right=hand;

    complete_sols=solutions;

    double best_cost;

    if (!multiple_superq)
        return complete_sol;
    else
    {
        best_cost=cost[0];
        for (size_t i=0; i<cost.size() - 1; i++)
        {
            yDebug()<<"i "<<i;
            yDebug()<<"cost "<<cost[i];
            if ((best_cost >= cost[i+1]) && (cost[i+1]>0.0))
            {
                best_cost=cost[i+1];
                best_scenario=i+1;
            }
            else if ((i==0) && (cost[i]>0.0))
                best_scenario=i;

            yDebug()<<"best scenerio "<<best_scenario;
        }

        yInfo()<<"Best scenario "<< best_scenario<< "with cost: "<<best_cost;
        for (int k=0; k<best_hands.size(); k++)
            yInfo()<<"Best hands "<< best_hands[k];

        Property all_sol;
        if(graspComp->left_right!="both")
            putPropertiesTogether(solutions, graspComp->left_right, all_sol);
        else
        {
            putPropertiesTogether(solutions, "left", all_sol);
            putPropertiesTogether(solutions, "right", all_sol);
        }

        t_grasp=Time::now() - t0;

        cout<<endl;
        cout<<"----------------------------------------------------------------------------------------------------";
        cout<<endl;
        yInfo()<<"             Time for computing all poses      :"<<t_grasp;
        cout<<"-------------------------------------------------------------------------------";

        //Temporary
        //if (best_scenario>0)
        //    return solutions[best_scenario];
        //else
        //    return solutions[0];

        return all_sol;
    }
}
/**********************************************************************/
void GraspingModule::putPropertiesTogether(deque<Property> &solutions, const string &hand, Property &all_sols)
{
    for (size_t i=0; i<solutions.size(); i++)
    {
        stringstream ss;
        ss<<i;

        Bottle all_list;
        yDebug()<<"sol size "<<solutions.size();
        yDebug()<<"sol "<<solutions[i].toString();


        Bottle *content_list1=solutions[i].find("pose_0_"+hand).asList();
        Bottle *content_list2=solutions[i].find("solution_0_"+hand).asList();
        double cost=solutions[i].find("cost_0_"+hand).asDouble();
        Bottle &content_list_sup1=all_list.addList();
        Bottle &content_list_sup2=all_list.addList();
        double dim=solutions[i].find("hand_length_0_"+hand).asDouble();

        content_list_sup1=*content_list1;
        content_list_sup2=*content_list2;

        yDebug()<<"all_list "<<all_list.toString();

        all_sols.put("pose_"+ss.str()+"_"+hand, all_list.get(0));
        all_sols.put("solution_"+ss.str()+"_"+hand, all_list.get(1));
        all_sols.put("cost_"+ss.str()+"_"+hand, cost);
        all_sols.put("hand_length_"+ss.str()+"_"+hand, dim);

    }

    yDebug()<<"all sols "<<all_sols.toString();
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
    if (!multiple_superq)
        return graspComp->best_hand;
    else
    {
        return best_hands[best_scenario];
    }
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
    {
        graspComp->setPosePar(newOptions, false);
    }
    else if (field=="trajectory")
    {
        graspComp->setTrajectoryPar(newOptions, false);
    }
    else if (field=="optimization")
    {
        graspComp->setIpoptPar(newOptions, false);
    }
    else if (field=="execution")
    {
        graspExec->setPosePar(newOptions, false);
    }
    else if (field=="visualization")
    {
        graspVis->setPar(newOptions,false);
    }
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
    if (visualization)
    {
        Vector center(3,0.0);
        center[0]=-0.35;

        graspVis->igaze->setTrackingMode(false);

        return graspVis->igaze->lookAtFixationPoint(center);
    }
    else
        return false;
}

/**********************************************************************/
bool GraspingModule::look_obj()
{
    if (visualization)
    {
        graspVis->look_object=true;

        return true;
    }
    else
        return false;
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
    demo=(rf.check("demo", Value("off")).asString()=="on");

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

    Bottle shift_right_bottle;
    Bottle &pd=shift_right_bottle.addList();
    pd.addDouble(shift_right[0]); pd.addDouble(shift_right[1]);
    pd.addDouble(shift_right[2]);
    movement_par.put("shift_right",shift_right_bottle.get(0));
    Bottle shift_left_bottle;
    Bottle &pd2=shift_left_bottle.addList();
    pd2.addDouble(shift_left[0]); pd2.addDouble(shift_left[1]);
    pd2.addDouble(shift_left[2]);
    movement_par.put("shift_left",shift_left_bottle.get(0));
    Bottle home_right_bottle;
    Bottle &p2=home_right_bottle.addList();
    p2.addDouble(home_right[0]); p2.addDouble(home_right[1]);
    p2.addDouble(home_right[2]); p2.addDouble(home_right[3]);
    p2.addDouble(home_right[4]); p2.addDouble(home_right[5]);p2.addDouble(home_right[6]);
    movement_par.put("home_right", home_right_bottle.get(0));

    Bottle home_left_bottle;
    Bottle &p2l=home_left_bottle.addList();
    p2l.addDouble(home_left[0]); p2l.addDouble(home_left[1]);
    p2l.addDouble(home_left[2]); p2l.addDouble(home_left[3]);
    p2l.addDouble(home_left[4]); p2l.addDouble(home_left[5]);p2l.addDouble(home_left[6]);
    movement_par.put("home_left", home_left_bottle.get(0));

    Bottle basket_right_bottle;
    Bottle &pk=basket_right_bottle.addList();
    pk.addDouble(basket_right[0]); pk.addDouble(basket_right[1]);
    pk.addDouble(basket_right[2]); pk.addDouble(basket_right[3]);
    pk.addDouble(basket_right[4]); pk.addDouble(basket_right[5]);pk.addDouble(basket_right[6]);
    movement_par.put("basket_right", basket_right_bottle.get(0));

    Bottle basket_left_bottle;
    Bottle &pk2=basket_left_bottle.addList();
    pk2.addDouble(basket_left[0]); pk2.addDouble(basket_left[1]);
    pk2.addDouble(basket_left[2]); pk2.addDouble(basket_left[3]);
    pk2.addDouble(basket_left[4]); pk2.addDouble(basket_left[5]);pk2.addDouble(basket_left[6]);
    movement_par.put("basket_left", basket_left_bottle.get(0));

    Bottle stiff_right_bottle;
    Bottle &ps=stiff_right_bottle.addList();
    ps.addDouble(stiff_right[0]); ps.addDouble(stiff_right[1]);
    ps.addDouble(stiff_right[2]); ps.addDouble(stiff_right[3]);
    ps.addDouble(stiff_right[4]);
    movement_par.put("stiff_right", stiff_right_bottle.get(0));

    Bottle stiff_left_bottle;
    Bottle &ps2=stiff_left_bottle.addList();
    ps2.addDouble(stiff_left[0]); ps2.addDouble(stiff_left[1]);
    ps2.addDouble(stiff_left[2]); ps2.addDouble(stiff_left[3]);
    ps2.addDouble(stiff_left[4]);
    movement_par.put("stiff_left", stiff_left_bottle.get(0));

    Bottle damp_right_bottle;
    Bottle &pdamp=damp_right_bottle.addList();
    pdamp.addDouble(damp_right[0]); pdamp.addDouble(damp_right[1]);
    pdamp.addDouble(damp_right[2]); pdamp.addDouble(damp_right[3]);
    pdamp.addDouble(damp_right[4]);
    movement_par.put("damp_right", damp_right_bottle.get(0));

    Bottle damp_left_bottle;
    Bottle &pdamp2=damp_left_bottle.addList();
    pdamp2.addDouble(damp_left[0]); pdamp2.addDouble(damp_left[1]);
    pdamp2.addDouble(damp_left[2]); pdamp2.addDouble(damp_left[3]);
    pdamp2.addDouble(damp_left[4]);
    movement_par.put("damp_left", damp_left_bottle.get(0));

    executed=true;
    hand_to_move="right";

    yInfo()<<"[GraspExecution] lift_z:        "<<lift_z;
    yInfo()<<"[GraspExecution] force_threshold: "<<force_threshold;
    yInfo()<<"[GraspExecution] shift_right:   "<<shift_right.toString(3,3);
    yInfo()<<"[GraspExecution] shift_left:    "<<shift_left.toString(3,3);
    yInfo()<<"[GraspExecution] home_right:    "<<home_right.toString(3,3);
    yInfo()<<"[GraspExecution] home_left:     "<<home_left.toString(3,3);
    yInfo()<<"[GraspExecution] basket_right:  "<<basket_right.toString(3,3);
    yInfo()<<"[GraspExecution] basket_left:   "<<basket_left.toString(3,3);
    yInfo()<<"[GraspExecution] stiff_right:   "<<stiff_right.toString(3,3);
    yInfo()<<"[GraspExecution] stiff_left:    "<<stiff_left.toString(3,3);
    yInfo()<<"[GraspExecution] damp_right:    "<<damp_right.toString(3,3);
    yInfo()<<"[GraspExecution] damp_left:     "<<damp_left.toString(3,3);

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
    {
        graspVis->stop();
        delete graspVis;
        igaze->stopControl();
        igaze->restoreContext(context_gaze);
        GazeCtrl.close();
    }

    if (portRpc.asPort().isOpen())
        portRpc.close();

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

    if (executed==false && demo==true)
    {
        if (!multiple_superq)
            graspExec->getPoses(complete_sol);
        else
            graspExec->getPoses(solutions[best_scenario]);
        executed_var=executed=graspExec->executeTrajectory(hand_to_move);
    }

    if (save_poses && (graspComp->count_file == graspComp->count_file_old))
    {
        if (solutions.size()==1)
            saveSol(complete_sol, 0);
        else
            saveSolMultiple(complete_sols);
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

    yDebug()<<"Blocking eyes..."<<block_eye;
    igaze->blockEyes(block_eye);
    yDebug()<<"Done: "<<igaze->waitMotionDone(2.0);

    yDebug()<<"Blocking roll..."<<block_neck;
    igaze->blockNeckRoll(block_neck);
    yDebug()<<"Done: "<<igaze->waitMotionDone(2.0);

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

    torso_pitch_max=rf.check("torso_pitch_max", Value(15.0)).asDouble();
    robot=rf.find("robot").asString().c_str();
    if(rf.find("robot").isNull())
        robot="icubSim";

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
        yError()<<"No plane and displacement provided!";
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

    tol_single=rf.check("tol_single", Value(1e-3)).asDouble();
    constr_viol_tol_single=rf.check("constr_tol_single", Value(1e-2)).asDouble();
    tol_multiple=rf.check("tol_multiple", Value(1e-3)).asDouble();
    constr_viol_tol_multiple=rf.check("constr_tol_multiple", Value(1e-2)).asDouble();
    acceptable_iter=rf.check("acceptable_iter", Value(0)).asInt();
    max_iter=rf.check("max_iter", Value(1e8)).asInt();

    mu_strategy=rf.find("mu_strategy").asString().c_str();
    if(rf.find("mu_strategy").isNull())
       mu_strategy="monotone";

    nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
    if(rf.find("nlp_scaling_method").isNull())
       nlp_scaling_method="none";

    ipopt_par.put("max_cpu_time",max_cpu_time);
    ipopt_par.put("tol_single",tol_single);
    ipopt_par.put("constr_tol_single",constr_viol_tol_single);
    ipopt_par.put("tol_multiple",tol_multiple);
    ipopt_par.put("constr_tol_multiple",constr_viol_tol_multiple);
    ipopt_par.put("max_iter",max_iter);
    ipopt_par.put("acceptable_iter",acceptable_iter);
    ipopt_par.put("IPOPT_mu_strategy",mu_strategy);
    ipopt_par.put("IPOPT_nlp_scaling_method",nlp_scaling_method);
    ipopt_par.put("print_level",print_level);

    pose_par.put("n_pointshand",n_pointshand);
    Bottle disp_bottle;
    Bottle &pd=disp_bottle.addList();
    pd.addDouble(displacement[0]); pd.addDouble(displacement[1]);
    pd.addDouble(displacement[2]);
    pose_par.put("hand_displacement",disp_bottle.get(0));
    Bottle plane_bottle;
    Bottle &p2=plane_bottle.addList();
    p2.addDouble(plane[0]); p2.addDouble(plane[1]);
    p2.addDouble(plane[2]); p2.addDouble(plane[3]);
    pose_par.put("plane", plane_bottle.get(0));

    // For selecting the best pose
    pose_par.put("robot", robot);
    pose_par.put("torso_pitch_max", torso_pitch_max);

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

    graspComp= new GraspComputation(ipopt_par, pose_par, traj_par, left_or_right, hand, hand1, this->rf, complete_sol, object,obstacles, cost_right, cost_left, multiple_superq);

    graspComp->init();


    if (visualization)
    {
        config=configViewer(rf);

        if (config==false)
            return false;

        graspVis= new GraspVisualization(rate_vis,eye,igaze,executed_var, K, left_or_right, complete_sols, object_vis,obstacles_vis, hand, hand1, vis_par, cost_vis_r, cost_vis_l, best_scenario);


        bool thread_started=graspVis->start();

        if (thread_started)
            yInfo()<<"[GraspVisualization]: Thread started!";
        else
            yError()<<"[GraspVisualization]: Problems in starting the thread!";
    }
    //else
    //{
    //    graspVis->start();
    //    graspVis->suspend();
    //}

    if (demo)
    {
        configMovements(rf);

        configGrasp(rf);

        graspExec= new GraspExecution(movement_par, complete_sol, grasp, lib_context, lib_filename);

        config=graspExec->configure();

        executed_var=false;
    }

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
void GraspingModule::saveSol(const Property &poses, int scenario)
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

/**********************************************************************/
void GraspingModule::saveSolMultiple(deque<Property> &poses)
{
    for (size_t i=0; i<poses.size(); i++)
    {
        saveSol(poses[i], i);
    }
}

/**********************************************************************/
Property GraspingModule::mergeProperties(string l_o_r)
{
    Property poses;
    Bottle bottle;

    if ((l_o_r=="right") || (l_o_r=="both"))
    {
        Bottle &bottle_right_pose=bottle.addList();
        for (size_t i=0; i<graspComp->poseR.size(); i++)
        {
            bottle_right_pose.addDouble(graspComp->poseR[i]);
        }
        poses.put("pose_right1", bottle.get(0));

        Bottle &bottle_right_sol=bottle.addList();
        for (size_t i=0; i<graspComp->solR.size(); i++)
        {
            bottle_right_sol.addDouble(graspComp->solR[i]);
        }
        poses.put("solution_right1", bottle.get(1));

        Bottle &bottle_right_traj=bottle.addList();
        for (size_t i=0; i<graspComp->trajectory_right.size(); i++)
        {
            Bottle &bb=bottle_right_traj.addList();
            for (size_t j=0; j<graspComp->trajectory_right[i].size();j++)
                bb.addDouble(graspComp->trajectory_right[i][j]);
        }
        poses.put("trajectory_right1", bottle.get(2));
    }

    if (l_o_r=="both")
    {
        Bottle &bleft2=bottle.addList();
        for (size_t i=0; i<graspComp->poseL.size(); i++)
        {
            bleft2.addDouble(graspComp->poseL[i]);
        }
        poses.put("pose_left1", bottle.get(3));

        Bottle &bottle_left_sol=bottle.addList();
        for (size_t i=0; i<graspComp->solL.size(); i++)
        {
            bottle_left_sol.addDouble(graspComp->solL[i]);
        }
        poses.put("solution_left1", bottle.get(4));

        Bottle &bottle_left_traj=bottle.addList();
        for (size_t i=0; i<graspComp->trajectory_left.size(); i++)
        {
            Bottle &bb=bottle_left_traj.addList();
            for (size_t j=0; j<graspComp->trajectory_left[i].size();j++)
                bb.addDouble(graspComp->trajectory_left[i][j]);
        }
        poses.put("trajectory_left1", bottle.get(5));
    }

    if (l_o_r=="left")
    {
        Bottle &bottle_left_pose=bottle.addList();
        for (size_t i=0; i<graspComp->poseL.size(); i++)
        {
            bottle_left_pose.addDouble(graspComp->poseL[i]);
        }
        poses.put("pose_left1", bottle.get(0));

        Bottle &bottle_left_sol=bottle.addList();
        for (size_t i=0; i<graspComp->solL.size(); i++)
        {
            bottle_left_sol.addDouble(graspComp->solL[i]);
        }
        poses.put("solution_left1", bottle.get(1));

        Bottle &bottle_left_traj=bottle.addList();
        for (size_t i=0; i<graspComp->trajectory_left.size(); i++)
        {
            Bottle &bb=bottle_left_traj.addList();
            for (size_t j=0; j<graspComp->trajectory_left[i].size();j++)
                bb.addDouble(graspComp->trajectory_left[i][j]);
        }
        poses.put("trajectory_left1", bottle.get(2));
    }

    if ((l_o_r=="right") || (l_o_r=="both"))
    {
        Bottle &bottle_right_pose=bottle.addList();
        for (size_t i=0; i<graspComp2->poseR.size(); i++)
        {
            bottle_right_pose.addDouble(graspComp2->poseR[i]);
        }
        poses.put("pose_right2", bottle.get(6));

        Bottle &bottle_right_sol=bottle.addList();
        for (size_t i=0; i<graspComp2->solR.size(); i++)
        {
            bottle_right_sol.addDouble(graspComp2->solR[i]);
        }
        poses.put("solution_right2", bottle.get(7));

        Bottle &bottle_right_traj=bottle.addList();
        for (size_t i=0; i<graspComp2->trajectory_right.size(); i++)
        {
            Bottle &bb=bottle_right_traj.addList();
            for (size_t j=0; j<graspComp2->trajectory_right[i].size();j++)
                bb.addDouble(graspComp2->trajectory_right[i][j]);
        }
        poses.put("trajectory_right2", bottle.get(8));
    }

    if (l_o_r=="both")
    {
        Bottle &bottle_left_pose=bottle.addList();
        for (size_t i=0; i<graspComp2->poseL.size(); i++)
        {
            bottle_left_pose.addDouble(graspComp2->poseL[i]);
        }
        poses.put("pose_left2", bottle.get(9));

        Bottle &bottle_left_sol=bottle.addList();
        for (size_t i=0; i<graspComp2->solL.size(); i++)
        {
            bottle_left_sol.addDouble(graspComp2->solL[i]);
        }
        poses.put("solution_left2", bottle.get(10));

        Bottle &bottle_left_traj=bottle.addList();
        for (size_t i=0; i<graspComp2->trajectory_left.size(); i++)
        {
            Bottle &bb=bottle_left_traj.addList();
            for (size_t j=0; j<graspComp2->trajectory_left[i].size();j++)
                bb.addDouble(graspComp2->trajectory_left[i][j]);
        }
        poses.put("trajectory_left2", bottle.get(11));
    }


    if (l_o_r=="left")
    {
        Bottle &bottle_left_pose=bottle.addList();
        for (size_t i=0; i<graspComp2->poseL.size(); i++)
        {
            bottle_left_pose.addDouble(graspComp2->poseL[i]);
        }
        poses.put("pose_left2", bottle.get(3));

        Bottle &bottle_left_sol=bottle.addList();
        for (size_t i=0; i<graspComp2->solL.size(); i++)
        {
            bottle_left_sol.addDouble(graspComp2->solL[i]);
        }
        poses.put("solution_left2", bottle.get(4));

        Bottle &bottle_left_traj=bottle.addList();
        for (size_t i=0; i<graspComp2->trajectory_left.size(); i++)
        {
            Bottle &bb=bottle_left_traj.addList();
            for (size_t j=0; j<graspComp2->trajectory_left[i].size();j++)
                bb.addDouble(graspComp2->trajectory_left[i][j]);
        }
        poses.put("trajectory_left2", bottle.get(5));
    }

    return poses;
}
