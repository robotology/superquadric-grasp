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

#include <cmath>
#include <algorithm>
#include <sstream>
#include <set>
#include <fstream>

#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>


#include "graspExecution.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/*******************************************************************************/
GraspExecution::GraspExecution(Property &_movement_par, const Property &_complete_sol,
                               bool _grasp, string _lib_context, string _lib_filename):
                                movement_par(_movement_par), complete_sol(_complete_sol),
                                grasp(_grasp), lib_context(_lib_context), lib_filename(_lib_filename)

{

}

/*******************************************************************************/
bool GraspExecution::configure()
{
    bool config;

    home_right.resize(7,0.0);
    home_left.resize(7,0.0);
    shift_right.resize(3,0.0);
    shift_left.resize(3,0.0);
    basket_right.resize(7,0.0);
    basket_left.resize(7,0.0);
    stiff_right.resize(5,0.0);
    stiff_left.resize(5,0.0);
    damp_right.resize(5,0.0);
    damp_left.resize(5,0.0);

    portForces_right.open("/superquadric-grasp/forces_right:i");
    portForces_left.open("/superquadric-grasp/forces_left:i");

    portWholeBodyRpc.open("/superquadric-grasp/wb:rpc");

    i=-1;

    setPosePar(movement_par, true);
    config=configTorso();

    if ((left_or_right!="both") && (compliant==true))
    {
        config=config && configCompliant(left_or_right);
    }
    else if (compliant==true)
    {
        config=configCompliant("right");
        config=config && configCompliant("left");
    }


    if (left_or_right!="both")
    {
        config=config && configCartesian(left_or_right);
    }
    else
    {
        config=configCartesian("right");
        config=config && configCartesian("left");
    }

    if (visual_serv)
        config=config && configVisualServoing();

    if (grasp)
    {
        config=config && configGrasp();

        yDebug()<<"Grasped configured: "<<config;
    }

    reached=false;

    return config;
}

/*******************************************************************************/
bool GraspExecution::configCartesian(const string &which_hand)
{
    bool done=true;
    int context_tmp;

    if (which_hand=="right")
    {
        Property option_arm_r("(device cartesiancontrollerclient)");
        option_arm_r.put("remote","/"+robot+"/cartesianController/"+which_hand+"_arm");
        option_arm_r.put("local","/superquadric-grasp/cartesian/"+which_hand+"_arm");

        robotDevice_right.open(option_arm_r);

        if (!robotDevice_right.isValid())
        {
            yError("Device index for right arm not available!");
            return false;
        }

        robotDevice_right.view(icart_right);

        icart_right->storeContext(&context_right);

        Vector curDof;
        icart_right->getDOF(curDof);
        Vector newDof(3);
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        icart_right->setDOF(newDof,curDof);
        icart_right->getDOF(curDof);
        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        icart_right->setTrajTime(traj_time);
        icart_right->setInTargetTol(traj_tol);

        //icart_right->goToPoseSync(home_right.subVector(0,2),home_right.subVector(3,6));
        //icart_right->waitMotionDone();
        //icart_right->checkMotionDone(&done);

        icart_right->goToPoseSync(home_right.subVector(0,2),home_right.subVector(3,6));
        if (compliant)
            icart_right->waitMotionDone(0.1, 2.0);
        else
            icart_right->waitMotionDone();

        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;
        icart_right->setDOF(newDof,curDof);

        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        double min, max;

        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        yDebug()<<"Setting max torso pitch";
        icart_right->setLimits(0, 0.0, torso_pitch_max);
        icart_right->getLimits(0, &min, &max);
        yDebug()<<"Get limit of pitch"<<min<<max;
    }
    else if (which_hand=="left")
    {
        Property option_arm_l("(device cartesiancontrollerclient)");
        option_arm_l.put("remote","/"+robot+"/cartesianController/"+which_hand+"_arm");
        option_arm_l.put("local","/superquadric-grasp/cartesian/"+which_hand+"_arm");

        robotDevice_left.open(option_arm_l);

        if (!robotDevice_left.isValid())
        {
            yError("Device index for left arm not available!");
            return false;
        }

        robotDevice_left.view(icart_left);

        icart_left->storeContext(&context_left);

        Vector curDof;
        icart_left->getDOF(curDof);
        Vector newDof(3);
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        icart_left->setDOF(newDof,curDof);

        icart_left->getDOF(curDof);
        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        icart_left->setTrajTime(traj_time);
        icart_left->setInTargetTol(traj_tol);

        //icart_left->goToPoseSync(home_left.subVector(0,2),home_left.subVector(3,6));
        //icart_left->waitMotionDone();
        //icart_left->checkMotionDone(&done);

        icart_left->goToPoseSync(home_left.subVector(0,2),home_left.subVector(3,6));
        if (compliant)
            icart_left->waitMotionDone(0.1, 2.0);
        else
            icart_left->waitMotionDone();


        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;
        icart_left->setDOF(newDof,curDof);

        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        double min, max;
        yDebug()<<"Setting max torso pitch";
        icart_left->setLimits(0, 0.0, torso_pitch_max);
        icart_left->getLimits(0, &min, &max);
        yDebug()<<"Get limit of pitch"<<min<<max;
    }

    return done;
}

/*******************************************************************************/
bool GraspExecution::configCompliant(const string &which_hand)
{
    if (which_hand=="right")
    {
        Property opt_rightArm("(device remote_controlboard)");
        opt_rightArm.put("remote",("/"+robot+"/right_arm").c_str());
        opt_rightArm.put("local",("/compliantControl/right_arm"));

        if (!driverImped_right.open(opt_rightArm))
        {
            yError()<<"Problem in opening driver for impedance right";
            return false;
        }

        IInteractionMode  *imode_right;
        IImpedanceControl *iimp_right;

        driverImped_right.view(imode_right);
        driverImped_right.view(iimp_right);

        yDebug()<<"stiff r"<<stiff_right.toString();
        yDebug()<<"damp r"<<damp_right.toString();

        for (int j=0; j<5; j++)
        {
            iimp_right->setImpedance(j,stiff_right[j],damp_right[j]);
            imode_right->setInteractionMode(j,VOCAB_IM_COMPLIANT);
        }
    }
    else
    {
        Property opt_leftArm("(device remote_controlboard)");
        opt_leftArm.put("remote",("/"+robot+"/left_arm").c_str());
        opt_leftArm.put("local",("/compliantControl/left_arm"));


        if (!driverImped_left.open(opt_leftArm))
        {
            yError()<<"Problem in opening driver for impedance left";
            return false;
        }

        IInteractionMode  *imode_left;
        IImpedanceControl *iimp_left;

        driverImped_left.view(imode_left);
        driverImped_left.view(iimp_left);

        yDebug()<<"stiff l"<<stiff_left.toString();
        yDebug()<<"damp l"<<damp_left.toString();

        for (int j=0; j<5; j++)
        {
            iimp_left->setImpedance(j,stiff_left[j],damp_left[j]);
            imode_left->setInteractionMode(j,VOCAB_IM_COMPLIANT);
        }
    }
}

/*******************************************************************************/
bool GraspExecution::configVisualServoing()
{
    Property prop_server_vs;
    prop_server_vs.put("device","visualservoingclient");
    prop_server_vs.put("verbosity",true);
    prop_server_vs.put("local","/VisualServoingClientTest");
    prop_server_vs.put("remote", "/visualservoing");

    drv_server_vs.open(prop_server_vs);
    if (!drv_server_vs.isValid())
    {
       yError("Could not run VisualServoingClient!");
       return false;
    }

    drv_server_vs.view(visual_servoing_right);
    if (visual_servoing_right == NULL)
    {
       yError("Could not get interfacate to VisualServoingClient!");
       return false;
    }

    visual_servoing_right->setGoToGoalTolerance(pixel_tol);

    return true;
}

/*******************************************************************************/
bool GraspExecution::configGrasp()
{
    if (left_or_right=="right")
    {
        handContr_right.set(lib_context, lib_filename);
        handContr_right.set("hand", Value("right"));
        handContr_right.openHand(true, true);
        handContr_right.set("useRingLittleFingers", Value(five_fingers));
    }
    else if (left_or_right=="left")
    {
        handContr_left.set(lib_context, lib_filename);
        handContr_left.set("hand", Value("left"));
        handContr_left.openHand(true, true);
        handContr_left.set("useRingLittleFingers", Value(five_fingers));
    }
    else if (left_or_right=="both")
    {
        handContr_right.set(lib_context, lib_filename);
        handContr_left.set(lib_context, lib_filename);

        handContr_right.set("hand", Value("right"));
        handContr_right.set("useRingLittleFingers", Value(five_fingers));

        handContr_left.set("hand", Value("left"));
        handContr_left.set("useRingLittleFingers", Value(five_fingers));

        handContr_right.openHand(true, true);
        handContr_left.openHand(true, true);
    }

    if ((left_or_right !="left") && (!handContr_right.open()))
    {
        yError()<<"[GraspExecution]: Problems in initializing the tactile control for right arm";
        return false;
    }

    if ((left_or_right!="right") && (!handContr_left.open()))
    {
        yError()<<"[GraspExecution]: Problems in initializing the tactile control for left arm";
        return false;
    }

    return true;
}

/*******************************************************************************/
bool GraspExecution::configTorso()
{
    Property option;
    option.put("device","remote_controlboard");
    option.put("remote","/"+robot+"/torso");
    option.put("local","/controllerTorso");

    // open the driver
    if (!driverTorso.open(option))
    {
        yError()<<"Unable to open the device driver";
        return false;
    }

    // open the views
    driverTorso.view(imodTorso);
    driverTorso.view(iencTorso);
    driverTorso.view(iposTorso);

    // tell the device we aim to control
    // in position mode all the joints
    int nAxes;
    iencTorso->getAxes(&nAxes);
    vector<int> modes(nAxes,VOCAB_CM_POSITION);
    imodTorso->setControlModes(modes.data());

    //Putting torso in home position
    iposTorso->positionMove(0, 0.0);
    iposTorso->positionMove(1, 0.0);
    iposTorso->positionMove(2, 1.0);

    return true;
}

/*******************************************************************************/
void GraspExecution::setPosePar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);

    string rob=newOptions.find("robot").asString();

    if (newOptions.find("robot").isNull() && (first_time==true))
    {
        robot="icubSim";
    }
    else if (!newOptions.find("robot").isNull())
    {
        if ((rob=="icub") || (rob=="icubSim"))
        {
            robot=rob;
        }
        else
        {
            robot="icubSim";
        }
    }

    string han=newOptions.find("hand").asString();

    if (newOptions.find("hand").isNull() && (first_time==true))
    {
        left_or_right="both";
    }
    else if (!newOptions.find("hand").isNull())
    {
        if ((han=="right") || (han=="left") || han=="both")
        {
            left_or_right=han;
        }
        else
        {
            left_or_right="both";
        }
    }

    string lobj=newOptions.find("lift_object").asString();

    if (newOptions.find("lift_object").isNull() && (first_time==true))
    {
        lift_object=false;
    }
    else if (!newOptions.find("lift_object").isNull())
    {
        if (lobj=="on")
            lift_object=true;
        else
            lift_object=false;
    }

    string fivef=newOptions.find("five_fingers").asString();

    if (newOptions.find("five_fingers").isNull() && (first_time==true))
    {
        five_fingers="false";
    }
    else if (!newOptions.find("five_fingers").isNull())
    {
        if (fivef=="on")
            five_fingers="true";
        else
            five_fingers="false";

        if ((left_or_right=="right") || (left_or_right=="both"))
        {
            handContr_right.set("useRingLittleFingers", Value(five_fingers));
        }
        else if ((left_or_right=="left") || ("left_or_right"=="both"))
        {
            handContr_left.set("useRingLittleFingers", Value(five_fingers));
        }
    }

    string vs=newOptions.find("visual_servoing").asString();

    if (newOptions.find("visual_servoing").isNull() && (first_time==true))
    {
        visual_serv=false;
    }
    else if (!newOptions.find("visual_servoing").isNull())
    {
        if (vs=="on")
            visual_serv=true;
        else
            visual_serv=false;
    }

    string cm=newOptions.find("compliant").asString();

    if (newOptions.find("compliant").isNull() && (first_time==true))
    {
        compliant=false;
    }
    else if (!newOptions.find("compliant").isNull())
    {
        if (cm=="on")
            compliant=true;
        else
            compliant=false;
    }

    string direct_kin=newOptions.find("use_direct_kin").asString();

    if (newOptions.find("use_direct_kin").isNull() && (first_time==true))
    {
        use_direct_kin=false;
    }
    else if (!newOptions.find("use_direct_kin").isNull())
    {
        if (direct_kin=="on")
            use_direct_kin=true;
        else
            use_direct_kin=false;
    }

    double ttime=newOptions.find("traj_time").asDouble();

    if (newOptions.find("traj_time").isNull() && (first_time==true))
    {
        traj_time=2.0;
    }
    else if (!newOptions.find("traj_time").isNull())
    {
        if ((ttime>=0.5) && (ttime<=5.0))
        {
            traj_time=ttime;
        }
        else if (ttime<0.5)
        {
            traj_time=0.5;
        }
        else if (ttime>5.0)
        {
            traj_time=5.0;
        }

        if (first_time==false)
        {
            if ((left_or_right=="right") || (left_or_right=="both"))
                icart_right->setTrajTime(traj_time);
            else if ((left_or_right=="left") || (left_or_right=="both"))
                icart_left->setTrajTime(traj_time);
        }
    }

    double ttol=newOptions.find("traj_tol").asDouble();

    if (newOptions.find("traj_tol").isNull() && (first_time==true))
    {
        traj_tol=0.005;
    }
    else if (!newOptions.find("traj_tol").isNull())
    {
        if ((ttol>=0.001) && (ttol<=0.05))
        {
            traj_tol=ttol;
        }
        else if (ttol<0.001)
        {
            traj_tol=0.001;
        }
        else if (ttol>0.05)
        {
            traj_tol=0.05;
        }

        if (first_time==false)
        {
            if ((left_or_right=="right") || (left_or_right=="both"))
                icart_right->setInTargetTol(traj_tol);
            else if ((left_or_right=="left") || (left_or_right=="both"))
                icart_left->setInTargetTol(traj_tol);
        }
    }

    double ptol=newOptions.find("pixel_tol").asDouble();

    if (newOptions.find("pixel_tol").isNull() && (first_time==true))
    {
        pixel_tol=15.0;
    }
    else if (!newOptions.find("pixel_tol").isNull())
    {
        if ((ptol>=1.0) && (ptol<=20.0))
        {
            pixel_tol=ptol;
        }
        else if (ptol<1.0)
        {
            pixel_tol=1.0;
        }
        else if (ptol>20.0)
        {
            pixel_tol=20.0;
        }

        if (first_time==false)
        {
            visual_servoing_right->setGoToGoalTolerance(pixel_tol);
        }
    }

    double lz=newOptions.find("lift_z").asDouble();

    if (newOptions.find("lift_z").isNull() && (first_time==true))
    {
        lift_z=0.15;
    }
    else if (!newOptions.find("lift_z").isNull())
    {
        if ((lz>=0.05) && (lz<=0.3))
        {
            lift_z=lz;
        }
        else if (lz<0.05)
        {
            lift_z=0.05;
        }
        else if (lz>0.3)
        {
            lift_z=0.3;
        }
    }

    double fthres=newOptions.find("force_threshold").asDouble();

    if (newOptions.find("force_threshold").isNull() && (first_time==true))
    {
        force_threshold=6.0;
    }
    else if (!newOptions.find("force_threshold").isNull())
    {
        if ((fthres>=1.5) && (fthres<=10.0))
        {
            force_threshold=fthres;
        }
        else if (fthres<1.5)
        {
            force_threshold=1.5;
        }
        else if (fthres>10.0)
        {
            force_threshold=10.0;
        }

    }

    Bottle *sh=newOptions.find("shift_right").asList();
    if (newOptions.find("shift_right").isNull() && (first_time==true))
    {
        shift_right[0]=0.0;
        shift_right[1]=0.0;
        shift_right[2]=0.0;
    }
    else if (!newOptions.find("shift_right").isNull())
    {
        Vector tmp(3,0.0);
        tmp[0]=sh->get(0).asDouble();
        tmp[1]=sh->get(1).asDouble();
        tmp[2]=sh->get(2).asDouble();

        if (norm(tmp)>0.0)
        {
            shift_right=tmp;
        }
        else
        {
            shift_right[0]=0.0;
            shift_right[1]=0.0;
            shift_right[2]=0.0;
        }
    }

    Bottle *sh2=newOptions.find("shift_left").asList();
    if (newOptions.find("shift_left").isNull() && (first_time==true))
    {
        shift_left[0]=0.0;
        shift_left[1]=0.0;
        shift_left[2]=0.0;
    }
    else if (!newOptions.find("shift_left").isNull())
    {
        Vector tmp(3,0.0);
        tmp[0]=sh2->get(0).asDouble();
        tmp[1]=sh2->get(1).asDouble();
        tmp[2]=sh2->get(2).asDouble();

        if (norm(tmp)>0.0)
        {
            shift_left=tmp;
        }
        else
        {
            shift_left[0]=0.0;
            shift_left[1]=0.0;
            shift_left[2]=0.0;
        }
}

    Bottle *pl=newOptions.find("home_right").asList();
    if (newOptions.find("home_right").isNull() && (first_time==true))
    {
        home_right[0]=-0.30; home_right[1]=0.21; home_right[2]=0.15;
        home_right[3]=0.113261; home_right[4]=-0.954747; home_right[5]=0.275008; home_right[6]=2.868312;
    }
    else if (!newOptions.find("home_right").isNull())
    {
        Vector tmp(7,0.0);
        tmp[0]=pl->get(0).asDouble();
        tmp[1]=pl->get(1).asDouble();
        tmp[2]=pl->get(2).asDouble();
        tmp[3]=pl->get(3).asDouble();
        tmp[4]=pl->get(4).asDouble();
        tmp[5]=pl->get(5).asDouble();
        tmp[6]=pl->get(6).asDouble();

        if (norm(tmp)>0.0)
        {
            home_right=tmp;
        }
        else
        {
            home_right[0]=-0.30; home_right[1]=0.21; home_right[2]=0.15;
            home_right[3]=0.113261; home_right[4]=-0.954747; home_right[5]=0.275008; home_right[6]=2.868312;
        }
    }

    Bottle *pll=newOptions.find("home_left").asList();
    if (newOptions.find("home_left").isNull() && (first_time==true))
    {
        home_left[0]=-0.30; home_left[1]=-0.21; home_left[2]=0.15;
        home_left[3]=0.13631; home_left[4]=-0.30149; home_left[5]=0.943675; home_left[6]=2.865295;
    }
    else if (!newOptions.find("home_left").isNull())
    {
        Vector tmp(7,0.0);
        tmp[0]=pll->get(0).asDouble();
        tmp[1]=pll->get(1).asDouble();
        tmp[2]=pll->get(2).asDouble();
        tmp[3]=pll->get(3).asDouble();
        tmp[4]=pll->get(4).asDouble();
        tmp[5]=pll->get(5).asDouble();
        tmp[6]=pll->get(6).asDouble();
        if (norm(tmp)>0.0)
        {
            home_left=tmp;
        }
        else
        {
            home_left[0]=-0.30; home_left[1]=-0.21; home_left[2]=0.15;
            home_left[3]=0.13631; home_left[4]=-0.30149; home_left[5]=0.943675; home_left[6]=2.865295;
        }
    }

    Bottle *bask_r=newOptions.find("basket_right").asList();
    if (newOptions.find("basket_right").isNull() && (first_time==true))
    {
        basket_right[0]=-0.30; basket_right[1]=0.21; basket_right[2]=0.15;
        basket_right[3]=0.113261; basket_right[4]=-0.954747; basket_right[5]=0.275008; basket_right[6]=2.868312;
    }
    else if (!newOptions.find("basket_right").isNull())
    {
        Vector tmp(7,0.0);
        tmp[0]=bask_r->get(0).asDouble();
        tmp[1]=bask_r->get(1).asDouble();
        tmp[2]=bask_r->get(2).asDouble();
        tmp[3]=bask_r->get(3).asDouble();
        tmp[4]=bask_r->get(4).asDouble();
        tmp[5]=bask_r->get(5).asDouble();
        tmp[6]=bask_r->get(6).asDouble();

        if (norm(tmp)>0.0)
        {
            basket_right=tmp;
        }
        else
        {
            basket_right[0]=-0.30; basket_right[1]=0.21; basket_right[2]=0.15;
            basket_right[3]=0.113261; basket_right[4]=-0.954747; basket_right[5]=0.275008; basket_right[6]=2.868312;
        }
    }

    Bottle *bask_l=newOptions.find("basket_left").asList();
    if (newOptions.find("basket_left").isNull() && (first_time==true))
    {
        basket_left[0]=-0.30; basket_left[1]=0.21; basket_left[2]=0.15;
        basket_left[3]=0.113261; basket_left[4]=-0.954747; basket_left[5]=0.275008; basket_left[6]=2.868312;
    }
    else if (!newOptions.find("basket_left").isNull())
    {
        Vector tmp(7,0.0);
        tmp[0]=bask_l->get(0).asDouble();
        tmp[1]=bask_l->get(1).asDouble();
        tmp[2]=bask_l->get(2).asDouble();
        tmp[3]=bask_l->get(3).asDouble();
        tmp[4]=bask_l->get(4).asDouble();
        tmp[5]=bask_l->get(5).asDouble();
        tmp[6]=bask_l->get(6).asDouble();

        if (norm(tmp)>0.0)
        {
            basket_left=tmp;
        }
        else
        {
            basket_left[0]=-0.30; basket_left[1]=0.21; basket_left[2]=0.15;
            basket_left[3]=0.113261; basket_left[4]=-0.954747; basket_left[5]=0.275008; basket_left[6]=2.868312;
        }
    }

    Bottle *stiff_r=newOptions.find("stiff_right").asList();
    if (newOptions.find("stiff_right").isNull() && (first_time==true))
    {
        stiff_right[0]=0.4; stiff_right[1]=0.4; stiff_right[2]=0.4;
        stiff_right[3]=0.2; stiff_right[4]=0.2;
    }
    else if (!newOptions.find("stiff_right").isNull())
    {
        Vector tmp(5,0.0);
        tmp[0]=stiff_r->get(0).asDouble();
        tmp[1]=stiff_r->get(1).asDouble();
        tmp[2]=stiff_r->get(2).asDouble();
        tmp[3]=stiff_r->get(3).asDouble();
        tmp[4]=stiff_r->get(4).asDouble();

        if (norm(tmp)>0.0)
        {
            stiff_right=tmp;
        }
        else
        {
            stiff_right[0]=0.4; stiff_right[1]=0.4; stiff_right[2]=0.4;
            stiff_right[3]=0.2; stiff_right[4]=0.2;
        }
    }

    Bottle *stiff_l=newOptions.find("stiff_left").asList();
    if (newOptions.find("stiff_left").isNull() && (first_time==true))
    {
        stiff_left[0]=0.4; stiff_left[1]=0.4; stiff_left[2]=0.4;
        stiff_left[3]=0.2; stiff_left[4]=0.2;
    }
    else if (!newOptions.find("stiff_left").isNull())
    {
        Vector tmp(5,0.0);
        tmp[0]=stiff_l->get(0).asDouble();
        tmp[1]=stiff_l->get(1).asDouble();
        tmp[2]=stiff_l->get(2).asDouble();
        tmp[3]=stiff_l->get(3).asDouble();
        tmp[4]=stiff_l->get(4).asDouble();

        if (norm(tmp)>0.0)
        {
            stiff_left=tmp;
        }
        else
        {
            stiff_left[0]=0.4; stiff_left[1]=0.4; stiff_left[2]=0.4;
            stiff_left[3]=0.2; stiff_left[4]=0.2;
        }
    }

    Bottle *damp_r=newOptions.find("damp_right").asList();
    if (newOptions.find("damp_right").isNull() && (first_time==true))
    {
        damp_right[0]=0.002; damp_right[1]=0.002; damp_right[2]=0.002;
        damp_right[3]=0.002; damp_right[4]=0.0;
    }
    else if (!newOptions.find("damp_right").isNull())
    {
        Vector tmp(5,0.0);
        tmp[0]=damp_r->get(0).asDouble();
        tmp[1]=damp_r->get(1).asDouble();
        tmp[2]=damp_r->get(2).asDouble();
        tmp[3]=damp_r->get(3).asDouble();
        tmp[4]=damp_r->get(4).asDouble();

        if (norm(tmp)>0.0)
        {
            damp_right=tmp;
        }
        else
        {
            damp_right[0]=0.002; damp_right[1]=0.002; damp_right[2]=0.002;
            damp_right[3]=0.002; damp_right[4]=0.0;
        }
    }

    Bottle *damp_l=newOptions.find("damp_left").asList();
    if (newOptions.find("damp_left").isNull() && (first_time==true))
    {
        damp_left[0]=0.002; damp_left[1]=0.002; damp_left[2]=0.002;
        damp_left[3]=0.002; damp_left[4]=0.0;
    }
    else if (!newOptions.find("damp_left").isNull())
    {
        Vector tmp(5,0.0);
        tmp[0]=damp_l->get(0).asDouble();
        tmp[1]=damp_l->get(1).asDouble();
        tmp[2]=damp_l->get(2).asDouble();
        tmp[3]=damp_l->get(3).asDouble();
        tmp[4]=damp_l->get(4).asDouble();

        if (norm(tmp)>0.0)
        {
            damp_left=tmp;
        }
        else
        {
            damp_left[0]=0.002; damp_left[1]=0.002; damp_left[2]=0.002;
            damp_left[3]=0.002; damp_left[4]=0.0;
        }
    }


    double pitch_max=newOptions.find("torso_pitch_max").asDouble();

    if (newOptions.find("torso_pitch_max").isNull() && (first_time==true))
    {
        torso_pitch_max=15.0;
    }
    else if (!newOptions.find("torso_pitch_max").isNull())
    {
        if ((pitch_max>=0.0) && (pitch_max<=40.0))
        {
            torso_pitch_max=pitch_max;
        }
        else if (pitch_max<0.0)
        {
            torso_pitch_max=0.0;
        }
        else if (pitch_max>40.0)
        {
            torso_pitch_max=15.0;
        }
    }

    yDebug()<<"In execution module ....";
    yInfo()<<"[GraspExecution] lift_z:          "<<lift_z;
    yInfo()<<"[GraspExecution] force_threshold: "<<force_threshold;
    yInfo()<<"[GraspExecution] shift_right:     "<<shift_right.toString(3,3);
    yInfo()<<"[GraspExecution] shift_left:      "<<shift_left.toString(3,3);
    yInfo()<<"[GraspExecution] home_right:      "<<home_right.toString(3,3);
    yInfo()<<"[GraspExecution] home_left:       "<<home_left.toString(3,3);
    yInfo()<<"[GraspExecution] basket_right:    "<<basket_right.toString(3,3);
    yInfo()<<"[GraspExecution] basket_left:     "<<basket_left.toString(3,3);
    yInfo()<<"[GraspExecution] stiff_right:     "<<stiff_right.toString(3,3);
    yInfo()<<"[GraspExecution] stiff_left:      "<<stiff_left.toString(3,3);
    yInfo()<<"[GraspExecution] damp_right:      "<<damp_right.toString(3,3);
    yInfo()<<"[GraspExecution] damp_left:       "<<damp_left.toString(3,3);
}

/*******************************************************************************/
Property GraspExecution::getPosePar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("robot",robot);
    advOptions.put("hand",left_or_right);
    advOptions.put("five_fingers",five_fingers);
    if (lift_object)
        advOptions.put("lift_object","on");
    else
        advOptions.put("lift_object","off");

    if (visual_serv)
        advOptions.put("visual_servoing","on");
    else
        advOptions.put("visual_servoing","off");
    if (use_direct_kin)
        advOptions.put("use_direct_kin","on");
    else
        advOptions.put("use_direct_kin","off");
    if (compliant)
        advOptions.put("compliant","on");
    else
        advOptions.put("compliant","off");
    advOptions.put("traj_time",traj_time);
    advOptions.put("pixel_tol",pixel_tol);
    advOptions.put("traj_tol",traj_tol);
    advOptions.put("lift_z",lift_z);
    Bottle s;
    Bottle &pd=s.addList();
    pd.addDouble(shift_right[0]); pd.addDouble(shift_right[1]);
    pd.addDouble(shift_right[2]);
    advOptions.put("shift_right",s.get(0));
    Bottle &pd2=s.addList();
    pd2.addDouble(shift_left[0]); pd2.addDouble(shift_left[1]);
    pd2.addDouble(shift_left[2]);
    advOptions.put("shift_left",s.get(0));

    Bottle planeb;
    Bottle &p2=planeb.addList();
    p2.addDouble(home_right[0]); p2.addDouble(home_right[1]);
    p2.addDouble(home_right[2]); p2.addDouble(home_right[3]);
    p2.addDouble(home_right[4]); p2.addDouble(home_right[5]); p2.addDouble(home_right[6]);
    advOptions.put("home_right", planeb.get(0));

    Bottle planel;
    Bottle &p2l=planel.addList();
    p2l.addDouble(home_left[0]); p2l.addDouble(home_left[1]);
    p2l.addDouble(home_left[2]); p2l.addDouble(home_left[3]);
    p2l.addDouble(home_left[4]); p2l.addDouble(home_left[5]); p2l.addDouble(home_left[6]);
    advOptions.put("home_left", planel.get(0));

    Bottle planebask_r;
    Bottle &pk_r=planebask_r.addList();
    pk_r.addDouble(basket_right[0]); pk_r.addDouble(basket_right[1]);
    pk_r.addDouble(basket_right[2]); pk_r.addDouble(basket_right[3]);
    pk_r.addDouble(basket_right[4]); pk_r.addDouble(basket_right[5]);pk_r.addDouble(basket_right[6]);
    movement_par.put("basket_right", planebask_r.get(0));

    Bottle planebask_l;
    Bottle &pk2_l=planebask_l.addList();
    pk2_l.addDouble(basket_left[0]); pk2_l.addDouble(basket_left[1]);
    pk2_l.addDouble(basket_left[2]); pk2_l.addDouble(basket_left[3]);
    pk2_l.addDouble(basket_left[4]); pk2_l.addDouble(basket_left[5]);pk2_l.addDouble(basket_left[6]);
    movement_par.put("basket_left", planebask_l.get(0));

    Bottle planestiff_r;
    Bottle &pkr=planestiff_r.addList();
    pkr.addDouble(stiff_right[0]); pkr.addDouble(stiff_right[1]);
    pkr.addDouble(stiff_right[2]); pkr.addDouble(stiff_right[3]);
    pkr.addDouble(stiff_right[4]);
    movement_par.put("stiff_right", planestiff_r.get(0));

    Bottle planestiff_l;
    Bottle &pk2l=planestiff_l.addList();
    pk2l.addDouble(stiff_left[0]); pk2l.addDouble(stiff_left[1]);
    pk2l.addDouble(stiff_left[2]); pk2l.addDouble(stiff_left[3]);
    pk2l.addDouble(stiff_left[4]);
    movement_par.put("stiff_left", planestiff_l.get(0));

    Bottle planedamp_r;
    Bottle &pkl=planedamp_r.addList();
    pkl.addDouble(damp_right[0]); pkl.addDouble(damp_right[1]);
    pkl.addDouble(damp_right[2]); pkl.addDouble(damp_right[3]);
    pkl.addDouble(damp_right[4]);
    movement_par.put("damp_right", planedamp_r.get(0));

    Bottle planedamp_l;
    Bottle &pk2=planedamp_l.addList();
    pk2.addDouble(damp_left[0]); pk2.addDouble(damp_left[1]);
    pk2.addDouble(damp_left[2]); pk2.addDouble(damp_left[3]);
    pk2.addDouble(damp_left[4]);
    movement_par.put("damp_left", planedamp_l.get(0));

    return advOptions;
}

/*******************************************************************************/
void GraspExecution::getPoses(const Property &poses)
{
    LockGuard lg(mutex);
    Vector tmp(6,0.0);
    trajectory_right.clear();
    trajectory_left.clear();

    Bottle &pose2=poses.findGroup("trajectory_right");

    if (!pose2.isNull())
    {
        Bottle *p=pose2.get(1).asList();
        for (size_t i=0; i<p->size(); i++)
        {
            Bottle *p1=p->get(i).asList();


            for (size_t j=0; j<p1->size(); j++)
            {
                tmp[j]=p1->get(j).asDouble();
            }

            trajectory_right.push_back(tmp);
        }

        if (norm(shift_right)>0.0)
        {
            for (size_t k=0; k<trajectory_right.size(); k++)
            {
                yDebug()<<"[GraspExecution]: Waypoint right"<<k<<trajectory_right[k].toString(3,3);

                trajectory_right[k].setSubvector(0,trajectory_right[k].subVector(0,2) +shift_right);
                yDebug()<<"[GraspExecution]: Shifted waypoint right"<<k<<trajectory_right[k].toString(3,3);
            }
        }
    }

    Bottle &pose3=poses.findGroup("trajectory_left");

    if (!pose3.isNull())
    {
        Bottle *p=pose3.get(1).asList();
        for (size_t i=0; i<p->size(); i++)
        {
            Bottle *p1=p->get(i).asList();


            for (size_t j=0; j<p1->size(); j++)
            {
                tmp[j]=p1->get(j).asDouble();
            }
            trajectory_left.push_back(tmp);
        }

        if (norm(shift_left)>0.0)
        {
            yDebug()<<"shift left "<<shift_left.toString(3,3);
            for (size_t k=0; k<trajectory_left.size(); k++)
            {
                yDebug()<<"[GraspExecution]: Waypoint left"<<k<<trajectory_left[k].toString(3,3);
                trajectory_left[k].setSubvector(0,trajectory_left[k].subVector(0,2) +shift_left);
                yDebug()<<"[GraspExecution]: Shifted waypoint left"<<k<<trajectory_left[k].toString(3,3);
            }
        }
    }
}

/*******************************************************************************/
bool GraspExecution::calibrateWholeBody()
{
    Bottle cmd, reply;
    cmd.addString("calib");
    cmd.addString("all");

    portWholeBodyRpc.write(cmd, reply);

    yDebug()<<reply.get(0).asString();

    return (reply.get(0).asString()=="Recalibrated");
}

/*******************************************************************************/
bool GraspExecution::executeTrajectory(string &hand)
{
    trajectory.clear();
    if (trajectory_right.size()>0 || trajectory_left.size()>0)
    {
        if (hand=="right")
        {
            for (size_t i=0; i<trajectory_right.size(); i++)
            {
                trajectory.push_back(trajectory_right[i]);
            }

            if (lift_object==true)
                liftObject(trajectory, trajectory_right.size());
        }
        else
        {
            for (size_t i=0; i<trajectory_left.size(); i++)
            {
                trajectory.push_back(trajectory_left[i]);
            }

            if (lift_object==true)
                liftObject(trajectory, trajectory_left.size());
        }

        yDebug()<<"[GraspExecution]: Complete trajectory ";
        for (size_t k=0; k<trajectory.size(); k++)
        {
            yDebug()<<"[GraspExecution]: Waypoint "<<k<<trajectory[k].toString(3,3);
        }

        if (i==-1)
        {
            reached=false;
            reached_tot=false;
            i++;
        }

        if ((reached==false) && (i<=trajectory.size()) && (i>=0))
        {
            yDebug()<<"[GraspExecution]: Waypoint: "<<i<<" : "<<trajectory[i].toString(3,3);

            if ((visual_serv==false) || (hand=="left") || (i != trajectory_right.size()-1))
                reached=reachWaypoint(i, hand);
            else if ((visual_serv==true) && (hand=="right") && (i == trajectory_right.size()-1))
                reached=reachWithVisual(i,hand);

            if (grasp==true && reached==true)
            {
                if (((i==trajectory_right.size()-1) && (hand=="right")) || ((i==trajectory_left.size()-1) && (hand=="left")))
                    reached=graspObject(hand);

                if (((i==trajectory_right.size()+1) && (hand=="right")) || ((i==trajectory_left.size()+1) && (hand=="left")))
                    reached=releaseObject(hand);
            }
        }

        if (reached==true)
        {
            i++;
        }

        if ((i==trajectory.size()) && (reached==true))
            reached_tot=true;

        if (reached_tot==true)
            i=-1;

        reached=false;

        return reached_tot;
    }
    else
    {
        yError()<<"[GraspExecution]: No trajectory available!";
        return true;
    }
}

/*******************************************************************************/
bool GraspExecution::reachWaypoint(int i, string &hand)
{
    bool done=false;
    int context_tmp;
    Bottle *force;
    Vector forceThre(3,0.0);

    //done_force=false;

    double min, max;

    Vector x(3,0.0);
    Vector o(4,0.0);

    Vector newDof, curDof;
    newDof.resize(3,1);
    newDof[1]=0;

    Vector limit_min(10,0.0);
    Vector limit_max(10,0.0);

    x=trajectory[i].subVector(0,2);
    if (trajectory[i].size()==6)
        o=(dcm2axis(euler2dcm(trajectory[i].subVector(3,5))));
    else
        o=trajectory[i].subVector(3,6);

    if (i == 0)
            yInfo()<<"Whole body calibration completed "<<calibrateWholeBody();

    if (hand=="right")
    {
        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        if (i==0)
        {
            yDebug()<<"[GraspExecution]: opening hand ... ";
            handContr_right.openHand(true, true);
        }

        icart_right->goToPoseSync(x,o);

        if ((i == trajectory_right.size()-1) || (i == trajectory_right.size()-2))
        {
            force=portForces_right.read(false);

            yDebug()<<"1";
            while (!done)
            {
                 yDebug()<<"2";
                force=portForces_right.read(false);
                if (force!=NULL)
                {
                    yInfo()<<"Forces of right arm detected while moving     "<<force->toString();
                    forceThre[0]=force->get(0).asDouble();  forceThre[1]=force->get(1).asDouble();  forceThre[2]=force->get(2).asDouble();
                    yDebug()<<"forces 3 "<<forceThre.toString();
                    yDebug()<<"Norm forces "<<norm(forceThre);

                    if (norm(forceThre)>=force_threshold)
                    {
                         return true;
                    }

                }
                else
                    yDebug()<<"No forces received";

                Time::delay(0.01);

                icart_right->checkMotionDone(&done);

            }
        }
        else
        {

            if (compliant)
            {
                icart_right->waitMotionDone(0.1, 2.0);
                done=true;
            }
            else
            {
                icart_right->waitMotionDone();
                icart_right->checkMotionDone(&done);
            }
        }

        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_right->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(x-x_reached)<<" and in orientation: "<<norm(o-o_reached);
        }
    }
    if (hand=="left")
    {
        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        if (i==0)
        {
            yDebug()<<"[GraspExecution]: opening hand ... ";
            handContr_left.openHand(true, true);
        }

        icart_left->goToPoseSync(x,o);

        if ((i == trajectory_right.size()-1) || (i == trajectory_right.size()-2))
        {
            force=portForces_left.read(false);
            while (!done)
            {
                force=portForces_left.read(false);
                if (force!=NULL)
                {
                    yInfo()<<"Forces of right arm detected while moving     "<<force->toString();
                    forceThre[0]=force->get(0).asDouble();  forceThre[1]=force->get(1).asDouble();  forceThre[2]=force->get(2).asDouble();
                    yDebug()<<"forces 3 "<<forceThre.toString();
                    yDebug()<<"Norm forces "<<norm(forceThre);

                    if (norm(forceThre)>=force_threshold)
                    {
                         return true;
                    }

                }
                else
                    yDebug()<<"No forces received";

                Time::delay(0.01);

                icart_left->checkMotionDone(&done);

            }
        }
        else
        {

            if (compliant)
            {
                icart_left->waitMotionDone(0.1, 2.0);
                done=true;
            }
            else
            {
                icart_left->waitMotionDone();
                icart_left->checkMotionDone(&done);
            }
        }

        if (done)
        {
            Vector x_reached(3,0.0);
            Vector o_reached(4,0.0);
            icart_left->getPose(x_reached, o_reached);
            yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(x-x_reached)<<" and in orientation: "<<norm(o-o_reached);
        }
    }

    return done;
}

/*******************************************************************************/
bool GraspExecution::reachWithVisual(int i, string &hand)
{
    Vector x(3,0.0);
    Vector o(4,0.0);
    bool done=false;
    Bottle *force;
    Vector forceThre(3,0.0);

    x=trajectory[i].subVector(0,2);
    if (trajectory[i].size()==6)
        o=(dcm2axis(euler2dcm(trajectory[i].subVector(3,5))));
    else
        o=trajectory[i].subVector(3,6);

    visual_servoing_right->initFacilities(use_direct_kin);

    visual_servoing_right->goToGoal(x,o);

    while (!done)
    {
        done=!visual_servoing_right->checkVisualServoingController();

        force=portForces_right.read(false);
        if (force!=NULL)
        {
            yInfo()<<"Forces of right arm detected while moving     "<<force->toString();
            forceThre[0]=force->get(0).asDouble();  forceThre[1]=force->get(1).asDouble();  forceThre[2]=force->get(2).asDouble();
            yDebug()<<"forces 3 "<<forceThre.toString();
            yDebug()<<"Norm forces "<<norm(forceThre);

            if (norm(forceThre)>=force_threshold)
            {
                 visual_servoing_right->stopController();
                 done=true;
                 return true;
            }

        }
        else
            yDebug()<<"No forces received";

        Time::delay(0.01);
    }

    visual_servoing_right->stopFacilities();

    return done;
}

/*******************************************************************************/
bool GraspExecution::release()
{
    if (hand_to_move=="right" && left_or_right !="left")
        icart_right->stopControl();
    else if (hand_to_move=="left" && left_or_right !="right")
        icart_left->stopControl();

    driverTorso.close();

    if (left_or_right=="both" || left_or_right=="right")
    {
        icart_right->restoreContext(context_right);
        robotDevice_right.close();

        if (compliant)
        {
            IInteractionMode  *imode_right;
            IImpedanceControl *iimp_right;

            driverImped_right.view(imode_right);
            driverImped_right.view(iimp_right);

            for (int j=0; j<5; j++)
            {
                imode_right->setInteractionMode(j,VOCAB_IM_STIFF);
            }
        }
    }
    if (left_or_right=="both" || left_or_right=="left")
    {
        icart_left->restoreContext(context_left);
        robotDevice_left.close();

        if (compliant)
        {
            IInteractionMode  *imode_left;
            IImpedanceControl *iimp_left;

            driverImped_left.view(imode_left);
            driverImped_left.view(iimp_left);

            for (int j=0; j<5; j++)
            {
                imode_left->setInteractionMode(j,VOCAB_IM_STIFF);
            }
        }
    }

    if (visual_serv==true)
    {
        drv_server_vs.close();
    }

    if (grasp==true)
    {
        if (left_or_right != "left")
            handContr_right.close();

        if (left_or_right != "right")
            handContr_left.close();
    }

    if (!portForces_right.isClosed())
        portForces_right.close();
    if (!portForces_left.isClosed())
        portForces_left.close();

    if (portWholeBodyRpc.asPort().isOpen())
        portWholeBodyRpc.close();

    return true;
}

/*******************************************************************************/
bool GraspExecution::goHome(const string &hand)
{
    bool done;
    int context_tmp;


    if (visual_serv)
        visual_servoing_right->stopFacilities();

    if (hand=="right")
    {
        icart_right->storeContext(&context_tmp);
        double min, max;
        yDebug()<<"Setting torso to 0";
        icart_right->setLimits(0, 0.0, 0.0);
        icart_right->setLimits(1, 0.0, 0.0);
        icart_right->setLimits(2, 0.0, 0.0);

        icart_right->getLimits(0, &min, &max);
        yDebug()<<"Get limit of pitch"<<min<<max;

        yDebug()<<"[GraspExecution]: opening hand ... ";
        handContr_right.openHand(true, true);
        yDebug()<<"[GraspExecution]: going back home: "<<home_right.toString(3,3);
        icart_right->goToPoseSync(home_right.subVector(0,2),home_right.subVector(3,6));
        if (compliant)
        {
            icart_right->waitMotionDone(0.1, 2.0);
            done=true;
        }
        else
        {
            icart_right->waitMotionDone();
            icart_right->checkMotionDone(&done);
        }

        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_right->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(home_right.subVector(0,2)-x_reached)<<" and in orientation: "<<norm(home_right.subVector(3,6)-o_reached);
             icart_right->restoreContext(context_tmp);

             icart_right->getLimits(0, &min, &max);
             yDebug()<<"Get limits os pitch restored"<<min<<max;
        }
    }
    if (hand=="left")
    {
        icart_left->storeContext(&context_tmp);
        double min, max;
        yDebug()<<"Setting torso to 0";
        icart_left->setLimits(0, 0.0, 0.0);
        icart_left->setLimits(1, 0.0, 0.0);
        icart_left->setLimits(2, 0.0, 0.0);

        icart_left->getLimits(0, &min, &max);
        yDebug()<<"Get limit of pitch"<<min<<max;


        yDebug()<<"[GraspExecution]: opening hand ... ";
        handContr_left.openHand(true, true);
        yDebug()<<"[GraspExecution]: going back home: "<<home_left.toString(3,3);
        icart_left->goToPoseSync(home_left.subVector(0,2),home_left.subVector(3,6));
        if (compliant)
        {
            icart_left->waitMotionDone(0.1, 2.0);
            done=true;
        }
        else
        {
            icart_left->waitMotionDone();
            icart_left->checkMotionDone(&done);
        }

        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_left->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(home_left.subVector(0,2)-x_reached)<<" and in orientation: "<<norm(home_left.subVector(3,6)-o_reached);
             icart_left->restoreContext(context_tmp);
        }
    }

    return done;
}

/*******************************************************************************/
bool GraspExecution::goToBasket(const string &hand)
{
    bool done;
    int context_tmp;

    if (visual_serv)
        visual_servoing_right->stopFacilities();

    if (hand=="right")
    {
        yDebug()<<"[GraspExecution]: going to the basket: "<<basket_right.toString(3,3);
        icart_right->goToPoseSync(basket_right.subVector(0,2),basket_right.subVector(3,6));
        if (compliant)
        {
            icart_right->waitMotionDone(0.1, 2.0);
            done=true;
        }
        else
        {
            icart_right->waitMotionDone();
            icart_right->checkMotionDone(&done);
        }

        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_right->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(basket_right.subVector(0,2)-x_reached)<<" and in orientation: "<<norm(basket_right.subVector(3,6)-o_reached);
        }
        yDebug()<<"[GraspExecution]: opening hand ... ";
        handContr_right.openHand(true, true);

    }
    if (hand=="left")
    {
        yDebug()<<"[GraspExecution]: going to the basket: "<<basket_left.toString(3,3);
        icart_left->goToPoseSync(basket_left.subVector(0,2),basket_left.subVector(3,6));
        if (compliant)
        {
            icart_left->waitMotionDone(0.1, 2.0);
            done=true;
        }
        else
        {
            icart_left->waitMotionDone();
            icart_left->checkMotionDone(&done);
        }

        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_left->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(basket_left.subVector(0,2)-x_reached)<<" and in orientation: "<<norm(basket_left.subVector(3,6)-o_reached);
        }
        yDebug()<<"[GraspExecution]: opening hand ... ";
        handContr_left.openHand(true, true);
    }

    return done;
}

/*******************************************************************************/
bool GraspExecution::stop()
{
    reached=true;
}

/*******************************************************************************/
void GraspExecution::liftObject(deque<Vector> &traj, int index)
{
    Vector waypoint_lift(6,0.0);
    waypoint_lift=trajectory[index-1];
    waypoint_lift[2]+=lift_z;

    traj.push_back(waypoint_lift);
    //traj.push_back(trajectory[index-1]);
}

/*******************************************************************************/
bool GraspExecution::graspObject(const string &hand)
{
    yDebug()<<"[GraspExecution]: Grasping object ..";
    bool f;
    if (hand=="right")
    {
        handContr_right.openHand(false,true);
        handContr_right.closeHand(true);
        f=handContr_right.isHandClose();
    }
    else
    {
        handContr_left.openHand(false,true);
        handContr_left.closeHand(true);
        f=handContr_left.isHandClose();
    }

    return f;
}

/*******************************************************************************/
bool GraspExecution::releaseObject(const string &hand)
{
    yDebug()<<"[GraspExecution]: Releasing object ..";
    bool f;
    if (hand=="right")
    {
        handContr_right.openHand(true, true);
    }
    else
    {
        handContr_left.openHand(true, true);
    }

    return true;
}
