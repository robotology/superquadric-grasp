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
using namespace tactileControl;

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
    shift.resize(3,0.0);

    i=-1;

    setPosePar(movement_par, true);

    if (left_or_right!="both")
    {
        config=configCartesian(left_or_right);
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
    bool done;
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

        Vector curDof;
        icart_right->getDOF(curDof);
        Vector newDof(3);
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        icart_right->setDOF(newDof,curDof);

        icart_right->getDOF(curDof);

        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        icart_right->storeContext(&context_right);

        icart_right->setTrajTime(traj_time);

        icart_right->setInTargetTol(traj_tol);

        icart_right->goToPoseSync(home_right.subVector(0,2),home_right.subVector(3,6));
        icart_right->waitMotionDone();
        icart_right->checkMotionDone(&done);

        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;
        icart_right->setDOF(newDof,curDof);
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

        Vector curDof;
        icart_left->getDOF(curDof);
        Vector newDof(3);
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        icart_left->setDOF(newDof,curDof);

        icart_left->getDOF(curDof);
        yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

        icart_left->storeContext(&context_left);

        icart_left->setTrajTime(traj_time);

        icart_left->setInTargetTol(traj_tol);

        icart_left->goToPoseSync(home_left.subVector(0,2),home_left.subVector(3,6));
        icart_left->waitMotionDone();
        icart_left->checkMotionDone(&done);

        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;
        icart_right->setDOF(newDof,curDof);
    }

    return done;
}

/*******************************************************************************/
bool GraspExecution::configVisualServoing()
{
    Property prop_server_vs;
    prop_server_vs.put("device","visualservoingclient");
    prop_server_vs.put("verbosity",true);
    prop_server_vs.put("local","/VisualServoingClientTest");
    prop_server_vs.put("remote", "/visual-servoing");

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
            handContr_left.set("useRingLittleFingers", Value(five_fingers));
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
        if ((ptol>=5.0) && (ptol<=20.0))
        {
            pixel_tol=ptol;
        }
        else if (ptol<5.0)
        {
            pixel_tol=5.0;
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

    Bottle *sh=newOptions.find("shift").asList();
    if (newOptions.find("shift").isNull() && (first_time==true))
    {
        shift[0]=0.0;
        shift[1]=0.0;
        shift[2]=0.0;
    }
    else if (!newOptions.find("shift").isNull())
    {
        Vector tmp(3,0.0);
        tmp[0]=sh->get(0).asDouble();
        tmp[1]=sh->get(1).asDouble();
        tmp[2]=sh->get(2).asDouble();

        if (norm(tmp)>0.0)
        {
            shift=tmp;
        }
        else
        {
            shift[0]=0.0;
            shift[1]=0.0;
            shift[2]=0.0;
        }
    }

    Bottle *pl=newOptions.find("home_right").asList();
    if (newOptions.find("home_right").isNull() && (first_time==true))
    {
        home_right[0]=-0.35; home_right[1]=0.25; home_right[2]=0.2;
        home_right[3]=-0.035166; home_right[4]=-0.67078; home_right[5]=0.734835; home_right[6]=2.46923;
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
            home_right[0]=-0.35; home_right[1]=0.25; home_right[2]=0.2;
            home_right[3]=-0.035166; home_right[4]=-0.67078; home_right[5]=0.734835; home_right[6]=2.46923;
        }
    }

    Bottle *pll=newOptions.find("home_left").asList();
    if (newOptions.find("home_left").isNull() && (first_time==true))
    {
        home_left[0]=-0.35; home_left[1]=-0.25; home_left[2]=0.2;
        home_left[3]=-0.35166; home_left[4]=0.697078; home_left[5]=-0.624835; home_left[6]=3.106923;
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
            home_left[0]=-0.35; home_left[1]=-0.25; home_left[2]=0.2;
            home_left[3]=-0.35166; home_left[4]=0.697078; home_left[5]=-0.624835; home_left[6]=3.106923;
        }
    }

    double pitch_max=newOptions.find("torso_pitch_max").asDouble();

    if (newOptions.find("torso_pitch_max").isNull() && (first_time==true))
    {
        torso_pitch_max=30.0;
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
            torso_pitch_max=40.0;
        }
    }
}

/*******************************************************************************/
Property GraspExecution::getPosePar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("robot",robot);
    advOptions.put("hand",left_or_right);
    advOptions.put("five_fingers",five_fingers);
    advOptions.put("five_fingers",five_fingers);
    advOptions.put("traj_time",traj_time);
    advOptions.put("traj_tol",traj_tol);
    advOptions.put("lift_z",lift_z);
    Bottle s;
    Bottle &pd=s.addList();
    pd.addDouble(shift[0]); pd.addDouble(shift[1]);
    pd.addDouble(shift[2]);
    advOptions.put("shift",s.get(0));

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
    }
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

                liftObject(trajectory, trajectory_right.size());

            for (size_t i=1; i<trajectory_right.size(); i++)
                trajectory.push_back(trajectory_right[trajectory_right.size()-1-i]);

            trajectory.push_back(home_right);
        }
        else
        {
            for (size_t i=0; i<trajectory_left.size(); i++)
            {
                trajectory.push_back(trajectory_left[i]);
            }

                liftObject(trajectory, trajectory_left.size());

            for (size_t i=1; i<trajectory_left.size(); i++)
                trajectory.push_back(trajectory_left[trajectory_right.size()-1-i]);

            trajectory.push_back(home_left);
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
    bool done;
    int context_tmp;

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

    if (hand=="right")
    {
        if(i==trajectory.size()-1)
        {
            for (size_t i=0; i<limit_min.size(); i++)
            {
                icart_right->getLimits(i, &min, &max);
                limit_min[i]=min;
                limit_max[i]=max;
            }

            icart_right->setLimits(0,0.0, 0.0);
            icart_right->setLimits(1,0.0, 0.0);
            icart_right->setLimits(2,0.0, 0.0);    
        }
        
        icart_right->getDOF(curDof);
        icart_right->setDOF(newDof,curDof);

        if (i==0)
        {
            yDebug()<<"[GraspExecution]: opening hand ... ";
            handContr_right.openHand(true, true);
        }

        icart_right->goToPoseSync(x,o);
        icart_right->waitMotionDone();
        icart_right->checkMotionDone(&done);

        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_right->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(x-x_reached)<<" and in orientation: "<<norm(o-o_reached);
        }

        if(i==trajectory.size()-1)
        {
            icart_right->setLimits(0,limit_min[0], limit_max[0]);
            icart_right->setLimits(1,limit_min[1], limit_max[1]);
            icart_right->setLimits(2,limit_min[2], torso_pitch_max);
        }

    }
    if (hand=="left")
    {
        if(i==trajectory.size()-1)
        {
            for (size_t i=0; i<limit_min.size(); i++)
            {
                icart_left->getLimits(i, &min, &max);
                limit_min[i]=min;
                limit_max[i]=max;
            }

            icart_left->setLimits(0,0.0, 0.0);
            icart_left->setLimits(1,0.0, 0.0);
            icart_left->setLimits(2,0.0, 0.0);  
        }
 
        icart_left->getDOF(curDof);
        icart_left->setDOF(newDof,curDof);

        if (i==0)
        {
            yDebug()<<"[GraspExecution]: opening hand ... ";
            handContr_left.openHand(true, true);
        }

        icart_left->goToPoseSync(x,o);
        icart_left->waitMotionDone();
        icart_left->checkMotionDone(&done);

        if (done)
        {
            Vector x_reached(3,0.0);
            Vector o_reached(4,0.0);
            icart_left->getPose(x_reached, o_reached);
            yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(x-x_reached)<<" and in orientation: "<<norm(o-o_reached);
        }

        if(i==trajectory.size()-1)
        {
            icart_left->setLimits(0,limit_min[0], limit_max[0]);
            icart_left->setLimits(1,limit_min[1], limit_max[1]);
            icart_left->setLimits(2,limit_min[2], torso_pitch_max);
        }
    }

    Vector Dof;
    icart_right->getDOF(Dof);
    yDebug()<<"Dof used "<<Dof.toString(3,3);

    return done;
}

/*******************************************************************************/
bool GraspExecution::reachWithVisual(int i, string &hand)
{
    Vector x(3,0.0);
    Vector o(4,0.0);
    bool done;

    vector<Vector> pixels_left, pixels_right;

    x=trajectory[i].subVector(0,2);
    if (trajectory[i].size()==6)
        o=(dcm2axis(euler2dcm(trajectory[i].subVector(3,5))));
    else
        o=trajectory[i].subVector(3,6);

    pixels_left=visual_servoing_right->getPixelPositionGoalFrom3DPose(x,o, IVisualServoing::CamSel::left);
    pixels_right=visual_servoing_right->getPixelPositionGoalFrom3DPose(x,o,IVisualServoing::CamSel::right);

    visual_servoing_right->goToGoal(pixels_left, pixels_right);

    Time::delay(2.0);
    done=visual_servoing_right->waitVisualServoingDone();

    yDebug()<<"done"<<done;

    return done;
}

/*******************************************************************************/
bool GraspExecution::release()
{
    if (hand_to_move=="right" && left_or_right !="left")
        icart_right->stopControl();
    else if (hand_to_move=="left" && left_or_right !="right")
        icart_left->stopControl();

    if (left_or_right=="both" || left_or_right=="right")
    {
        robotDevice_right.close();
        icart_right->restoreContext(context_right);
    }
    if (left_or_right=="both" || left_or_right=="left")
    {
        robotDevice_left.close();
        icart_left->restoreContext(context_left);
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

    return true;
}

/*******************************************************************************/
bool GraspExecution::goHome(const string &hand)
{
    bool done;
    int context_tmp;

    if (hand=="right")
    {
        yDebug()<<"[GraspExecution]: opening hand ... ";
        handContr_right.openHand(true, true);
        yDebug()<<"[GraspExecution]: going back home: "<<home_right.toString(3,3);
        icart_right->goToPoseSync(home_right.subVector(0,2),home_right.subVector(3,6));
        icart_right->waitMotionDone();
        icart_right->checkMotionDone(&done);
        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_right->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(home_right.subVector(0,2)-x_reached)<<" and in orientation: "<<norm(home_right.subVector(3,6)-o_reached);
        }
    }
    if (hand=="left")
    {
        yDebug()<<"[GraspExecution]: opening hand ... ";
        handContr_left.openHand(true, true);
        yDebug()<<"[GraspExecution]: going back home: "<<home_left.toString(3,3);
        icart_left->goToPoseSync(home_left.subVector(0,2),home_left.subVector(3,6));
        icart_left->waitMotionDone();
        icart_left->checkMotionDone(&done);
        if (done)
        {
             Vector x_reached(3,0.0);
             Vector o_reached(4,0.0);
             icart_left->getPose(x_reached, o_reached);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(home_left.subVector(0,2)-x_reached)<<" and in orientation: "<<norm(home_left.subVector(3,6)-o_reached);
        }
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
    traj.push_back(trajectory[index-1]);
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


