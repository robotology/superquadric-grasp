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
                               bool _grasp):movement_par(_movement_par), complete_sol(_complete_sol), grasp(_grasp)

{

}

/*******************************************************************************/
bool GraspExecution::configure()
{
    bool config;

    shift.resize(3,0.0);

    i=-1;

    setPosePar(movement_par, true);

    if (left_or_right!="both")
    {
        config=configController(left_or_right);
    }
    else
    {
        config=configController("right");
        config=config && configController("left");
    }

    if (grasp)
    {
        config=config && configGrasp();

        yDebug()<<"Grasped configured: "<<config;
    }

    reached=false;

    return config;
}

/*******************************************************************************/
bool GraspExecution::configController(const string &which_hand)
{
    if (which_hand=="right")
    {
        reachRightPort.open("/superquadric-grasp/right/reach:rpc");
        stateRightPort.open("/superquadric-grasp/right/state:i");
    }
    else if (which_hand=="left")
    {
        reachLeftPort.open("/superquadric-grasp/left/reach:rpc");
        stateLeftPort.open("/superquadric-grasp/left/state:i");
    }

    return true;
}

/*******************************************************************************/
bool GraspExecution::configGrasp()
{
    if (left_or_right=="right" || left_or_right=="both")
    {
        // configure the options for the driver
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/cer/right_hand");
        option.put("local","/controller");

        // open the driver
        if (!driver_right.open(option))
        {
            yError()<<"Unable to open the device driver";
            return false;
        }

        // open the views
        driver_right.view(imod_right);
        driver_right.view(ienc_right);
        driver_right.view(ipos_right);

        // tell the device we aim to control
        // in position mode all the joints
        int nAxes;
        ienc_right->getAxes(&nAxes);
        vector<int> modes(nAxes,VOCAB_CM_POSITION);
        imod_right->setControlModes(modes.data());
    }

    if (left_or_right=="left" || left_or_right=="both")
    {
        // configure the options for the driver
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/cer/left_hand");
        option.put("local","/controller");

        // open the driver
        if (!driver_left.open(option))
        {
            yError()<<"Unable to open the device driver";
            return false;
        }

        // open the views
        driver_left.view(imod_left);
        driver_left.view(ienc_left);
        driver_left.view(ipos_left);

        // tell the device we aim to control
        // in position mode all the joints
        int nAxes;
        ienc_left->getAxes(&nAxes);
        vector<int> modes(nAxes,VOCAB_CM_POSITION);
        imod_left->setControlModes(modes.data());
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

    double angp=newOptions.find("angle_paddle").asDouble();

    if (newOptions.find("angle_paddle").isNull() && (first_time==true))
    {
        angle_paddle=70.0;
    }
    else if (!newOptions.find("angle_paddle").isNull())
    {
        if ((angp>=0.001) && (angp<=80.0))
        {
            angle_paddle=angp;
        }
        else if (angp<0.001)
        {
            angle_paddle=0.001;
        }
        else if (angp>80.0)
        {
            angle_paddle=70.0;
        }
    }

    double angt=newOptions.find("angle_thumb").asDouble();

    if (newOptions.find("angle_thumb").isNull() && (first_time==true))
    {
        angle_thumb=50.0;
    }
    else if (!newOptions.find("angle_thumb").isNull())
    {
        if ((angt>=0.001) && (angt<=80.0))
        {
            angle_thumb=angt;
        }
        else if (angt<0.001)
        {
            angle_thumb=0.001;
        }
        else if (angt>80.0)
        {
            angle_thumb=80.0;
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
        home_right[0]=0.35; home_right[1]=-0.25; home_right[2]=0.7;
        home_right[3]=1.0; home_right[4]=0.0; home_right[5]=0.0; home_right[6]=-1.57;
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
            home_right[0]=0.35; home_right[1]=-0.25; home_right[2]=0.7;
            home_right[3]=1.0; home_right[4]=0.0; home_right[5]=0.0; home_right[6]=-1.57;
        }
    }

    Bottle *pll=newOptions.find("home_left").asList();
    if (newOptions.find("home_left").isNull() && (first_time==true))
    {
        home_left[0]=0.35; home_left[1]=0.25; home_left[2]=0.7;
        home_left[3]=1.0; home_left[4]=0.0; home_left[5]=0.0; home_left[6]=-1.57;
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
            home_left[0]=0.35; home_left[1]=0.25; home_left[2]=0.7;
            home_left[3]=1.0; home_left[4]=0.0; home_left[5]=0.0; home_left[6]=-1.57;
        }
    }

    yDebug()<<"In execution module ....";
    yInfo()<<"[GraspExecution] lift_z:     "<<lift_z;
    yInfo()<<"[GraspExecution] shift:      "<<shift.toString(3,3);
    yInfo()<<"[GraspExecution] home_right: "<<home_right.toString(3,3);
    yInfo()<<"[GraspExecution] home_left:  "<<home_left.toString(3,3);
    yInfo()<<"[GraspExecution] angle_paddle: "<<angle_paddle;
    yInfo()<<"[GraspExecution] angle_thumb:  "<<angle_thumb;


}

/*******************************************************************************/
Property GraspExecution::getPosePar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("robot",robot);
    advOptions.put("hand",left_or_right);

    advOptions.put("lift_z",lift_z);
    advOptions.put("angle_paddle",angle_paddle);
    advOptions.put("angle_thumb",angle_thumb);
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

        if (norm(shift)>0.0)
        {
            for (size_t k=0; k<trajectory_right.size(); k++)
            {
                yDebug()<<"[GraspExecution]: Waypoint "<<k<<trajectory_right[k].toString(3,3);
                
                trajectory_right[k].setSubvector(0,trajectory_right[k].subVector(0,2) +shift);
                yDebug()<<"[GraspExecution]: Shifted waypoint "<<k<<trajectory_right[k].toString(3,3);
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


        if (norm(shift)>0.0)
        {
            for (size_t k=0; k<trajectory_left.size(); k++)
            {
                yDebug()<<"[GraspExecution]: Waypoint "<<k<<trajectory_left[k].toString(3,3);
                trajectory_left[k].setSubvector(0,trajectory_left[k].subVector(0,2) +shift);
                yDebug()<<"[GraspExecution]: Shifted waypoint "<<k<<trajectory_left[k].toString(3,3);
            }
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

            //trajectory.push_back(home_right);
        }
        else
        {
            for (size_t i=0; i<trajectory_left.size(); i++)
            {
                trajectory.push_back(trajectory_left[i]);
            }

                liftObject(trajectory, trajectory_left.size());

            for (size_t i=1; i<trajectory_left.size(); i++)
                trajectory.push_back(trajectory_left[trajectory_left.size()-1-i]);

            //trajectory.push_back(home_left);
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

        if ((reached==false) && (i<=2) && (i>=0))
        {
            yDebug()<<"[GraspExecution]: Waypoint: "<<i<<" : "<<trajectory[i].toString(3,3);

            string mode="heave";

            reached=reachWaypoint(i, hand, mode);

            if (grasp==true && reached==true)
            {
               if (((i==1) && (hand=="right")) || ((i==1) && (hand=="left")))
                    reached=graspObject(hand);

                //if (((i==trajectory_right.size()+1) && (hand=="right")) || ((i==trajectory_left.size()+1) && (hand=="left")))
                //    reached=releaseObject(hand);
            }
        }

        if (reached==true)
        {
            i++;
        }

        if ((i==3) && (reached==true))
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
bool GraspExecution::reachWaypoint(int i, const string &hand, const string &mode)
{
    bool done=false;

    double min, max;

    Vector x(3,0.0);
    Vector o(4,0.0);

    x=trajectory[i].subVector(0,2);
    if (trajectory[i].size()==6)
        o=(dcm2axis(euler2dcm(trajectory[i].subVector(3,5))));
    else
        o=trajectory[i].subVector(3,6);

    if (hand=="right")
    {
        if (i==0)
        {
            yDebug()<<"[GraspExecution]: opening hand ... ";
            if (grasp)
                releaseObject(hand);
        }

        Bottle cmd, reply;
        cmd.addString("go");
        Bottle &content=cmd.addList();
        Bottle &par=content.addList();
        par.addString("parameters");
        Bottle &par_val=par.addList();
        Bottle &value=par_val.addList();
        value.addString("mode");
        value.addString("full_pose+no_torso_"+mode);

        Bottle &target=content.addList();
        target.addString("target");
        Bottle &pose_t=target.addList();
        pose_t.addDouble(x[0]); pose_t.addDouble(x[1]); pose_t.addDouble(x[2]);
        pose_t.addDouble(o[0]); pose_t.addDouble(o[1]); pose_t.addDouble(o[2]); pose_t.addDouble(o[3]);

        yDebug()<<"Command for moving "<<cmd.toString();

        reachRightPort.write(cmd, reply);

        if (reply.get(0).asString()=="ack")
            yInfo()<<"Motion started";
        else
            yError()<<"Problems in sending the command";

        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            cmd.clear(); reply.clear();
            cmd.addString("done");

            reachRightPort.write(cmd, reply);

            if (reply.get(1).asInt()==1)
            {
                done=true;
                yInfo()<<"Movement completed";
                break;
            }
        }

        if (done)
        {
             Vector *pose_reached=stateRightPort.read(false);
             yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(x-pose_reached->subVector(0,2))<<" and in orientation: "<<norm(o-pose_reached->subVector(3,6));
        }
        else
            yError()<<"Target point not reached!";
    }
    if (hand=="left")
    {
        if (i==0)
        {
            yDebug()<<"[GraspExecution]: opening hand ... ";
            if (grasp)
                releaseObject(hand);
        }

        Bottle cmd, reply;
        cmd.addString("go");
        Bottle &content=cmd.addList();
        Bottle &par=content.addList();
        par.addString("parameters");
        Bottle &par_val=par.addList();
        Bottle &value=par_val.addList();
        value.addString("mode");
        value.addString("full_pose+no_torso_"+mode);

        Bottle &target=content.addList();
        target.addString("target");
        Bottle &pose_t=target.addList();
        pose_t.addDouble(x[0]); pose_t.addDouble(x[1]); pose_t.addDouble(x[2]);
        pose_t.addDouble(o[0]); pose_t.addDouble(o[1]); pose_t.addDouble(o[2]); pose_t.addDouble(o[3]);

        yDebug()<<"Command for moving "<<cmd.toString();

        reachLeftPort.write(cmd, reply);

        if (reply.get(0).asString()=="ack")
            yInfo()<<"Motion started";
        else
            yError()<<"Problems in sending the command";

        double t0=Time::now();
        while (Time::now()-t0<5.0)
        {
            cmd.clear(); reply.clear();
            cmd.addString("done");

            reachLeftPort.write(cmd, reply);

            if (reply.get(1).asInt()==1)
            {
                done=true;
                yInfo()<<"Movement completed";
                break;
            }
        }

        if (done)
        {
            Vector *pose_reached=stateLeftPort.read(false);
            yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(x-pose_reached->subVector(0,2))<<" and in orientation: "<<norm(o-pose_reached->subVector(3,6));
        }
        else
            yError()<<"Target point not reached!";
    }

    return done;
}

/*******************************************************************************/
bool GraspExecution::release()
{
    if (left_or_right=="both" || left_or_right=="right")
    {
        if (reachRightPort.asPort().isOpen())
            reachRightPort.close();

        if (!stateRightPort.isClosed())
            stateRightPort.close();
    }
    if (left_or_right=="both" || left_or_right=="left")
    {
        if (reachLeftPort.asPort().isOpen())
            reachLeftPort.close();

        if (!stateLeftPort.isClosed())
            stateLeftPort.close();
    }

    return true;
}

/*******************************************************************************/
bool GraspExecution::goHome(const string &hand)
{
    bool done;

    if (hand=="right")
    {        
        yDebug()<<"[GraspExecution]: opening hand ... ";
        //releaseObject(hand);
        //handContr_right.openHand(true, true);
        yDebug()<<"[GraspExecution]: going back home: "<<home_right.toString(3,3);
        Bottle cmd, reply;
        cmd.addString("go");
        Bottle &content=cmd.addList();
        Bottle &par=content.addList();
        par.addString("parameters");
        Bottle &par_val=par.addList();
        Bottle &value=par_val.addList();
        value.addString("mode");
        value.addString("full_pose+no_torso_no_heave");

        Bottle &target=content.addList();
        target.addString("target");
        Bottle &pose_t=target.addList();
        pose_t.addDouble(home_right[0]); pose_t.addDouble(home_right[1]); pose_t.addDouble(home_right[2]);
        pose_t.addDouble(home_right[3]); pose_t.addDouble(home_right[4]); pose_t.addDouble(home_right[5]); pose_t.addDouble(home_right[6]);

        yDebug()<<"Command for moving "<<cmd.toString();

        reachRightPort.write(cmd, reply);

        if (reply.get(0).asString()=="ack")
            yInfo()<<"Motion started";
        else
            yError()<<"Problems in sending the command";

        double t0=Time::now();
        while (Time::now()-t0<5.0)
        {
            cmd.clear(); reply.clear();
            cmd.addString("done");

            reachRightPort.write(cmd, reply);

            if (reply.get(1).asInt()==1)
            {
                done=true;
                yInfo()<<"Movement completed";
                break;
            }
        }

        if (done)
        {
            Vector *pose_reached=stateRightPort.read(false);
            yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(home_right.subVector(0,2)-pose_reached->subVector(0,2))<<" and in orientation: "<<norm(home_right.subVector(3,6)-pose_reached->subVector(3,6));
        }
        else
            yError()<<"Home position not reached!";
    }
    if (hand=="left")
    {
        yDebug()<<"[GraspExecution]: opening hand ... ";
        //releaseObject(hand);
        //handContr_left.openHand(true, true);
        yDebug()<<"[GraspExecution]: going back home: "<<home_left.toString(3,3);
        Bottle cmd, reply;
        cmd.addString("go");
        Bottle &content=cmd.addList();
        Bottle &par=content.addList();
        par.addString("parameters");
        Bottle &par_val=par.addList();
        Bottle &value=par_val.addList();
        value.addString("mode");
        value.addString("full_pose+no_torso_no_heave");

        Bottle &target=content.addList();
        target.addString("target");
        Bottle &pose_t=target.addList();
        pose_t.addDouble(home_left[0]); pose_t.addDouble(home_left[1]); pose_t.addDouble(home_left[2]);
        pose_t.addDouble(home_left[3]); pose_t.addDouble(home_left[4]); pose_t.addDouble(home_left[5]); pose_t.addDouble(home_left[6]);

        yDebug()<<"Command for moving "<<cmd.toString();

        reachRightPort.write(cmd, reply);

        if (reply.get(0).asString()=="ack")
            yInfo()<<"Motion started";
        else
            yError()<<"Problems in sending the command";

        double t0=Time::now();
        while (Time::now()-t0<5.0)
        {
            cmd.clear(); reply.clear();
            cmd.addString("done");

            reachLeftPort.write(cmd, reply);

            if (reply.get(1).asInt()==1)
            {
                done=true;
                yInfo()<<"Movement completed";
                break;
            }
        }

        if (done)
        {
            Vector *pose_reached=stateLeftPort.read(false);
            yDebug()<<"[Grasp Execution]: Waypoint "<<i<< " reached with error in position: "<<norm(home_left.subVector(0,2)-pose_reached->subVector(0,2))<<" and in orientation: "<<norm(home_left.subVector(3,6)-pose_reached->subVector(3,6));
        }
        else
            yError()<<"Home position not reached!";
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
    yDebug()<<"[GraspExecution]: Grasping object with hand .."<<hand;
    bool f;
    Vector vel(2,50.0);
    if (hand=="right")
    {        
        ipos_right->setRefSpeeds(vel.data());
        Vector angles(2);
        angles[0]=70.0;
        angles[1]=50.0;
        f=ipos_right->positionMove(angles.data());
    }
    else
    {
        ipos_left->setRefSpeeds(vel.data());
        Vector angles(2);
        angles[0]=70.0;
        angles[1]=50.0;
        f=ipos_left->positionMove(angles.data());
        Time::delay(4.0);
        yDebug()<<"Delay ....";
    }

    return f;
}

/*******************************************************************************/
bool GraspExecution::releaseObject(const string &hand)
{
    yDebug()<<"[GraspExecution]: Releasing object ..";
    bool f;
    Vector vel(2,30.0);
    if (hand=="right")
    {
        ipos_right->setRefSpeeds(vel.data());
        Vector angles(2, 0.0);
        f=ipos_right->positionMove(angles.data());
    }
    else
    {
        ipos_left->setRefSpeeds(vel.data());
        Vector angles(2, 0.0);
        f=ipos_left->positionMove(angles.data());
    }

    return f;
}


