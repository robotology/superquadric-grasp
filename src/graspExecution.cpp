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
GraspExecution::GraspExecution(Property &_movement_par, const Property &_complete_sol):
               movement_par(_movement_par), complete_sol(_complete_sol)
{

}

/*******************************************************************************/
bool GraspExecution::configure()
{
    home_right.resize(7,0.0);
    home_left.resize(7,0.0);
    shift.resize(3,0.0);

    i=-1;

    setPosePar(movement_par);

    if (left_or_right!="both")
    {
        configDev(left_or_right);
    }
    else
    {
        configDev("right");
        configDev("left");
    }

    reached=false;
}

/*******************************************************************************/
bool GraspExecution::configDev(const string &which_hand)
{
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
        newDof[0]=1;
        newDof[1]=1;
        newDof[2]=1;
        icart_right->setDOF(newDof,curDof);

        icart_right->storeContext(&context_right);

        icart_right->setTrajTime(traj_time);
        icart_right->setInTargetTol(traj_tol);
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

        icart_left->setTrajTime(traj_time);

        icart_left->setInTargetTol(traj_tol);
    }

    return true;
}

/*******************************************************************************/
void GraspExecution::setPosePar(const Property &newOptions)
{
    LockGuard lg(mutex);

    string rob=newOptions.find("robot").asString();

    if (newOptions.find("robot").isNull())
    {
        robot="icubSim";
    }
    else
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

    if (newOptions.find("hand").isNull())
    {
        left_or_right="both";
    }
    else
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

    double ttime=newOptions.find("traj_time").asDouble();

    if (newOptions.find("traj_time").isNull())
    {
        traj_time=2.0;
    }
    else
    {
        if ((ttime>=0.5) && (ttime<=5.0))
        {
            traj_time=ttime;
        }
        else
        {
            traj_time=2.0;
        }
    }

    double ttol=newOptions.find("traj_tol").asDouble();

    if (newOptions.find("traj_tol").isNull())
    {
        traj_tol=0.005;
    }
    else
    {
        if ((ttol>=0.5) && (ttol<=5.0))
        {
            traj_tol=ttol;
        }
        else
        {
            traj_tol=0.005;
        }
    }

    Bottle *sh=newOptions.find("shift").asList();
    if (newOptions.find("shift").isNull())
    {
        shift[0]=0.05;
        shift[1]=0.0;
        shift[2]=0.0;
    }
    else
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
            shift[0]=0.05;
            shift[1]=0.0;
            shift[2]=0.0;
        }
    }

    Bottle *pl=newOptions.find("home_position_right").asList();
    if (newOptions.find("home_position_right").isNull())
    {
        home_right[0]=-0.35; home_right[1]=0.25; home_right[2]=0.2;
        home_right[3]=-0.035166; home_right[4]=-0.67078; home_right[5]=0.734835; home_right[6]=2.46923;
    }
    else
    {
        Vector tmp(4,0.0);
        tmp[0]=pl->get(0).asDouble();
        tmp[1]=pl->get(1).asDouble();
        tmp[2]=pl->get(2).asDouble();
        tmp[3]=pl->get(3).asDouble();
        tmp[1]=pl->get(4).asDouble();
        tmp[2]=pl->get(5).asDouble();
        tmp[3]=pl->get(6).asDouble();
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

    Bottle *pll=newOptions.find("home_position_left").asList();
    if (newOptions.find("home_position_left").isNull())
    {
        home_left[0]=-0.35; home_left[1]=-0.25; home_left[2]=0.2;
        home_left[3]=-0.35166; home_left[4]=0.697078; home_left[5]=-0.624835; home_left[6]=3.106923;
    }
    else
    {
        Vector tmp(4,0.0);
        tmp[0]=pll->get(0).asDouble();
        tmp[1]=pll->get(1).asDouble();
        tmp[2]=pll->get(2).asDouble();
        tmp[3]=pll->get(3).asDouble();
        tmp[1]=pll->get(4).asDouble();
        tmp[2]=pll->get(5).asDouble();
        tmp[3]=pll->get(6).asDouble();
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
}

/*******************************************************************************/
Property GraspExecution::getPosePar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("robot",robot);
    advOptions.put("hand",left_or_right);
    advOptions.put("traj_time",traj_time);
    advOptions.put("traj_tol",traj_tol);
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
    bool reached_tot;
    trajectory.clear();

    if (hand=="right")
    {
        for (size_t i=0; i<trajectory_right.size(); i++)
            trajectory.push_back(trajectory_right[i]);

        for (size_t i=1; i<trajectory_right.size(); i++)
            trajectory.push_back(trajectory_right[trajectory_right.size()-1-i]);
    }
    else
    {
        for (size_t i=0; i<trajectory_left.size(); i++)
            trajectory.push_back(trajectory_left[i]);

        for (size_t i=1; i<trajectory_left.size(); i++)
            trajectory.push_back(trajectory_left[trajectory_right.size()-1-i]);
    }

    if (i==-1)
    {
        reached=false;
        reached_tot=false;
        i++;
    }

    if ((reached==false) && (i<=trajectory.size()) && (i>=0))
    {
        yDebug()<<"[GraspExecution]: Waypoint: "<<i<<" : "<<trajectory[i].toString();
        reached=reachWaypoint(i, hand);
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

/*******************************************************************************/
bool GraspExecution::reachWaypoint(int i, string &hand)
{
    bool done;

    Vector x(3,0.0);
    Vector o(4,0.0);

    x=trajectory[i].subVector(0,2);
    o=(dcm2axis(euler2dcm(trajectory[i].subVector(3,5))));

    if (hand=="right")
    {
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
    }
    if (hand=="left")
    {
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
    }

    return done;
}

/*******************************************************************************/
bool GraspExecution::release()
{
    if (hand_to_move=="right")
        icart_right->stopControl();
    else
        icart_left->stopControl();

    icart_right->restoreContext(context_right);
    icart_left->restoreContext(context_left);

    if (left_or_right=="both" || left_or_right=="right")
        robotDevice_right.close();
    if (left_or_right=="both" || left_or_right=="left")
        robotDevice_left.close();
}

/*******************************************************************************/
bool GraspExecution::goHome(const string &hand)
{
    bool done;

    if (hand=="right")
    {
        yDebug()<<"[GraspExecution]: going back home: "<<home_right.toString();
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


