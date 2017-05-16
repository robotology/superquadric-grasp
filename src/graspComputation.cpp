#include <cmath>
#include <algorithm>
#include <sstream>
#include <set>
#include <fstream>

#include <yarp/math/Math.h>

#include "graspComputation.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***********************************************************************/
GraspComputation::GraspComputation(const Property &_ipopt_par, const Property &_pose_par,
                                   const Property &_trajectory_par, const string &_left_or_right,
                                    Vector &_hand, Vector &_hand1, ResourceFinder *_rf,
                                   Property &_complete_sol, const Vector &_object):
                                   ipopt_par(_ipopt_par), pose_par(_pose_par), trajectory_par(_trajectory_par),
                                   left_or_right(_left_or_right), hand(_hand), hand1(_hand1), rf(_rf), complete_sol(_complete_sol), object(_object)

{

}

/***********************************************************************/
void GraspComputation::setIpoptPar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);

    double maxCpuTime=newOptions.find("max_cpu_time").asDouble();
    if (newOptions.find("max_cpu_time").isNull() && (first_time==true))
    {
        max_cpu_time=5.0;
    }
    else if (!newOptions.find("max_cpu_time").isNull())
    {
        if ((maxCpuTime>=0.01) && (maxCpuTime<=10.0))
        {
            max_cpu_time=maxCpuTime;
        }
        else if (maxCpuTime<0.01)
        {
            max_cpu_time=0.01;
        }
        else if (maxCpuTime>10.0)
        {
            max_cpu_time=10.0;
        }
    }

    double tolValue=newOptions.find("tol").asDouble();
    if (newOptions.find("tol").isNull() && (first_time==true))
    {
        tol=1e-5;
    }
    else if (!newOptions.find("tol").isNull())
    {
        if ((tolValue>1e-8) && (tolValue<=0.01))
        {
            tol=tolValue;
        }
        else if (tolValue<1e-8)
        {
            tol=1e-8;
        }
        else if (tolValue>0.01)
        {
            tol=0.01;
        }
    }

    double constrTolValue=newOptions.find("constr_viol_tol").asDouble();
    if (newOptions.find("constr_viol_tol").isNull() && (first_time==true))
    {
        constr_viol_tol=1e-5;
    }
    else if (!newOptions.find("constr_viol_tol").isNull())
    {
        if ((constrTolValue>1e-8) && (constrTolValue<=0.01))
        {
            constr_viol_tol=constrTolValue;
        }
        else if (constrTolValue<1e-8)
        {
            constr_viol_tol=1e-8;
        }
        else if (constrTolValue>0.01)
        {
            constr_viol_tol=0.01;
        }
    }

    int accIter=newOptions.find("acceptable_iter").asInt();
    if (newOptions.find("acceptable_iter").isNull() && (first_time==true))
    {
        acceptable_iter=0;
    }
    else if (!newOptions.find("acceptable_iter").isNull())
    {
        if ((accIter>=0 )&& (accIter<=10))
        {
             acceptable_iter=accIter;
        }
        else if (accIter<0 )
        {
            acceptable_iter=0;
        }
        else if (accIter>10)
        {
            acceptable_iter=10;
        }
    }

    int maxIter=newOptions.find("max_iter").asInt();
    if (newOptions.find("max_iter").isNull() && (first_time==true))
    {
        max_iter=100;
    }
    else if (!newOptions.find("max_iter").isNull())
    {
        if ((maxIter>1))
        {
            max_iter=maxIter;
        }
        else
        {
            max_iter=100;
        }
    }

    string mu_str=newOptions.find("mu_strategy").asString();
    if (newOptions.find("mu_strategy").isNull() && (first_time==true))
    {
        mu_strategy="monotone";
    }
    else if (!newOptions.find("mu_strategy").isNull())
    {
        if ((mu_str=="adaptive") || (mu_str=="monotone"))
        {
            mu_strategy=mu_str;
        }
        else
        {
            mu_strategy="monotone";
        }
    }

    string nlp=newOptions.find("nlp_scaling_method").asString();
    if (newOptions.find("nlp_scaling_method").isNull() && (first_time==true))
    {
        nlp_scaling_method="gradient-based";
    }
    else if (!newOptions.find("nlp_scaling_method").isNull())
    {
        if ((nlp=="none") || (nlp=="gradient-based"))
        {
            nlp_scaling_method=nlp;
        }
        else
        {
            nlp_scaling_method="gradient-based";
        }
    }
}

/***********************************************************************/
Property GraspComputation::getIpoptPar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("max_cpu_time",max_cpu_time);
    advOptions.put("tol",tol);
    advOptions.put("max_iter",max_iter);
    advOptions.put("acceptable_iter",acceptable_iter);
    advOptions.put("IPOPT_mu_strategy",mu_strategy);
    advOptions.put("IPOPT_nlp_scaling_method",nlp_scaling_method);
    return advOptions;
}

/***********************************************************************/
void GraspComputation::setPosePar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);
    if (first_time)
    {
        displacement.resize(3,0.0);
        plane.resize(4,0.0);
    }

    int points=newOptions.find("n_pointshand").asInt();

    if (newOptions.find("n_pointshand").isNull() && (first_time==true))
    {
        n_pointshand=46;
    }
    else if (!newOptions.find("n_pointshand").isNull())
    {
        if ((points>=4) && (points<=100))
        {
            n_pointshand=points;
        }
        else if (points<4)
        {
            n_pointshand=4;
        }
        else if (points>100)
        {
            n_pointshand=100;
        }
    }

    Bottle *disp=newOptions.find("hand_displacement").asList();
    if (newOptions.find("hand_displacement").isNull() && (first_time==true))
    {
        displacement[0]=0.05;
        displacement[1]=0.0;
        displacement[2]=0.0;
    }
    else if (!newOptions.find("hand_displacement").isNull())
    {
        Vector tmp(3,0.0);
        tmp[0]=disp->get(0).asDouble();
        tmp[1]=disp->get(1).asDouble();
        tmp[2]=disp->get(2).asDouble();

        if (norm(tmp)>0.0)
        {
            displacement=tmp;
        }
        else
        {
            displacement[0]=0.05;
            displacement[1]=0.0;
            displacement[2]=0.0;
        }
    }
    Bottle *pl=newOptions.find("plane").asList();

    if (newOptions.find("plane").isNull() && (first_time==true))
    {
        plane[0]=0.0; plane[1]=0.0; plane[2]=1.0; plane[3]=0.11;
    }
    else if (!newOptions.find("plane").isNull())
    {
        Vector tmp(4,0.0);
        tmp[0]=pl->get(0).asDouble();
        tmp[1]=pl->get(1).asDouble();
        tmp[2]=pl->get(2).asDouble();
        tmp[3]=pl->get(3).asDouble();
        if (norm(tmp)>0.0)
        {
            plane=tmp;
        }
        else
        {
            plane[0]=0.0; plane[1]=0.0; plane[2]=1.0; plane[3]=0.11;
        }
    }
}

/***********************************************************************/
Property GraspComputation::getPosePar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("n_pointshand",n_pointshand);
    Bottle planed;
    Bottle &pd=planed.addList();
    pd.addDouble(displacement[0]); pd.addDouble(displacement[1]);
    pd.addDouble(displacement[2]);
    advOptions.put("hand_displacement",planed.get(0));

    Bottle planeb;
    Bottle &p2=planeb.addList();
    p2.addDouble(plane[0]); p2.addDouble(plane[1]);
    p2.addDouble(plane[2]); p2.addDouble(plane[3]);
    advOptions.put("plane", planeb.get(0));

    return advOptions;
}

/***********************************************************************/
void GraspComputation::setTrajectoryPar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);

    double dist=newOptions.find("distance_on_x").asDouble();


    if (newOptions.find("distance_on_x").isNull() && (first_time==true))
    {
        distance=0.13;
    }
    else if (!newOptions.find("distance_on_x").isNull())
    {
        if ((dist>=0.0) && (dist<=0.3))
        {
            distance=dist;
        }
        else if (dist<0.0)
        {
            distance=0.0;
        }
        else if (dist>0.3)
        {
            distance=0.3;
        }
    }

    dist=newOptions.find("distance_on_z").asDouble();

    if (newOptions.find("distance_on_z").isNull() && (first_time==true))
    {
        distance1=0.05;
    }
    else if (!newOptions.find("distance_on_z").isNull())
    {
        if ((dist>=0.0) && (dist<=0.3))
        {
            distance1=dist;
        }
        else if (dist<0.0)
        {
            distance1=0.0;
        }
        else if (dist>0.3)
        {
            distance1=0.3;
        }
    }

    string direct=newOptions.find("approaching_direction").asString();
    if (newOptions.find("approaching_direction").isNull() && (first_time==true))
    {
        dir=direct;
    }
    else if (!newOptions.find("approaching_direction").isNull())
    {
        if (direct=="z")
        {
            dir=direct;
        }
        else
        {
            dir="xz";
        }
    }
}

/***********************************************************************/
Property GraspComputation::getTrajectoryPar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("distance_on_x",distance);
    advOptions.put("distance_on_z",distance1);
    advOptions.put("approaching_direction",dir);

    return advOptions;
}

/***********************************************************************/
bool GraspComputation::init()
{
    yInfo()<<"[GraspComputation]: Thread initing ... ";

    setIpoptPar(ipopt_par,true);
    setPosePar(pose_par, true);
    setTrajectoryPar(trajectory_par, true);

    solR.resize(11,0.0);
    solL.resize(11,0.0);
    poseR.resize(6,0.0);
    poseL.resize(6,0.0);

    go_on=false;

    return true;
}

/***********************************************************************/
void GraspComputation::run()
{
    t0=Time::now();
    //LockGuard lg(mutex);

    if (norm(hand)!=0.0 && norm(object)!=0.0)
    {
        if (left_or_right!="both")
            go_on=computePose(hand, left_or_right);
        else if (left_or_right=="both" && norm(hand1)!=0.0)
        {
            go_on=computePose(hand, "right");
            bool go_on1=computePose(hand1, "left");
            go_on=((go_on==true) || (go_on1==true));
        }
    }

    if ((go_on==true))
    {
        if (left_or_right!="both")
            computeTrajectory(left_or_right, dir);
        else
        {
            go_on=computeTrajectory("right", dir);
            bool go_on1=computeTrajectory("left", dir);
            go_on=((go_on==true) || (go_on1==true));
        }
    }

    t_grasp=Time::now() - t0;
}

/***********************************************************************/
bool GraspComputation::computePose(Vector &which_hand, const string &l_o_r)
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",tol);
    app->Options()->SetNumericValue("constr_viol_tol",constr_viol_tol);
    app->Options()->SetIntegerValue("acceptable_iter",acceptable_iter);
    app->Options()->SetStringValue("mu_strategy",mu_strategy);
    app->Options()->SetIntegerValue("max_iter",max_iter);
    app->Options()->SetStringValue("nlp_scaling_method",nlp_scaling_method);
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test","first-order");
    app->Options()->SetStringValue("derivative_test_print_all","yes");
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    Ipopt::SmartPtr<grasping_NLP>  grasp_nlp= new grasping_NLP;
    grasp_nlp->init(object, which_hand, n_pointshand, l_o_r);
    grasp_nlp->configure(this->rf,l_o_r, displacement, plane);

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(grasp_nlp));

    if(status==Ipopt::Solve_Succeeded)
    {
        if (l_o_r=="right")
        {
            solR=grasp_nlp->get_result();
            poseR=grasp_nlp->robot_pose;
            which_hand=grasp_nlp->get_hand();
            yInfo()<<"[GraspComputation]: Solution (hand pose) for "<<l_o_r<<" hand is: "<<poseR.toString(3,3).c_str();
            yInfo()<<"[GraspComputation]: Stretched hand is: "<<which_hand.toString(3,3).c_str();
        }
        else
        {
            solL=grasp_nlp->get_result();
            poseL=grasp_nlp->robot_pose;
            which_hand=grasp_nlp->get_hand();
            yInfo()<<"[GraspComputation]: Solution (hand pose) for "<<l_o_r<<" hand is: "<<poseL.toString(3,3).c_str();
        }
        return true;
    }
    else
    {
        yError()<<"[GraspComputation]: Problem not solved!";
        return false;
    }
}

/***********************************************************************/
bool GraspComputation::computeTrajectory(const string &chosen_hand, const string &direction)
{
    Vector pose(6,0.0);

    if (chosen_hand=="right")
    {
        pose=poseR;
    }
    else
        pose=poseL;

    Vector pose1(6,0.0);
    Vector euler(3,0.0);
    euler[0]=pose[3];
    euler[1]=pose[4];
    euler[2]=pose[5];
    Matrix H(4,4);
    H=euler2dcm(euler);
    euler[0]=pose[0];
    euler[1]=pose[1];
    euler[2]=pose[2];
    H.setSubcol(euler,0,3);

    pose1=pose;

    if (direction=="z")
    {
        if (chosen_hand=="right")
        {
            pose1.setSubvector(0,pose.subVector(0,2)-distance*(H.getCol(2).subVector(0,2)));
        }
        else
        {
            pose1.setSubvector(0,pose.subVector(0,2)+distance*(H.getCol(2).subVector(0,2)));
        }
    }
    else if (direction=="xz")
    {
        if (chosen_hand=="right")
        {
            pose1.setSubvector(0,pose.subVector(0,2)-distance1*(H.getCol(2).subVector(0,2)));
            pose1.setSubvector(0,pose1.subVector(0,2)-distance*(H.getCol(0).subVector(0,2)));
        }
        else
        {
            pose1.setSubvector(0,pose.subVector(0,2)+distance1*(H.getCol(2).subVector(0,2)));
            pose1.setSubvector(0,pose1.subVector(0,2)-distance*(H.getCol(0).subVector(0,2)));
        }
    }

    if (chosen_hand=="right")
    {
        trajectory_right.clear();

        pose.setSubvector(0,pose.subVector(0,2));
        trajectory_right.push_back(pose1);
        trajectory_right.push_back(pose);
    }
    else
    {
        trajectory_left.clear();

        pose.setSubvector(0,pose.subVector(0,2));
        trajectory_left.push_back(pose1);
        trajectory_left.push_back(pose);
    }

    return true;
}

/***********************************************************************/
double GraspComputation::getTime()
{
    LockGuard lg(mutex);
    
    return t_grasp;
}

/***********************************************************************/
void GraspComputation::getSolution(const string &hand)
{
    LockGuard lg(mutex);

    complete_sol=fillProperty(hand);
}

/**********************************************************************/
Property GraspComputation::fillProperty(const string &l_o_r)
{
    Property poses;
    Bottle bottle;

    if ((l_o_r=="right") || (l_o_r=="both"))
    {
        Bottle &bright2=bottle.addList();
        for (size_t i=0; i<poseR.size(); i++)
        {
            bright2.addDouble(poseR[i]);
        }
        poses.put("pose_right", bottle.get(0));

        Bottle &bright1=bottle.addList();
        for (size_t i=0; i<solR.size(); i++)
        {
            bright1.addDouble(solR[i]);
        }
        poses.put("solution_right", bottle.get(1));

        Bottle &bright3=bottle.addList();
        for (size_t i=0; i<trajectory_right.size(); i++)
        {
            Bottle &bb=bright3.addList();
            for (size_t j=0; j<trajectory_right[i].size();j++)
                bb.addDouble(trajectory_right[i][j]);
        }
        poses.put("trajectory_right", bottle.get(2));
    }

    if (l_o_r=="both")
    {
        Bottle &bleft2=bottle.addList();
        for (size_t i=0; i<poseL.size(); i++)
        {
            bleft2.addDouble(poseL[i]);
        }
        poses.put("pose_left", bottle.get(3));

        Bottle &bleft1=bottle.addList();
        for (size_t i=0; i<solL.size(); i++)
        {
            bleft1.addDouble(solL[i]);
        }
        poses.put("solution_left", bottle.get(4));

        Bottle &bright3=bottle.addList();
        for (size_t i=0; i<trajectory_left.size(); i++)
        {
            Bottle &bb=bright3.addList();
            for (size_t j=0; j<trajectory_left[i].size();j++)
                bb.addDouble(trajectory_left[i][j]);
        }
        poses.put("trajectory_left", bottle.get(5));
    }
    if (l_o_r=="left")
    {
        Bottle &bleft2=bottle.addList();
        for (size_t i=0; i<poseL.size(); i++)
        {
            bleft2.addDouble(poseL[i]);
        }
        poses.put("pose_left", bottle.get(0));

        Bottle &bleft1=bottle.addList();
        for (size_t i=0; i<solL.size(); i++)
        {
            bleft1.addDouble(solL[i]);
        }
        poses.put("solution_left", bottle.get(1));

        Bottle &bright3=bottle.addList();
        for (size_t i=0; i<trajectory_left.size(); i++)
        {
            Bottle &bb=bright3.addList();
            for (size_t j=0; j<trajectory_left[i].size();j++)
                bb.addDouble(trajectory_left[i][j]);
        }
        poses.put("trajectory_left", bottle.get(2));
    }

    return poses;
}

/***********************************************************************/
void GraspComputation::setPar(const string &par_name, const string &value)
{
    LockGuard lg(mutex);
    if (par_name=="hand")
        left_or_right=value;
}
