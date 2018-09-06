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
                                   Vector &_hand, Vector &_hand1, ResourceFinder *_rf,Property &_complete_sol,
                                   const Vector &_object, const deque<Vector> &_obstacles, double &_cost_right, double &_cost_left, bool &_multiple_superq):
                                   ipopt_par(_ipopt_par), pose_par(_pose_par), trajectory_par(_trajectory_par), left_right(_left_or_right), hand(_hand),
                                   hand1(_hand1), rf(_rf), multiple_superq(_multiple_superq), complete_sol(_complete_sol), object(_object), obstacles(_obstacles),
                                   cost_right(_cost_right), cost_left(_cost_left)

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

    int pl=newOptions.find("print_level").asInt();
    if (newOptions.find("print_level").isNull() && (first_time==true))
    {
        print_level=0;
    }
    else if (!newOptions.find("print_level").isNull())
    {
        if ((pl>=0 )&& (pl<=10))
        {
             print_level=pl;
        }
        else if (pl<0 )
        {
            pl=0;
        }
        else if (pl>10)
        {
            print_level=10;
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
    advOptions.put("IPOPT_print_level", print_level);
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
        n_pointshand=49;
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

        displacement=tmp;
    }

    Bottle *pl=newOptions.find("plane").asList();
    if (newOptions.find("plane").isNull() && (first_time==true))
    {
        plane[0]=0.0; plane[1]=0.0; plane[2]=1.0; plane[3]=0.3;
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
            plane[0]=0.0; plane[1]=0.0; plane[2]=1.0; plane[3]=0.3;
        }
    }

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
}

/***********************************************************************/
Property GraspComputation::getPosePar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("n_pointshand",n_pointshand);
    Bottle displValue;
    Bottle &disp_bottle=displValue.addList();
    disp_bottle.addDouble(displacement[0]); disp_bottle.addDouble(displacement[1]);
    disp_bottle.addDouble(displacement[2]);
    advOptions.put("hand_displacement",displValue.get(0));

    Bottle planeValue;
    Bottle &plane_bottle=planeValue.addList();
    plane_bottle.addDouble(plane[0]); plane_bottle.addDouble(plane[1]);
    plane_bottle.addDouble(plane[2]); plane_bottle.addDouble(plane[3]);
    advOptions.put("plane", planeValue.get(0));

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

    if (left_right=="both")
    {
        setCartesianRight();
        setCartesianLeft();
    }
    else if (left_right=="right")
        setCartesianRight();
    else
        setCartesianLeft();

    go_on=false;

    count_file=0;

    return true;
}

/***********************************************************************/
bool GraspComputation::setCartesianRight()
{
    Property option_arm_r("(device cartesiancontrollerclient)");
    option_arm_r.put("remote","/"+robot+"/cartesianController/right_arm");
    option_arm_r.put("local","/superquadric-grasp/cartesian/right_arm");

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

    return true;
}

/***********************************************************************/
bool GraspComputation::setCartesianLeft()
{
    Property option_arm_l("(device cartesiancontrollerclient)");
    option_arm_l.put("remote","/"+robot+"/cartesianController/left_arm");
    option_arm_l.put("local","/superquadric-grasp/cartesian/left_arm");

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
    newDof[0]=1;
    newDof[1]=0;
    newDof[2]=1;
    icart_left->setDOF(newDof,curDof);

    yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

    double min, max;

    yDebug()<<"Torso DOFS "<<curDof.toString(3,3);

    yDebug()<<"Setting max torso pitch";
    icart_left->setLimits(0, 0.0, torso_pitch_max);
    icart_left->getLimits(0, &min, &max);
    yDebug()<<"Get limit of pitch"<<min<<max;
}

/***********************************************************************/
void GraspComputation::run()
{
    t0=Time::now();

    if (norm(hand)!=0.0 && norm(object)!=0.0)
    {
        if (left_right!="both")
            go_on=computePose(hand, left_right);
        else if (left_right=="both" && norm(hand1)!=0.0)
        {
            go_on=computePose(hand, "right");
            bool go_on1=computePose(hand1, "left");
            go_on=((go_on==true) || (go_on1==true));
        }

        if (left_right=="both")
            bestPose();

        count_file++;
        count_file_old=count_file;
    }

    if ((go_on==true))
    {
        if (left_right!="both")
            computeTrajectory(left_right, dir);
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
    stringstream ss;
    ss << count_file;
    string count_file_string=ss.str();

    string context=this->rf->getHomeContextPath().c_str();
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
    app->Options()->SetIntegerValue("print_level",print_level);

    if (print_level > 0)
        app->Options()->SetStringValue("output_file", context+"/ipopt_"+l_o_r+"_"+count_file_string+".out");

    app->Initialize();

    Ipopt::SmartPtr<grasping_NLP>  grasp_nlp= new grasping_NLP;
    grasp_nlp->init(object, which_hand,obstacles, n_pointshand, l_o_r);

    grasp_nlp->configure(this->rf,l_o_r, displacement, plane);

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(grasp_nlp));

    if(status==Ipopt::Solve_Succeeded)
    {
        if (l_o_r=="right")
        {
            solR=grasp_nlp->get_result();
            final_value_R=grasp_nlp->get_final_F();
            final_obstacles_value_R=grasp_nlp->get_final_constr_values();
            poseR=grasp_nlp->robot_pose;
            which_hand=grasp_nlp->get_hand();
            hand_length_right=grasp_nlp->hand[1];

            yInfo()<<"[GraspComputation]: Solution (hand pose) for "<<l_o_r<<" hand is: ";
            yInfo()<<"|| "<<poseR.toString(3,3).c_str();
            yInfo()<<"[GraspComputation]: Stretched hand is: ";
            yInfo()<<"|| "<<which_hand.toString(3,3).c_str();
        }
        else
        {
            solL=grasp_nlp->get_result();
            final_value_L=grasp_nlp->get_final_F();
            final_obstacles_value_L=grasp_nlp->get_final_constr_values();
            poseL=grasp_nlp->robot_pose;
            which_hand=grasp_nlp->get_hand();
            hand_length_left=grasp_nlp->hand[1];
            yInfo()<<"[GraspComputation]: Solution (hand pose) for "<<l_o_r<<" hand is: ";
            yInfo()<<"|| "<<poseL.toString(3,3).c_str();
            yInfo()<<"[GraspComputation]: Stretched hand is: ";
            yInfo()<<"|| "<<which_hand.toString(3,3).c_str();

        }

        return true;
    }
    else
    {
        yError()<<"[GraspComputation]: Problem for "<<l_o_r<<" not solved!";
        if (l_o_r=="right")
        {
            solR.resize(6,0.0);
            poseR.resize(6,0.0);
            cost_right=0.0;
        }

        if (l_o_r=="left")
        {
            solL.resize(6,0.0);
            poseL.resize(6,0.0);
            cost_left=0.0;
        }

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

    Vector pose_rot(6,0.0);
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

    pose_rot=pose;

    if (direction=="z")
    {
        if (chosen_hand=="right")
        {
            pose_rot.setSubvector(0,pose.subVector(0,2)-distance*(H.getCol(2).subVector(0,2)));
        }
        else
        {
            pose_rot.setSubvector(0,pose.subVector(0,2)+distance*(H.getCol(2).subVector(0,2)));
        }
    }
    else if (direction=="xz")
    {
        if (chosen_hand=="right")
        {
            pose_rot.setSubvector(0,pose.subVector(0,2)-distance1*(H.getCol(2).subVector(0,2)));
            pose_rot.setSubvector(0,pose_rot.subVector(0,2)-distance*(H.getCol(0).subVector(0,2)));
        }
        else
        {
            pose_rot.setSubvector(0,pose.subVector(0,2)+distance1*(H.getCol(2).subVector(0,2)));
            pose_rot.setSubvector(0,pose_rot.subVector(0,2)-distance*(H.getCol(0).subVector(0,2)));
        }
    }

    if (chosen_hand=="right")
    {
        trajectory_right.clear();

        pose.setSubvector(0,pose.subVector(0,2));
        trajectory_right.push_back(pose_rot);
        trajectory_right.push_back(pose);
    }
    else
    {
        trajectory_left.clear();

        pose.setSubvector(0,pose.subVector(0,2));
        trajectory_left.push_back(pose_rot);
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
        Bottle &bottle_right_pose=bottle.addList();
        for (size_t i=0; i<poseR.size(); i++)
        {
            bottle_right_pose.addDouble(poseR[i]);
        }
        poses.put("pose_0_right", bottle.get(0));

        Bottle &bottle_right_sol=bottle.addList();
        for (size_t i=0; i<solR.size(); i++)
        {
            bottle_right_sol.addDouble(solR[i]);
        }
        poses.put("solution_0_right", bottle.get(1));

        Bottle &bottle_right_traj=bottle.addList();
        for (size_t i=0; i<trajectory_right.size(); i++)
        {
            Bottle &bb=bottle_right_traj.addList();
            for (size_t j=0; j<trajectory_right[i].size();j++)
                bb.addDouble(trajectory_right[i][j]);
        }
        poses.put("trajectory_0_right", bottle.get(2));

        poses.put("cost_0_right", cost_right);

        poses.put("hand_length_0_right", hand_length_right);

    }

    if (l_o_r=="both")
    {
        Bottle &bottle_left_pose=bottle.addList();
        for (size_t i=0; i<poseL.size(); i++)
        {
            bottle_left_pose.addDouble(poseL[i]);
        }
        poses.put("pose_0_left", bottle.get(3));

        Bottle &bottle_left_sol=bottle.addList();
        for (size_t i=0; i<solL.size(); i++)
        {
            bottle_left_sol.addDouble(solL[i]);
        }
        poses.put("solution_0_left", bottle.get(4));

        Bottle &bottle_left_traj=bottle.addList();
        for (size_t i=0; i<trajectory_left.size(); i++)
        {
            Bottle &bb=bottle_left_traj.addList();
            for (size_t j=0; j<trajectory_left[i].size();j++)
                bb.addDouble(trajectory_left[i][j]);
        }
        poses.put("trajectory_0_left", bottle.get(5));

        poses.put("cost_0_left", cost_left);

        poses.put("hand_length_0_left", hand_length_left);
    }
    if (l_o_r=="left")
    {
        Bottle &bottle_left_pose=bottle.addList();
        for (size_t i=0; i<poseL.size(); i++)
        {
            bottle_left_pose.addDouble(poseL[i]);
        }
        poses.put("pose_0_left", bottle.get(0));

        Bottle &bottle_left_sol=bottle.addList();
        for (size_t i=0; i<solL.size(); i++)
        {
            bottle_left_sol.addDouble(solL[i]);
        }
        poses.put("solution_0_left", bottle.get(1));

        Bottle &bottle_left_traj=bottle.addList();
        for (size_t i=0; i<trajectory_left.size(); i++)
        {
            Bottle &bb=bottle_left_traj.addList();
            for (size_t j=0; j<trajectory_left[i].size();j++)
                bb.addDouble(trajectory_left[i][j]);
        }
        poses.put("trajectory_0_left", bottle.get(2));

        poses.put("cost_0_left", cost_left);

        poses.put("hand_length_0_left", hand_length_left);
    }

    return poses;
}

/***********************************************************************/
void GraspComputation::setPar(const string &par_name, const string &value)
{
    if (par_name=="left_or_right")
        left_right=value;
}

/***********************************************************************/
void GraspComputation::bestPose()
{
    double q_r=0.0;
    double q_l=0.0;
    double average_obstacle_value_r=0.0;
    double average_obstacle_value_l=0.0;

    double w1, w2, w3, w4;

    double error_position_r, error_orientation_r;
    double error_position_l, error_orientation_l;

    if (final_obstacles_value_R.size()>0)
    {
        for (size_t i=0; i<final_obstacles_value_R.size(); i++)
        {
            average_obstacle_value_r += final_obstacles_value_R[i];
        }
        average_obstacle_value_r /= final_obstacles_value_R.size();
    }

    if (final_obstacles_value_L.size()>0)
    {
        for (size_t i=0; i<final_obstacles_value_L.size(); i++)
        {
            average_obstacle_value_l += final_obstacles_value_L[i];
        }
        average_obstacle_value_l /= final_obstacles_value_L.size();
    }

    w1=0.1;
    w2=0.1;
    w3=1.0;

    if (multiple_superq==false)
    {
        w4=0.0;
        average_obstacle_value_l=1;
        average_obstacle_value_r=1;
    }
    else
    {
        if (average_obstacle_value_l>50  || average_obstacle_value_r>50)
            w4=0.0;

        else
            w4=5.0;
    }

    if (norm(poseR)!=0.0)
    {
        w1=1.0;
        w2=1.0;
        w3=1.0;
        w4=1.0;

        Vector x_d = poseR.subVector(0,2);
        Vector o_d = dcm2axis(euler2dcm(poseR.subVector(3,5)));
        Vector x_d_hat, o_d_hat, q_d_hat;

        icart_right->askForPose(x_d, o_d, x_d_hat, o_d_hat, q_d_hat);


        error_position_r=norm(x_d - x_d_hat);

        Matrix tmp = axis2dcm(o_d_hat).submatrix(0,2, 0,2);

        Matrix orientation_error_matrix =  euler2dcm(poseR.subVector(3,5)).submatrix(0,2, 0,2) * tmp.transposed();
        Vector orientation_error_vector = dcm2axis(orientation_error_matrix);

        error_orientation_r=norm(orientation_error_vector.subVector(0,2))* fabs(sin(orientation_error_vector(3)));

        cout<<endl;
        yDebug()<<"|| Pos error right       :"<<error_position_r;
        yDebug()<<"|| Orient error right    :"<<error_orientation_r;

        cost_right=w1*final_value_R + w2*error_position_r + w3*error_orientation_r + w4 /average_obstacle_value_r;
    }
    else
        cost_right=0.0;

    if (norm(poseL)!=0.0)
    {
        w1=1.0;
        w2=1.0;
        w3=1.0;
        w4=1.0;

        Vector x_d=poseL.subVector(0,2);
        Vector o_d=dcm2axis(euler2dcm(poseL.subVector(3,5)));
        Vector x_d_hat, o_d_hat, q_d_hat;

        icart_left->askForPose(x_d, o_d, x_d_hat, o_d_hat, q_d_hat);

        error_position_l=norm(x_d - x_d_hat);

        Matrix tmp = axis2dcm(o_d_hat).submatrix(0,2, 0,2);
        Matrix orientation_error_matrix =  euler2dcm(poseL.subVector(3,5)).submatrix(0,2, 0,2) * tmp.transposed();
        Vector orientation_error_vector = dcm2axis(orientation_error_matrix);

        error_orientation_l=norm(orientation_error_vector.subVector(0,2))* fabs(sin(orientation_error_vector(3)));

        cost_left=w1*final_value_L + w2*error_position_l + w3*error_orientation_l + w4 / average_obstacle_value_l;

        cout<<endl;
        yDebug()<<"|| Pos error left        :"<<error_position_l;
        yDebug()<<"|| Orient error left     :"<<error_orientation_l;
    }
    else
        cost_left=0.0;

    cout<<endl;
    yInfo()<<"|| Final value r              :"<<final_value_R;
    yInfo()<<"|| w1 * final value r         :"<<w1*final_value_R;
    yInfo()<<"|| obstacle r                 :"<<average_obstacle_value_r;
    yInfo()<<"|| w4 / obstacle r            :"<<w4 / average_obstacle_value_r;
    yInfo()<<"|| w2 * error_posit  r        :"<<w2*error_position_r;
    yInfo()<<"|| w3 * error_orient r        :"<<w3*error_orientation_r;
    yInfo()<<"|| cost right                 :"<<cost_right;

    cout<<endl;
    cout<<endl;
    yInfo()<<"|| Final value l              :"<<final_value_L;
    yInfo()<<"|| w1 * final value l         :"<<w1*final_value_L;
    yInfo()<<"|| obstacle l                 :"<<average_obstacle_value_l;
    yInfo()<<"|| w4 / obstacle l            :"<<w4 / average_obstacle_value_l;
    yInfo()<<"|| w2 * error_posit  l        :"<<w2*error_position_l;
    yInfo()<<"|| w3 * error_orient l        :"<<w3*error_orientation_l;
    yInfo()<<"|| cost left                  :"<<cost_left;
    cout<<endl;
    cout<<endl;


    if ((cost_right<=cost_left) && (cost_right>0.0) && (cost_left>0.))
    {
        yInfo()<<"||  Best pose for grasping is right hand";
        best_hand="right";
    }
    else if ((cost_left<=cost_right) && (cost_right!=0.0) && (cost_left!=0.0))
    {
        yInfo()<<"||  Best pose for grasping is left hand";
        best_hand="left";
    }
    else if (cost_right>0.0)
    {
        yInfo()<<"||  Best pose for grasping is right hand";
        best_hand="right";
    }
    else if (cost_left>0.0)
    {
        yInfo()<<"||  Best pose for grasping is left hand";
        best_hand="left";
    }

}
