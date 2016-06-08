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

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>

#include <opencv2/opencv.hpp>

#include <yarp/os/Port.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>
#include <iCub/perception/sensors.h>
#include <iCub/perception/tactileFingers.h>
#include "computePose.cpp"

#define AFFACTIONPRIMITIVESLAYER    ActionPrimitivesLayer1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;
using namespace iCub::iKin;

class GraspingModule: public RFModule
{
protected:
    AFFACTIONPRIMITIVESLAYER *action;
    Port                      rpcPort;

    Vector graspOrien;
    Vector graspDisp;
    Vector dOffs;
    Vector dLift;
    Vector home_x;

    bool firstRun;

    PolyDriver robotDevice;
    PolyDriver robotDevice2;

    ICartesianControl *icart_arm;
    IEncoders *enc;

    string robot;
    string left_or_right;

    ResourceFinder *rf;


    deque<Vector> trajectory;
    Vector pose, sol;
    double t,t0;

    double tol, constr_viol_tol;
    int max_iter, acceptable_iter, object_provided;
    string mu_strategy,nlp_scaling_method;

    Vector object;
    Vector hand;
    int n_pointshand;
    double distance;

    RpcClient portSuperqRpc;
    string superq_name;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    string eye;
    PolyDriver GazeCtrl;
    IGazeControl *igaze;

    bool go_on;
    bool online;
    bool move;
    bool viewer;

    string nameFileOut, nameFileSolution, nameFileTrajectory;

public:

    /************************************************************************/
    GraspingModule()
    {
        graspDisp.resize(3,0.0);
        graspOrien.resize(4,0.0);
        dOffs.resize(3,0.0);
        dLift.resize(3,0.0);
        home_x.resize(3,0.0);
        graspDisp[0]=0.0;
        graspDisp[1]=0.0;
        graspDisp[2]=0.05;

        graspOrien[0]=-0.171542;
        graspOrien[1]= 0.124396;
        graspOrien[2]=-0.977292;
        graspOrien[3]= 3.058211;

        dOffs[0]=-0.03;
        dOffs[1]=-0.07;
        dOffs[2]=-0.02;

        dLift[0]=0.0;
        dLift[1]=0.0;
        dLift[2]=0.15;

        home_x[0]=-0.29;
        home_x[1]=0.21;
        home_x[2]= 0.11;

        action=NULL;
        firstRun=true;
    }

    /************************************************************************/
    void getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
                                Vector &_dOffs, Vector &_dLift, Vector &_home_x)
    {
        if (Bottle *pB=b.find("grasp_displacement").asList())
        {
            int sz=pB->size();
            int len=_gDisp.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gDisp[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("grasp_orientation").asList())
        {
            int sz=pB->size();
            int len=_gOrien.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gOrien[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("systematic_error_displacement").asList())
        {
            int sz=pB->size();
            int len=_dOffs.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dOffs[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("lifting_displacement").asList())
        {
            int sz=pB->size();
            int len=_dLift.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dLift[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("home_position").asList())
        {
            int sz=pB->size();
            int len=_home_x.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _home_x[i]=pB->get(i).asDouble();
        }
    }

    /****************************************************************/
    bool configDevices(ResourceFinder &rf)
    {
        robot=rf.find("robot").asString().c_str();
        if(rf.find("robot").isNull())
            robot="iCubsim";

        left_or_right=rf.find("which_hand").asString().c_str();
        if(rf.find("which_hand").isNull())
            left_or_right="right";

        Property option_arm("(device cartesiancontrollerclient)");
        option_arm.put("remote","/"+robot+"/cartesianController/"+left_or_right+"_arm");
        option_arm.put("local","/superquadric-grasping/cartesian/"+left_or_right+"_arm");

        robotDevice.open(option_arm);
        if (!robotDevice.isValid())
        {
            yError("Device index not available!");
            return false;
        }

        robotDevice.view(icart_arm);

        Property option_arm2("(device remote_controlboard)");
        option_arm2.put("remote","/"+robot+"/"+left_or_right+"_arm");
        option_arm2.put("local","/superquadric-grasping/joint/"+left_or_right+"_arm");

        robotDevice2.open(option_arm2);
        if (!robotDevice2.isValid())
        {
            yError("Device not available!");
            return false;
        }

        Vector curDof;
        icart_arm->getDOF(curDof);
        cout<<"["<<curDof.toString()<<"]"<<endl;  // [0 0 0 1 1 1 1 1 1 1] will be printed out
        Vector newDof(3);
        newDof[0]=1;    // torso pitch: 1 => enable
        newDof[1]=2;    // torso roll:  2 => skip
        newDof[2]=1;    // torso yaw:   1 => enable
        icart_arm->setDOF(newDof,curDof);
        cout<<"["<<curDof.toString()<<"]"<<endl;  // [1 0 1 1 1 1 1 1 1 1] will be printed out

        return true;
    }

    /****************************************************************/
    bool close()
    {
        if (action!=NULL)
            delete action;

        if (rpcPort.isOpen())
            rpcPort.close();

        if (portSuperqRpc.asPort().isOpen())
            portSuperqRpc.close();

        if (!portImgIn.isClosed())
            portImgIn.close();

        if (!portImgOut.isClosed())
            portImgOut.close();

        robotDevice.close();
        robotDevice2.close();

        return true;

    }

    /****************************************************************/
    bool interruptModule()
    {
        portImgIn.interrupt();

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
        if (norm(hand)!=0.0 && norm(object)!=0.0)
            go_on=computePose();

        if (go_on)
            computeTrajectory();

        if ((go_on==true) && (viewer==true))
            go_on=showTrajectory();

        if ((go_on==true) && (move==true))
            go_on=reachPose();

        if ((go_on==true) && (move==true))
            go_on=graspObject();

        if ((go_on==true) && (move==true))
            go_on=comeBack();

        return !go_on;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool configAction(ResourceFinder &rf)
    {
        robot=rf.find("robot").asString().c_str();
        if(rf.find("robot").isNull())
            robot="iCubsim";

        left_or_right=rf.find("which_hand").asString().c_str();
        if(rf.find("which_hand").isNull())
            left_or_right="right";

        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        Property config;
        config.fromConfigFile(rf.findFile("from").c_str());
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
        {
            cout<<"Error: group general is missing!"<<endl;
            return false;
        }

        Property option(bGeneral.toString().c_str());
        option.put("local",name.c_str());
        option.put("part",left_or_right+"_arm");
        option.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        option.put("grasp_model_file",rf.findFile("grasp_model_file").c_str());
        option.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());

        Bottle &bArm=config.findGroup("arm_dependent");
        getArmDependentOptions(bArm,graspOrien,graspDisp,dOffs,dLift,home_x);

        action=new AFFACTIONPRIMITIVESLAYER(option);
        if (!action->isValid())
        {
            delete action;
            return false;
        }

        deque<string> q=action->getHandSeqList();
        cout<<"*** List of available hand sequence keys:"<<endl;
        for (size_t i=0; i<q.size(); i++)
            cout<<q[i]<<endl;

        string fwslash="/";
        rpcPort.open((fwslash+name+"/rpc").c_str());
        attach(rpcPort);

        Model *model; action->getGraspModel(model);

        if (model!=NULL)
        {
            if (!model->isCalibrated())
            {
                Property prop;
                prop.put("finger","all");
                model->calibrate(prop);
            }
        }
        return true;
    }

    /***********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool config_ok;

        config_ok=configPose(rf);

        viewer=(rf.check("viewer", Value("no"))=="yes");

        if (viewer)
            config_ok=configViewer(rf);

        move=(rf.check("move", Value("no"))=="yes");

        if ((config_ok==true) && (move==1))
        {
            config_ok=configDevices(rf);
            if (config_ok)
            config_ok=configAction(rf);
        }

        return config_ok;
    }

    /***********************************************************************/
    bool configViewer(ResourceFinder &rf)
    {
        portImgIn.open("/superquadric-grasping/img:i");
        portImgOut.open("/superquadric-grasping/img:o");

        eye=rf.check("eye", Value("left")).asString();

        Property optionG;
        optionG.put("device","gazecontrollerclient");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local","/superquadric-grasping/gaze");

        GazeCtrl.open(optionG);
        igaze=NULL;

        if (GazeCtrl.isValid())
            GazeCtrl.view(igaze);
        else
            return false;
    }


    /***********************************************************************/
    bool configPose(ResourceFinder &rf)
    {
        this->rf=&rf;

        online=(rf.check("online", Value("no"))=="yes");
        n_pointshand=rf.check("pointshand", Value(48)).asInt();
        distance=rf.check("distance", Value(0.1)).asDouble();
        superq_name=rf.check("superq_name", Value("box")).asString();

        portSuperqRpc.open("/superquadric-grasping/superq:rpc");

        if (online)
        {
             askForObject();
             getHandPose();
        }
        else
        {
            readSuperq("object",object,11,this->rf);
            readSuperq("hand",hand,11,this->rf);
        }

        nameFileOut=rf.find("nameFileOut").asString().c_str();
        if(rf.find("nameFileOut").isNull())
           nameFileOut="test.txt";

        nameFileSolution=rf.find("nameFileSolution").asString();
        if(rf.find("nameFileSolution").isNull())
           nameFileSolution="solution.txt";

        nameFileTrajectory=rf.find("nameFileTrajectory").asString().c_str();
        if(rf.find("nameFileTrajectory").isNull())
           nameFileOut="test-trajectory.txt";

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

        return true;
    }

    /****************************************************************/
    void askForObject()
    {
        Bottle cmd, reply;
        cmd.addString("get");
        cmd.addString("superq");
        cmd.addString(superq_name);

        if (portSuperqRpc.write(cmd,reply))
        {
            for (int idx=0; idx<reply.size(); idx++)
            {
                object[idx]=reply.get(idx).asDouble();
            }

            yInfo()<<"Object superquadric: "<<object.toString();
        }
        else
        {
            yError()<<"Superquadric not provided!";
        }
    }

    /****************************************************************/
    void getHandPose()
    {
        Vector x(3,0.0);
        Vector o(4,0.0);

        icart_arm->getPose(x,o);

        hand.setSubvector(0,x);
        Matrix H=axis2dcm(o);
        hand.setSubvector(3,dcm2euler(H));

        yInfo()<<"Hand pose: "<<hand.toString();
    }

    /****************************************************************/
    bool readSuperq(const string &name_obj, Vector &x, const int &dimension, ResourceFinder *rf)
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

    /***********************************************************************/
    bool computePose()
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
        app->Options()->SetStringValue("output_file",nameFileOut+".out");
        app->Options()->SetIntegerValue("print_level",5);
        app->Initialize();

        t0=Time::now();
        Ipopt::SmartPtr<grasping_NLP>  grasp_nlp= new grasping_NLP;
        grasp_nlp->init(object, hand, n_pointshand);
        grasp_nlp->configure(this->rf);

        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(grasp_nlp));
        t=Time::now()-t0;

        if(status==Ipopt::Solve_Succeeded)
        {
            sol=grasp_nlp->get_result();
            pose=grasp_nlp->robot_pose;
            cout<<"The optimal robot pose is: "<<pose.toString().c_str()<<endl;
            ofstream fout(nameFileSolution.c_str());

            if(fout.is_open())
            {
                fout<<"Initial hand volume pose: "<<" "<<"["<<grasp_nlp->hand.toString(3,3).c_str()<<"]"<<endl<<endl;

                fout<<"Final hand volume pose: "<<" "<<"["<<sol.toString(3,3).c_str()<<"]"<<endl<<endl;

                fout<<"Final hand pose "<<" "<<"["<<grasp_nlp->robot_pose.toString(3,3)<<"]"<<endl<<endl;

                fout<<"t for ipopt: "<<t<<endl<<endl;

                fout<<"object: "<<" "<<"["<<grasp_nlp->object.toString(3,3)<<"]"<<endl<<endl;

                fout<<"bounds in constrained:"<<endl<<"["<<grasp_nlp->bounds_constr.toString(3,3)<<"]"<<endl<<endl;

                fout<<"plane: ["<<grasp_nlp->plane.toString()<<endl;
            }

            return true;
        }
        else
        {
            cout<<"Problem not solved!"<<endl;
            return false;
        }
    }

    /***********************************************************************/
    bool computeTrajectory()
    {
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
        pose1.setSubvector(0,pose.subVector(0,2)-distance*(H.transposed().getCol(2).subVector(0,2)));
        trajectory.clear();
        trajectory.push_back(pose1);
        trajectory.push_back(pose);

        ofstream fout(nameFileTrajectory.c_str());

        if(fout.is_open())
        {
            for (size_t i=0; i<trajectory.size();i++)
            fout<<"waypoint "<<i<<" "<<trajectory[i].toString()<<endl;
        }

        return true;
    }

    /***********************************************************************/
    bool showTrajectory()
    {
        /****************/
        Bottle info;
        igaze->getInfo(info);
        Matrix K(3,4);
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
        /***************/


        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut=*imgIn;
        //imgOut.resize(imgIn->width(),imgIn->height());

        cv::Mat imgInMat=cv::Mat((IplImage*)imgIn->getIplImage());
        cv::Mat imgOutMat=cv::Mat((IplImage*)imgOut.getIplImage());
        imgInMat.copyTo(imgOutMat);

        PixelRgb color(255,255,0);

        Vector waypoint(6,0.0);
        Vector waypoint2D(2,0.0);

        Vector x(3,0.0);
        Vector y(3,0.0);
        Vector z(3,0.0);
        double length=0.08;
        Vector dir_x(3,0.0);
        Vector dir_y(3,0.0);
        Vector dir_z(3,0.0);
        Vector x2D(2,0.0);
        Vector y2D(2,0.0);
        Vector z2D(2,0.0);

        for (size_t i=0; i<trajectory.size(); i++)
        {
            waypoint=trajectory[i];
            yDebug()<<"waypoint "<<i<<waypoint.toString();

            if (eye=="left")
                igaze->get2DPixel(0,waypoint.subVector(0,2),waypoint2D);
            else
                igaze->get2DPixel(1,waypoint.subVector(0,2),waypoint2D);

            yDebug()<<"2D waypoint "<<i<<waypoint2D.toString();

            Matrix H=euler2dcm(waypoint.subVector(3,5));

            dir_x=H.subcol(0,0,3);
            dir_y=H.subcol(0,1,3);
            dir_z=H.subcol(0,2,3);

            x[0]=waypoint[0]+length*dir_x[0]; x[1]=waypoint[1]+length*dir_x[1]; x[2]=waypoint[2]+length*dir_x[2];
            y[0]=waypoint[0]+length*dir_y[0]; y[1]=waypoint[1]+length*dir_y[1]; y[2]=waypoint[2]+length*dir_y[2];
            z[0]=waypoint[0]+length*dir_z[0]; z[1]=waypoint[1]+length*dir_z[1]; z[2]=waypoint[2]+length*dir_z[2];

            if (eye=="left")
            {
                igaze->get2DPixel(0,x,x2D);
                igaze->get2DPixel(0,y,y2D);
                igaze->get2DPixel(0,z,z2D);
            }
            else
            {
                igaze->get2DPixel(1,x,x2D);
                igaze->get2DPixel(1,y,y2D);
                igaze->get2DPixel(1,z,z2D);
            }

            yDebug()<<"x2D "<<x2D.toString();
            yDebug()<<"y2D "<<y2D.toString();
            yDebug()<<"z2D "<<z2D.toString();


            cv::Point  target_point((int)waypoint2D[0],(int)waypoint2D[1]);
            cv::Point  target_pointx((int)x2D[0],(int)x2D[1]);
            cv::Point  target_pointy((int)y2D[0],(int)y2D[1]);
            cv::Point  target_pointz((int)z2D[0],(int)z2D[1]);

            if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
            {
                yError("Not acceptable pixels!");
            }
            else
                imgOut.pixel(target_point.x, target_point.y)= color;

            if ((target_pointx.x<0) || (target_pointx.y<0) || (target_pointx.x>=320) || (target_pointx.y>=240))
            {
                yError("Not acceptable pixels!");
            }
            else
                cv::line(imgOutMat,target_point,target_pointx,cv::Scalar(255,0,0));

            if ((target_pointy.x<0) || (target_pointy.y<0) || (target_pointy.x>=320) || (target_pointy.y>=240))
            {
                yError("Not acceptable pixels!");
            }
            else
                cv::line(imgOutMat,target_point,target_pointy,cv::Scalar(0,255,0));

            if ((target_pointz.x<0) || (target_pointz.y<0) || (target_pointz.x>=320) || (target_pointz.y>=240))
            {
                yError("Not acceptable pixels!");
            }
            else
                cv::line(imgOutMat,target_point,target_pointz,cv::Scalar(0,0,255));
        }

        portImgOut.write();
    }

    /***********************************************************************/
    bool reachPose()
    {
        bool f;

        if (firstRun)
        {
            action->pushAction("open_hand");
            action->checkActionsDone(f,true);

            firstRun=false;
        }

        Vector point(6,0.0);
        point=trajectory[0];

        Vector orient(4,0.0);
        orient=dcm2axis(euler2dcm(point.subVector(3,5)));

        yDebug()<<"point to be reached: "<<point.subVector(0,2).toString();
        yDebug()<<"orientation: "<<orient.toString();

        // grasp (wait until it's done)
        //action->pushAction(point.subVector(0,2), orient);
        //action->checkActionsDone(f,true);


        icart_arm->goToPose(point.subVector(0,2), orient);
        icart_arm->waitMotionDone(0.02);

        Vector x_tmp(3,0.0);
        Vector o_tmp(4,0.0);
        icart_arm->getPose(x_tmp, o_tmp);

        yDebug()<<"poses "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString();
        showTrajectory();

        point=trajectory[1];
        yDebug()<<"point to be reached: "<<point.subVector(0,2).toString();

        // grasp (wait until it's done)
        //action->pushAction(point.subVector(0,2), orient);
        //action->checkActionsDone(f,true);

        icart_arm->goToPose(point.subVector(0,2), orient);
        icart_arm->waitMotionDone(0.02);
        showTrajectory();

        return true;
    }

    /***********************************************************************/
    bool graspObject()
    {
        bool f;
        action->pushAction("close_hand");
        action->checkActionsDone(f,true);
        firstRun=true;
        return true;
    }

    /***********************************************************************/
    bool comeBack()
    {
        bool f;
        action->pushAction("open_hand");
        action->checkActionsDone(f,true);

        firstRun=false;


        Vector point(6,0.0);
        point=trajectory[1];

        Vector orient(4,0.0);
        orient=dcm2axis(euler2dcm(point.subVector(3,5)));

        icart_arm->goToPose(point.subVector(0,2), orient);
        icart_arm->waitMotionDone(0.02);

        Vector x_tmp(3,0.0);
        Vector o_tmp(4,0.0);
        icart_arm->getPose(x_tmp, o_tmp);

        point=trajectory[0];
        yDebug()<<"point to be reached: "<<point.subVector(0,2).toString();

        icart_arm->goToPose(point.subVector(0,2), orient);
        icart_arm->waitMotionDone(0.02);

        return true;
    }
};

/************************************************************************/
int main(int argc, char *argv[])
{
   Network yarp;
   if (!yarp.checkNetwork())
   {
       cout<<"YARP server not available!"<<endl;
       return 1;
   }

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("config.ini");
   rf.setDefaultContext("superquadric-grasping");
   rf.setDefault("grasp_model_type","tactile");
   rf.setDefault("grasp_model_file","grasp_model.ini");
   rf.setDefault("hand_sequences_file","hand_sequences.ini");
   rf.setDefault("name","actionPrimitivesMod");
   rf.configure(argc,argv);

   GraspingModule mod;
   return mod.runModule(rf);
}

