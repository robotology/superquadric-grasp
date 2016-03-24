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
#include "grasping.cpp"

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
    BufferedPort<Bottle>      inPort;
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

    deque<Vector> trajectory;
    Vector pose;

    double tol, constr_viol_tol;
    int max_iter, acceptable_iter, object_provided;
    string mu_strategy,nlp_scaling_method;

    Ipopt::ApplicationReturnStatus status;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app;
    Ipopt::SmartPtr<grasping_NLP>  grasp_nlp;

    Object obj_main;

    bool go_on;


public:

    /************************************************************************/
    GraspingModule()
    {
        graspDisp.resize(3,0.0);
        graspDisp[0]=0.0;
        graspDisp[1]=0.0;
        graspDisp[2]=0.05;

        action=NULL;
        firstRun=true;

    }

    /************************************************************************/
    void getArmDependentOptions(Bottle &b,  Vector &_gDisp)
    {
        if (Bottle *pB=b.find("grasp_displacement").asList())
        {
            int sz=pB->size();
            int len=_gDisp.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gDisp[i]=pB->get(i).asDouble();
        }
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        cout<<"debug 0"<<endl;
        robot=rf.find("robot").asString().c_str();
        if(rf.find("robot").isNull())
        {
            cerr<<"Robot name not provided!"<<endl;
            return false;
        }
        cout<<"debug 1"<<endl;

        left_or_right=rf.find("hand").asString().c_str();
        if(rf.find("hand").isNull())
        {
            //yError("Hand not provided!");
            return false;
        }


        cout<<"debug 2"<<endl;

        Property option_arm("(device cartesiancontrollerclient)");
        option_arm.put("remote","/"+robot+"/cartesianController/"+left_or_right+"_arm");
        option_arm.put("local","/superquadric-grasping/cartesian/"+left_or_right+"_arm");

        robotDevice.open(option_arm);
        if (!robotDevice.isValid())
        {
            //yErr("Device index not available!");
            return false;
        }

        robotDevice.view(icart_arm);

        cout<<"debug 3"<<endl;

        Property option_arm2("(device remote_controlboard)");
        option_arm2.put("remote","/"+robot+"/"+left_or_right+"_arm");
        option_arm2.put("local","/superquadric-grasping/joint/"+left_or_right+"_arm");

        robotDevice2.open(option_arm2);
        if (!robotDevice2.isValid())
        {
            //yErr("Device not available!");
            return false;
        }

        cout<<"debug 4"<<endl;
        robotDevice2.view(enc);
cout<<"debug 5"<<endl;

        bool ok=configAction(rf);
cout<<"debug 6"<<endl;
        configSuperq(rf);
cout<<"debug 7"<<endl;
        pose.resize(6,0.0);

        return ok;

    }

    /****************************************************************/
    bool close()
    {
        if (action!=NULL)
            delete action;

        inPort.close();
        rpcPort.close();

        robotDevice.close();
        robotDevice2.close();

        return true;

    }

    /****************************************************************/
    bool interruptModule()
    {
        inPort.close();
        rpcPort.close();

        robotDevice.close();
        robotDevice2.close();

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
        go_on=computeSuperq();

        if(go_on)
            computeTrajectory();

        go_on=reachPose();

        if(go_on)
            go_on=graspObject();

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool configAction(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        Property config; config.fromConfigFile(rf.findFile("from").c_str());
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
        getArmDependentOptions(bArm,graspDisp);

        action=new AFFACTIONPRIMITIVESLAYER(option);
        if (!action->isValid())
        {
            delete action;
            return false;
        }

        deque<string> q=action->getHandSeqList();
        cout<<"***** List of available hand sequence keys:"<<endl;
        for (size_t i=0; i<q.size(); i++)
            cout<<q[i]<<endl;

        string fwslash="/";
        inPort.open((fwslash+name+"/in").c_str());
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
    void configSuperq(ResourceFinder &rf)
    {
        tol=rf.check("tol",Value(1e-5)).asDouble();
        constr_viol_tol=rf.check("constr_tol",Value(1e-2)).asDouble();
        acceptable_iter=rf.check("acceptable_iter",Value(0)).asInt();
        max_iter=rf.check("max_iter",Value(2e19)).asInt();

        mu_strategy=rf.find("mu_strategy").asString().c_str();
        if(rf.find("mu_strategy").isNull())
            mu_strategy="adaptive";
        nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
        if(rf.find("nlp_scaling_method").isNull())
            nlp_scaling_method="none";
        object_provided=rf.find("object_prov").asInt();
            if(rf.find("object_prov").isNull())
                object_provided=1;

        obj_main.init(rf);
        obj_main.getObject(object_provided,rf);

        app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",tol);
        app->Options()->SetNumericValue("constr_viol_tol",constr_viol_tol);
        app->Options()->SetIntegerValue("acceptable_iter",acceptable_iter);
        app->Options()->SetStringValue("mu_strategy",mu_strategy);
        app->Options()->SetIntegerValue("max_iter",max_iter);
        app->Options()->SetStringValue("nlp_scaling_method",nlp_scaling_method);
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Initialize();

        grasp_nlp= new grasping_NLP;

        grasp_nlp->init(obj_main,rf);
        grasp_nlp->configure(rf);

    }

    /***********************************************************************/
    bool computeSuperq()
    {
        double t,t0;
        t0=Time::now();

        status=app->OptimizeTNLP(GetRawPtr(grasp_nlp));
        t=Time::now()-t0;
        cout<<"t "<<t<<endl;

        if(status==Ipopt::Solve_Succeeded)
        {
            pose=grasp_nlp->get_result();
            cout<<"solution "<<pose.toString()<<endl;
            yInfo("Solution of the optimization problem: %s", pose.toString().c_str());
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool computeTrajectory()
    {
        Vector pose1(6,0.0);
        Vector euler(6,0.0);
        euler[0]=pose[3];
        euler[1]=pose[4];
        euler[2]=pose[5];
        Matrix H_x(4,4);
        H_x=euler2dcm(euler);
        euler[0]=pose[0];
        euler[1]=pose[1];
        euler[2]=pose[2];
        H_x.setSubcol(euler,0,3);

        euler[0]=grasp_nlp->hnd.x[8];
        euler[1]=grasp_nlp->hnd.x[9];
        euler[2]=grasp_nlp->hnd.x[10];
        Matrix H_h2w(4,4);
        H_h2w=euler2dcm(euler);
        euler[0]=grasp_nlp->hnd.x[5];
        euler[1]=grasp_nlp->hnd.x[6];
        euler[2]=grasp_nlp->hnd.x[7];
        H_h2w.setSubcol(euler,0,3);

        Matrix H(4,4);
        H=H_x*H_h2w;

        pose1=pose;
        pose1.setSubvector(0,pose.subVector(0,2)-grasp_nlp->hnd.x[0]*(H.transposed().getCol(2).subVector(0,2)));

        trajectory.push_back(pose1);
        trajectory.push_back(pose);

        return true;
    }

     /***********************************************************************/
    bool reachPose()
    {
        bool f;

        if (firstRun)
        {
            action->pushAction(home_x,"open_hand");
            action->checkActionsDone(f,true);

            firstRun=false;
        }

        Bottle *b=inPort.read();

        if (b!=NULL)
        {
            Vector point(6,0.0);
            point=trajectory[0];

            // grasp (wait until it's done)
            action->grasp(point.subVector(0,2), point.subVector(3,5), graspDisp);
            action->checkActionsDone(f,true);

            point=trajectory[1];

            // grasp (wait until it's done)
            action->grasp(point.subVector(0,2), point.subVector(3,5), graspDisp);
            action->checkActionsDone(f,true);

        }

        return true;

    }

    /***********************************************************************/
   bool graspObject()
   {
       bool f;
       action->pushAction("close_hand");
       action->checkActionsDone(f,true);
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
   rf.setDefault("grasp_model_type","tactile");
   rf.setDefault("grasp_model_file","grasp_model.ini");
   rf.setDefault("hand_sequences_file","hand_sequences.ini");
   rf.setDefault("name","actionPrimitivesMod");
   rf.configure(argc,argv);

   GraspingModule mod;
   return mod.runModule(rf);
}

