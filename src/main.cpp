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
#include "src/superquadricGrasping_IDL.h"

#define AFFACTIONPRIMITIVESLAYER    ActionPrimitivesLayer1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;
using namespace iCub::iKin;

class GraspingModule: public RFModule,
                      public superquadricGrasping_IDL
{
protected:
    AFFACTIONPRIMITIVESLAYER *action;
    AFFACTIONPRIMITIVESLAYER *action2;
    RpcServer                 portRpc;

    Vector graspOrienR;
    Vector graspDispR;
    Vector dOffsR;
    Vector dLiftR;
    Vector home_xR;

    Vector graspOrienL;
    Vector graspDispL;
    Vector dOffsL;
    Vector dLiftL;
    Vector home_xL;

    bool firstRun;

    PolyDriver robotDevice;
    PolyDriver robotDevice2;
    PolyDriver robotDevice3;
    PolyDriver robotDevice4;

    ICartesianControl *icart_arm;
    ICartesianControl *icart_arm2;
    IEncoders *enc;

    string robot;
    string left_or_right;

    ResourceFinder *rf;

    deque<Vector> trajectory;
    Vector poseR, solR;
    Vector poseL, solL;
    Vector pose_tmp, pose_tmp2;
    double t,t0;

    double tol, constr_viol_tol;
    int max_iter, acceptable_iter, object_provided;
    string mu_strategy,nlp_scaling_method;

    Vector object;
    Vector hand, hand1;
    int n_pointshand;
    double distance;
    string dir;
    Vector displacement;

    RpcClient portSuperqRpc;
    RpcClient portCalibCamRpc;
    string superq_name;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    string eye;
    PolyDriver GazeCtrl;
    IGazeControl *igaze;
    Matrix K,H;

    bool go_on;
    bool online;
    bool move;
    bool viewer;
    bool stop_var;
    bool conf_dev_called;
    bool chosen_pose;
    bool conf_act_called;
    bool calib_cam;
    bool lift;

    string nameFileOut_right, nameFileSolution_right, nameFileTrajectory;
    string nameFileOut_left, nameFileSolution_left;
    string chosen_hand;

public:
    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /************************************************************************/
    bool start()
    {
        go_on=true;
        stop_var=false;
        return true;
    }

    /************************************************************************/
    bool stop()
    {
        stop_var=true;
        return true;
    }

    /************************************************************************/
    bool go_home(const string &hand)
    {
        bool f;
        f=false;
        action->pushAction("karate_hand");
        action->checkActionsDone(f,true);

        if (hand=="right")
        {
            action->pushAction(home_xR);
            action->checkActionsDone(f,true);
        }
        else if (hand=="left" && (left_or_right!="both"))
        {
            action->pushAction(home_xL);
            action->checkActionsDone(f,true);
        }
        else if (hand=="left" && (left_or_right=="both"))
        {
            action2->pushAction("karate_hand");
            action2->checkActionsDone(f,true);

            action2->pushAction(home_xL);
            action2->checkActionsDone(f,true);
        }

        return f;
    }

    /************************************************************************/
    string get_lift_object()
    {
        string value;

        if (lift)
            value="yes";
         else
            value="no";
        return value;
    }

    /************************************************************************/
    bool lift_object(const string &lift_or_not)
    {
        lift=(lift_or_not=="yes");
        return true;
    }

    /************************************************************************/
    bool calibrate_cam(const string &calibration_or_not)
    {
        if (calibration_or_not=="yes")
            calib_cam=true;
        else
            calib_cam=false;

        return true;
    }

    /************************************************************************/
    string get_calibrate_cam()
    {
        string value;

        if (calib_cam)
            value="yes";
         else
            value="no";
        return value;
    }

    /************************************************************************/
    bool clear_poses()
    {
        poseR.resize(6,0.0);
        poseL.resize(6,0.0);
        object.resize(11,0.0);
        chosen_pose=false;
        go_on=true;
        stop_var=false;
        cout<<"go_on "<<go_on<<endl;
        return true;
    }

    /************************************************************************/
    bool grasping_method(const string &precision_or_power)
    {
        if (precision_or_power=="power")
        {
            hand[0]=0.03; hand[1]=0.05; hand[2]=0.03;
            if (left_or_right=="both")
            {
                hand1[0]=0.03; hand1[1]=0.05; hand1[2]=0.03;
            }

            yInfo()<<"New hand: "<<hand.toString();
            if (left_or_right=="both")
                yInfo()<<"New other hand : "<<hand1.toString();

            return true;
        }
        else if (precision_or_power=="precision")
        {
            hand[0]=0.08; hand[1]=0.05; hand[2]=0.08;
            if (left_or_right=="both")
            {
                hand1[0]=0.08; hand1[1]=0.05; hand1[2]=0.08;
            }

            yInfo()<<"New hand: "<<hand.toString();
            if (left_or_right=="both")
                yInfo()<<"New other hand : "<<hand1.toString();

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    string get_grasping_method()
    {
        if (hand[0]==0.03)
            return "power";
        else
            return "precision";
    }

    /************************************************************************/
    bool choose_hand(const string &str_hand)
    {
        chosen_pose=true;
        chosen_hand=str_hand;

        yInfo()<<"Chosen hand: "<<chosen_hand;

        return true;
    }

    /************************************************************************/
    string get_chosen_hand()
    {
        return chosen_hand;
    }

    /************************************************************************/
    bool hand_displacement(const Vector &disp)
    {
        yError()<<"disp size "<<disp.size();
        if (disp.size()>=1)
            displacement[0]=disp[0];
        else
            return false;

        if (disp.size()>=2)
            displacement[1]=disp[1];
        if (disp.size()>=3)
            displacement[2]=disp[2];

        yInfo()<<"Hand displacement: "<<displacement.toString();

        return true;
    }

    /************************************************************************/
    vector<double> get_hand_displacement()
    {
        vector<double> hand_displ;
        hand_displ.clear();

        for (size_t i=0; i<3; i++)
            hand_displ.push_back(displacement[i]);

        return hand_displ;
    }

    /************************************************************************/
    bool set_trajectory_options(const Property &options)
    {
        Bottle &groupBottle=options.findGroup("grasp_displacement_r");

        if (!groupBottle.isNull())
        {
            if (Bottle *pB=groupBottle.get(1).asList())
            {
                int sz=pB->size();
                int len=graspDispR.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    graspDispR[i]=pB->get(i).asDouble();

                yInfo()<<"new grasp_displacement right "<<graspDispR.toString();
            }
        }

        Bottle &groupBottle1=options.findGroup("grasp_orientation_r");

        if (!groupBottle1.isNull())
        {
            if (Bottle *pB=groupBottle1.get(1).asList())
            {
                int sz=pB->size();
                int len=graspOrienR.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    graspOrienR[i]=pB->get(i).asDouble();

                yInfo()<<"new grasp_orientation right "<<graspOrienR.toString();
            }
        }

        Bottle &groupBottle2=options.findGroup("doffs_r");

        if (!groupBottle2.isNull())
        {
            if (Bottle *pB=groupBottle2.get(1).asList())
            {
                int sz=pB->size();
                int len=dOffsR.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    dOffsR[i]=pB->get(i).asDouble();

                yInfo()<<"new doff right "<<dOffsR.toString();
            }
        }

        Bottle &groupBottle3=options.findGroup("home_x_r");

        if (!groupBottle3.isNull())
        {
            if (Bottle *pB=groupBottle3.get(1).asList())
            {
                int sz=pB->size();
                int len=home_xR.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    home_xR[i]=pB->get(i).asDouble();

                yInfo()<<"new home_x right "<<home_xR.toString();
            }
        }

        Bottle &groupBottle4=options.findGroup("dlift_r");

        if (!groupBottle4.isNull())
        {
            if (Bottle *pB=groupBottle4.get(1).asList())
            {
                int sz=pB->size();
                int len=dLiftR.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    dLiftR[i]=pB->get(i).asDouble();

                yInfo()<<"new dlift right "<<dLiftR.toString();
            }
        }

        Bottle &groupBottle5=options.findGroup("grasp_displacement_l");

        if (!groupBottle5.isNull())
        {
            if (Bottle *pB=groupBottle5.get(1).asList())
            {
                int sz=pB->size();
                int len=graspDispL.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    graspDispL[i]=pB->get(i).asDouble();

                yInfo()<<"new grasp_displacement right "<<graspDispL.toString();
            }
        }

        Bottle &groupBottle6=options.findGroup("grasp_orientation_l");

        if (!groupBottle6.isNull())
        {
            if (Bottle *pB=groupBottle6.get(1).asList())
            {
                int sz=pB->size();
                int len=graspOrienL.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    graspOrienL[i]=pB->get(i).asDouble();

                yInfo()<<"new grasp_orientation left "<<graspOrienL.toString();
            }
        }

        Bottle &groupBottle7=options.findGroup("doffs_l");

        if (!groupBottle7.isNull())
        {
            if (Bottle *pB=groupBottle7.get(1).asList())
            {
                int sz=pB->size();
                int len=dOffsL.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    dOffsL[i]=pB->get(i).asDouble();

                yInfo()<<"new doff left "<<dOffsL.toString();
            }
        }

        Bottle &groupBottle8=options.findGroup("home_x_l");

        if (!groupBottle8.isNull())
        {
            if (Bottle *pB=groupBottle8.get(1).asList())
            {
                int sz=pB->size();
                int len=home_xL.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    home_xL[i]=pB->get(i).asDouble();

                yInfo()<<"new home_x left "<<home_xL.toString();
            }
        }

        Bottle &groupBottle9=options.findGroup("dlift_l");

        if (!groupBottle9.isNull())
        {
            if (Bottle *pB=groupBottle9.get(1).asList())
            {
                int sz=pB->size();
                int len=dLiftL.length();
                int l=len<sz?len:sz;

                for (int i=0; i<l; i++)
                    dLiftL[i]=pB->get(i).asDouble();

                yInfo()<<"new dlift left "<<dLiftL.toString();
            }
        }

        if ((groupBottle.isNull()) && (groupBottle2.isNull()) && (groupBottle3.isNull()) && (groupBottle4.isNull())  &&
                (groupBottle5.isNull()) && (groupBottle6.isNull()) && (groupBottle7.isNull()) && (groupBottle8.isNull()))
        {
            return false;
        }
        else
            return true;
    }

    /************************************************************************/
    GraspingModule()
    {
        graspDispR.resize(3,0.0);
        graspOrienR.resize(4,0.0);
        dOffsR.resize(3,0.0);
        dLiftR.resize(3,0.0);
        home_xR.resize(3,0.0);
        graspDispL.resize(3,0.0);
        graspOrienL.resize(4,0.0);
        dOffsL.resize(3,0.0);
        dLiftL.resize(3,0.0);
        home_xL.resize(3,0.0);

        graspDispR[0]=0.0;
        graspDispR[1]=0.0;
        graspDispR[2]=0.01;

        graspOrienR[0]=-0.171542;
        graspOrienR[1]= 0.124396;
        graspOrienR[2]=-0.977292;
        graspOrienR[3]= 3.058211;

        dOffsR[0]=-0.03;
        dOffsR[1]=-0.07;
        dOffsR[2]=-0.02;

        dLiftR[0]=0.0;
        dLiftR[1]=0.0;
        dLiftR[2]=0.15;

        home_xR[0]=-0.29;
        home_xR[1]=0.21;
        home_xR[2]= 0.11;

        graspDispL[0]=0.0;
        graspDispL[1]=0.0;
        graspDispL[2]=0.0;

        graspOrienL[0]=-0.171542;
        graspOrienL[1]= 0.124396;
        graspOrienL[2]=-0.977292;
        graspOrienL[3]= 3.058211;

        dOffsL[0]=-0.03;
        dOffsL[1]=-0.07;
        dOffsL[2]=-0.02;

        dLiftL[0]=0.0;
        dLiftL[1]=0.0;
        dLiftL[2]=0.15;

        home_xL[0]=-0.29;
        home_xL[1]=-0.21;
        home_xL[2]= 0.11;

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
    bool configBasics(ResourceFinder &rf)
    {
        robot=rf.find("robot").asString().c_str();
        if(rf.find("robot").isNull())
            robot="iCubsim";

        left_or_right=rf.find("which_hand").asString().c_str();
        if(rf.find("which_hand").isNull())
            left_or_right="right";

        conf_dev_called=false;
        chosen_pose==true;
        conf_act_called=false;
        go_on=true;
        stop_var=false;
        calib_cam=(rf.check("calib_cam", Value("yes"))=="yes");

        lift=rf.find("lift").asString().c_str();
        if(rf.find("lift").isNull())
            lift=true;

        portRpc.open("/superquadric-grasping/rpc");
        attach(portRpc);

        displacement.resize(3,0.0);
        dir=rf.check("approaching_direction", Value("z")).asString();

        return true;
    }    

    /****************************************************************/
    bool close()
    {
        if (portRpc.asPort().isOpen())
            portRpc.close();

        if (portSuperqRpc.asPort().isOpen())
            portSuperqRpc.close();

        if (!portImgIn.isClosed())
            portImgIn.close();

        if (!portImgOut.isClosed())
            portImgOut.close();

        if (move==true)
        {
            if (action!=NULL)
                delete action;

            if ((action2!=NULL) && (left_or_right=="both"))
                delete action2;

            robotDevice.close();
            robotDevice2.close();

            if (left_or_right=="both")
            {
                robotDevice3.close();
                robotDevice4.close();
            }
        }

        if (viewer==true)
            GazeCtrl.close();

        return true;
    }

    /****************************************************************/
    bool interruptModule()
    {
        if (move==true)
        {
            action->syncCheckInterrupt(true);
            if (left_or_right=="both")
                action2->syncCheckInterrupt(true);
        }
        portImgIn.interrupt();

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
        if (stop_var==true)
            go_on=false;

        if (online && norm(object)==0.0)
            askForObject(superq_name);

        if (norm(hand)!=0.0 && norm(object)!=0.0 && (go_on==true) && (norm(poseR)==0.0) && (norm(poseL)==0.0))
        {                        
            yDebug()<<"It's computing pose ...";
            if (left_or_right!="both")
                go_on=computePose(hand, left_or_right);
            else
            {
                go_on=computePose(hand, "right");
                bool go_on1=computePose(hand1, "left");
                go_on=((go_on==true) || (go_on1==true));
            }

            if (norm(poseR)!=0.0)
                pose_tmp=poseR;
            if (norm(poseL)!=0.0)
                pose_tmp2=poseL;
        }
        else
        {
            poseR=pose_tmp;
            poseL=pose_tmp2;
        }

        if ((go_on==true) && (viewer==true))
        {            
            if (left_or_right=="both")
            {
                go_on=showPoses(poseR,poseL,2,0);
            }
            else if (left_or_right=="right")
                go_on=showPoses(poseR,poseL,1,0);
            else
            {
                go_on=showPoses(poseL,poseL,1,0);
            }
        }

        if ((go_on==true) && (calib_cam==true))
        {
            if (norm(poseR)!=0.0)
            {
                poseR=calibCameras(pose_tmp);
            }
            if (norm(poseL)!=0.0)
                poseL=calibCameras(pose_tmp2);
        }

        if ((go_on==true) && (viewer==true) && (calib_cam==true))
        {
            if (left_or_right=="both")
            {
                go_on=showPoses(poseR,poseL,2,100);
                go_on=showPoses(pose_tmp,pose_tmp2,2,0);
            }
            else if (left_or_right=="right")
            {
                go_on=showPoses(poseR,poseR,1,100);
                go_on=showPoses(pose_tmp,pose_tmp2,1,0);
            }
            else
            {
                go_on=showPoses(poseL,poseL,1,100);
                go_on=showPoses(pose_tmp,pose_tmp2,1,0);
            }
        }

        if (chosen_pose)
        {
            /**if ((go_on==true) && (calib_cam==true))
            {
                if (chosen_hand =="right")
                    poseR=calibCameras(pose_tmp);
                else
                    poseL=calibCameras(pose_tmp2);
            }*/

            if ((go_on==true))
                computeTrajectory(chosen_hand, dir);

            if (stop_var==true)
                go_on=false;

            if ((go_on==true) && (viewer==true))
                go_on=showTrajectory();

            for (size_t i=0; i<trajectory.size();i++)
            {
                if (stop_var==true)
                    go_on=false;

                if ((go_on==true) && (move==true))
                {
                    go_on=reachPose(i);
                }
            }

            if (stop_var==true)
                go_on=false;

            if ((go_on==true) && (move==true))
            {
                go_on=graspObject();

                if (stop_var==true)
                    go_on=false;

                if (go_on==true && lift==true)
                    liftObject();
            }

            if (stop_var==true)
                go_on=false;

            if ((go_on==true) && (move==true))
                go_on=comeBack();

            if (stop_var==true)
                go_on=true;
        }

        return go_on;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.3;
    }

    /****************************************************************/
    bool configDevices(ResourceFinder &rf, const string &arm)
    {
        if (conf_dev_called)
        {
            Property option_arm3("(device cartesiancontrollerclient)");
            option_arm3.put("remote","/"+robot+"/cartesianController/"+arm+"_arm");
            option_arm3.put("local","/superquadric-grasping/cartesian/"+arm+"_arm");

            robotDevice3.open(option_arm3);
            if (!robotDevice3.isValid())
            {
                yError("Device index not available!");
                return false;
            }

            robotDevice3.view(icart_arm2);

            Property option_arm4("(device remote_controlboard)");
            option_arm4.put("remote","/"+robot+"/"+arm+"_arm");
            option_arm4.put("local","/superquadric-grasping/joint/"+arm+"_arm");

            robotDevice4.open(option_arm4);
            if (!robotDevice4.isValid())
            {
                yError("Device not available!");
                return false;
            }
        }
        else
        {
            Property option_arm("(device cartesiancontrollerclient)");
            option_arm.put("remote","/"+robot+"/cartesianController/"+arm+"_arm");
            option_arm.put("local","/superquadric-grasping/cartesian/"+arm+"_arm");


            robotDevice.open(option_arm);
            if (!robotDevice.isValid())
            {
                yError("Device index not available!");
                return false;
            }

            robotDevice.view(icart_arm);

            Property option_arm2("(device remote_controlboard)");
            option_arm2.put("remote","/"+robot+"/"+arm+"_arm");
            option_arm2.put("local","/superquadric-grasping/joint/"+arm+"_arm");

            robotDevice2.open(option_arm2);
            if (!robotDevice2.isValid())
            {
                yError("Device not available!");
                return false;
            }

            //getHandPose();

            Vector curDof;
            icart_arm->getDOF(curDof);
            cout<<"["<<curDof.toString()<<"]"<<endl;  // [0 0 0 1 1 1 1 1 1 1] will be printed out
            Vector newDof(3);
            newDof[0]=1;    // torso pitch: 1 => enable
            newDof[1]=2;    // torso roll:  2 => skip
            newDof[2]=1;    // torso yaw:   1 => enable
            icart_arm->setDOF(newDof,curDof);
            cout<<"["<<curDof.toString()<<"]"<<endl;  // [1 0 1 1 1 1 1 1 1 1] will be printed out
        }

        conf_dev_called=true;

        return true;
    }

    /****************************************************************/
    bool configAction(ResourceFinder &rf, const string &l_o_r)
    {
        if (conf_act_called)
        {
            string name=rf.find("name").asString().c_str();
            setName(name.c_str());

            Property config;
            config.fromConfigFile(rf.findFile("from").c_str());
            Bottle &bGeneral=config.findGroup("general");

            Property option2(bGeneral.toString().c_str());
            option2.put("local",name.c_str());
            option2.put("part",l_o_r+"_arm");
            option2.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
            option2.put("grasp_model_file",rf.findFile("grasp_model_file_"+l_o_r).c_str());
            option2.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());

            Bottle &bArm=config.findGroup("arm_dependent");
            if (l_o_r=="right")
                getArmDependentOptions(bArm,graspOrienR,graspDispR,dOffsR,dLiftR,home_xR);
            else
                getArmDependentOptions(bArm,graspOrienL,graspDispL,dOffsL,dLiftL,home_xL);

            action2=new AFFACTIONPRIMITIVESLAYER(option2);
            if (!action2->isValid())
            {
                delete action2;
                return false;
            }

            deque<string> q=action2->getHandSeqList();
            cout<<"*** List of available hand sequence keys:"<<endl;
            for (size_t i=0; i<q.size(); i++)
                cout<<q[i]<<endl;

            Model *model2; action2->getGraspModel(model2);

            if (model2!=NULL)
            {
               if (!model2->isCalibrated())
               {
                   Property prop2;
                   prop2.put("finger","all");
                   model2->calibrate(prop2);
               }
            }
        }
        else
        {
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
            option.put("part",l_o_r+"_arm");
            option.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
            option.put("grasp_model_file",rf.findFile("grasp_model_file_"+l_o_r).c_str());
            option.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());

            Bottle &bArm=config.findGroup("arm_dependent");
            if (l_o_r=="right")
                getArmDependentOptions(bArm,graspOrienR,graspDispR,dOffsR,dLiftR,home_xR);

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

            conf_act_called=true;
        }

        showTrajectory();
        return true;
    }

    /***********************************************************************/
    bool configViewer(ResourceFinder &rf)
    {
        portImgIn.open("/superquadric-grasping/img:i");
        portImgOut.open("/superquadric-grasping/img:o");
        portCalibCamRpc.open("/superquadric-grasping/camcalib:rpc");

        eye=rf.check("eye", Value("left")).asString();

        Property optionG;
        optionG.put("device","gazecontrollerclient");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local","/superquadric-grasping/gaze");

        GazeCtrl.open(optionG);
        igaze=NULL;

        if (GazeCtrl.isValid())
        {
            GazeCtrl.view(igaze);
        }
        else
            return false;

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

        return true;
    }

    /***********************************************************************/
    bool configPose(ResourceFinder &rf)
    {
        this->rf=&rf;

        poseR.resize(6,0.0);
        poseL.resize(6,0.0);

        online=(rf.check("online", Value("no"))=="yes");
        n_pointshand=rf.check("pointshand", Value(48)).asInt();
        distance=rf.check("distance", Value(0.05)).asDouble();
        superq_name=rf.check("superq_name", Value("Sponge")).asString();

        portSuperqRpc.open("/superquadric-grasping/superq:rpc");

        if (!online)
        {
            readSuperq("object",object,11,this->rf);
        }
        else
            object.resize(11,0.0);

        readSuperq("hand",hand,11,this->rf);


        cout<<"left or oright "<<left_or_right<<endl;
        if (left_or_right=="both")
        {
            readSuperq("hand1",hand1,11,this->rf);
        }

        nameFileOut_right=rf.find("nameFileOut_right").asString().c_str();
        if(rf.find("nameFileOut_right").isNull())
           nameFileOut_right="test_right";

        nameFileSolution_right=rf.find("nameFileSolution_right").asString();
        if(rf.find("nameFileSolution_right").isNull())
           nameFileSolution_right="solution_right.txt";

        nameFileTrajectory=rf.find("nameFileTrajectory_right").asString().c_str();
        if(rf.find("nameFileTrajectory_right").isNull())
           nameFileTrajectory="test-trajectory_right.txt";

        nameFileOut_left=rf.find("nameFileOut_left").asString().c_str();
        if(rf.find("nameFileOut_left").isNull())
           nameFileOut_left="test_left";

        nameFileSolution_left=rf.find("nameFileSolution_left").asString();
        if(rf.find("nameFileSolution_left").isNull())
           nameFileSolution_left="solution_left.txt";

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

    /***********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool config_ok;
        bool config_ok1;

        config_ok=configBasics(rf);

        if (config_ok==true)
            config_ok=configPose(rf);

        viewer=(rf.check("viewer", Value("no"))=="yes");

        if (viewer)
            config_ok=configViewer(rf);

        move=(rf.check("move", Value("no"))=="yes");

        if ((config_ok==true) && (move==1))
        {            
            if (left_or_right=="both")
            {
                config_ok=configDevices(rf, "right");
                config_ok1=configDevices(rf, "left");

                config_ok=(config_ok==true && config_ok1==true);
            }
            else
                 config_ok=configDevices(rf, left_or_right);

            if (config_ok)
            {
                if (left_or_right=="both")
                {
                    config_ok=configAction(rf, "right");
                    config_ok=configAction(rf, "left");
                }
                else
                    config_ok=configAction(rf, left_or_right);
            }
        }

        cout<<"config_ok "<<config_ok<<endl;

        return config_ok;
    }

    /****************************************************************/
    void askForObject(const string &obj_name)
    {
        Bottle cmd, reply;        
        cmd.addString("get_superq");
        cmd.addString(obj_name);

        portSuperqRpc.write(cmd,reply);

        if (reply.size()>0)
        {
            if (Bottle *b=reply.get(0).asList())
            {
                for (int idx=0; idx<b->size(); idx++)
                {
                    object[idx]=b->get(idx).asDouble();
                }

                yInfo()<<" Object superquadric received: "<<object.toString();

            }
        }
        else
        {
            yError()<<" Superquadric not provided!";
        }
    }

    /****************************************************************/
    void getHandPose()
    {
        Vector x(3,0.0);
        Vector o(4,0.0);

        icart_arm->getPose(x,o);

        hand.setSubvector(5,x);
        Matrix H=axis2dcm(o);
        hand.setSubvector(8,dcm2euler(H));

        yInfo()<<" Hand pose: "<<hand.toString();
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

    /****************************************************************/
    Vector calibCameras(Vector &v)
    {
        Bottle cmd,reply;
        cmd.clear();
        cmd.addString("getPoint");
        cmd.addString(eye);
        Vector v_calib;
        v_calib=v;

        yDebug()<<"Vector before calibration: "<<v.toString();

        cmd.addDouble(v[0]);
        cmd.addDouble(v[1]);
        cmd.addDouble(v[2]);

        portCalibCamRpc.write(cmd, reply);
        if (reply.get(0).asString()=="ok")
        {
            v_calib[0]=reply.get(1).asDouble();
            v_calib[1]=reply.get(2).asDouble();
            v_calib[2]=reply.get(3).asDouble();
        }

        yDebug()<<"Vector after calibration: "<<v_calib.toString();
        return v_calib;
    }

    /***********************************************************************/
    bool computePose(Vector &which_hand, const string &l_o_r)
    {
        yInfo()<<"******** Computing pose for "<<l_o_r<<" hand....";
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
        if (l_o_r=="right")
            app->Options()->SetStringValue("output_file",nameFileOut_right+".out");
        else
            app->Options()->SetStringValue("output_file",nameFileOut_left+".out");
        app->Options()->SetIntegerValue("print_level",0);
        app->Initialize();

        t0=Time::now();
        Ipopt::SmartPtr<grasping_NLP>  grasp_nlp= new grasping_NLP;
        grasp_nlp->init(object, which_hand, n_pointshand, l_o_r);
        grasp_nlp->configure(this->rf,l_o_r, displacement);

        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(grasp_nlp));
        t=Time::now()-t0;

        if(status==Ipopt::Solve_Succeeded)
        {
            if (l_o_r=="right")
            {
                solR=grasp_nlp->get_result();
                poseR=grasp_nlp->robot_pose;
                cout<<"The optimal robot pose for "<<l_o_r<<" hand is: "<<poseR.toString().c_str()<<endl;
                ofstream fout(nameFileSolution_right.c_str());

                if(fout.is_open())
                {
                    fout<<"Initial "<<l_o_r<<" hand volume pose: "<<" "<<"["<<grasp_nlp->hand.toString(3,3).c_str()<<"]"<<endl<<endl;

                    fout<<"Final  "<<l_o_r<<" hand volume pose: "<<" "<<"["<<solR.toString(3,3).c_str()<<"]"<<endl<<endl;

                    fout<<"Final "<<l_o_r<<" hand pose "<<" "<<"["<<poseR.toString(3,3)<<"]"<<endl<<endl;

                    fout<<"t for ipopt: "<<t<<endl<<endl;

                    fout<<"object: "<<" "<<"["<<grasp_nlp->object.toString(3,3)<<"]"<<endl<<endl;

                    fout<<"bounds in constrained:"<<endl<<"["<<grasp_nlp->bounds_constr.toString(3,3)<<"]"<<endl<<endl;

                    fout<<"plane: ["<<grasp_nlp->plane.toString()<<endl;
                }
            }
            else
            {
                solL=grasp_nlp->get_result();
                poseL=grasp_nlp->robot_pose;
                cout<<"The optimal robot pose for "<<l_o_r<<" hand is: "<<poseL.toString().c_str()<<endl;

                ofstream fout(nameFileSolution_left.c_str());

                if(fout.is_open())
                {
                    fout<<"Initial "<<l_o_r<<" hand volume pose: "<<" "<<"["<<grasp_nlp->hand.toString(3,3).c_str()<<"]"<<endl<<endl;

                    fout<<"Final "<<l_o_r<<" hand volume pose: "<<" "<<"["<<solL.toString(3,3).c_str()<<"]"<<endl<<endl;

                    fout<<"Final "<<l_o_r<<" hand pose "<<" "<<"["<<poseL.toString(3,3)<<"]"<<endl<<endl;

                    fout<<"t for ipopt: "<<t<<endl<<endl;

                    fout<<"object: "<<" "<<"["<<grasp_nlp->object.toString(3,3)<<"]"<<endl<<endl;

                    fout<<"bounds in constrained:"<<endl<<"["<<grasp_nlp->bounds_constr.toString(3,3)<<"]"<<endl<<endl;

                    fout<<"plane: ["<<grasp_nlp->plane.toString()<<endl;
                }
            }

            chosen_pose=false;
            return true;
        }
        else
        {
            chosen_pose=false;
            cout<<"Problem not solved!"<<endl;
            return false;
        }
    }

    /***********************************************************************/
    bool computeTrajectory(const string &chosen_hand, const string &direction)
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
        else
        {
            if (chosen_hand=="right")
            {
                pose1.setSubvector(0,pose.subVector(0,2)-distance*(H.getCol(2).subVector(0,2)));
                pose1.setSubvector(0,pose1.subVector(0,2)-distance/2*(H.getCol(0).subVector(0,2)));
            }
            else
            {
                pose1.setSubvector(0,pose.subVector(0,2)+distance*(H.getCol(2).subVector(0,2)));
                pose1.setSubvector(0,pose1.subVector(0,2)-distance/2*(H.getCol(0).subVector(0,2)));
            }
        }

        trajectory.clear();

        pose[1]=pose[1]-0.03;
        trajectory.push_back(pose1);
        trajectory.push_back(pose);

        ofstream fout(nameFileTrajectory.c_str());

        if(fout.is_open())
        {
            for (size_t i=0; i<trajectory.size();i++)
            fout<<"waypoint "<<i<<" for "<<chosen_hand<<" hand"<<trajectory[i].toString()<<endl;
        }

        return true;
    }

    /***********************************************************************/
    bool showPoses(Vector &pose, Vector &pose2, const int &n_poses, int change_color)
    {
        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut=*imgIn;

        deque<Vector> poses;
        poses.push_back(pose);
        poses.push_back(pose2);

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

        addSuperq(object,imgOut,255);
        Vector hand_in_pose(11,0.0);

        if (findMax(object.subVector(0,2))> findMax(hand.subVector(0,2)))
            hand[1]=findMax(object.subVector(0,2));

        if (n_poses==1)
        {
            hand_in_pose.setSubvector(0,hand);
            hand_in_pose.setSubvector(5,solR);
            addSuperq(hand_in_pose,imgOut,0);
        }
        else if (n_poses==2)
        {
            hand_in_pose.setSubvector(0,hand);
            hand_in_pose.setSubvector(5,solR);
            addSuperq(hand_in_pose,imgOut,0);
            hand_in_pose.setSubvector(0,hand1);
            hand_in_pose.setSubvector(5,solL);
            addSuperq(hand_in_pose,imgOut,0);
        }

        for (int i=0; i<n_poses; i++)
        {
            waypoint=poses[i];

            if (eye=="left")
                igaze->get2DPixel(0,waypoint.subVector(0,2),waypoint2D);
            else
                igaze->get2DPixel(1,waypoint.subVector(0,2),waypoint2D);

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
                cv::line(imgOutMat,target_point,target_pointx,cv::Scalar(255-change_color,0+change_color,0));

            if ((target_pointy.x<0) || (target_pointy.y<0) || (target_pointy.x>=320) || (target_pointy.y>=240))
            {
                yError("Not acceptable pixels!");
            }
            else
                cv::line(imgOutMat,target_point,target_pointy,cv::Scalar(0+change_color,255-change_color,0));

            if ((target_pointz.x<0) || (target_pointz.y<0) || (target_pointz.x>=320) || (target_pointz.y>=240))
            {
                yError("Not acceptable pixels!");
            }
            else
                cv::line(imgOutMat,target_point,target_pointz,cv::Scalar(0,0+change_color,255-change_color));
        }        

        //if (norm(object)!=0.0)
            //igaze->lookAtFixationPoint(object.subVector(5,7));
        portImgOut.write();

        return true;
    }

    /***********************************************************************/
    void addSuperq(const Vector &x, ImageOf<PixelRgb> &imgOut,const int &col)
    {
        PixelRgb color(col,255,0);
        Vector pos, orient;
        double co,so,ce,se;
        Stamp *stamp=NULL;

        Matrix R=euler2dcm(x.subVector(8,10));
        R=R.transposed();

        if ((norm(object)>0.0))
        {
            if (eye=="left")
            {
                if (igaze->getLeftEyePose(pos,orient,stamp))
                {
                    H=axis2dcm(orient);
                    H.setSubcol(pos,0,3);
                    H=SE3inv(H);
                }
            }
            else
            {
                if (igaze->getRightEyePose(pos,orient,stamp))
                {
                    H=axis2dcm(orient);
                    H.setSubcol(pos,0,3);
                    H=SE3inv(H);
                }
            }

            Vector point(3,0.0);
            Vector point2D(2,0.0);
            double step=2*M_PI/50;

            for (double eta=-M_PI; eta<M_PI; eta+=step)
            {
                 for (double omega=-M_PI; omega<M_PI;omega+=step)
                 {
                     co=cos(omega); so=sin(omega);
                     ce=cos(eta); se=sin(eta);

                     point[0]=x[0] * sign(ce)*(pow(abs(ce),x[3])) * sign(co)*(pow(abs(co),x[4])) * R(0,0) +
                                x[1] * sign(ce)*(pow(abs(ce),x[3]))* sign(so)*(pow(abs(so),x[4])) * R(0,1)+
                                    x[2] * sign(se)*(pow(abs(se),x[3])) * R(0,2) + x[5];

                     point[1]=x[0] * sign(ce)*(pow(abs(ce),x[3])) * sign(co)*(pow(abs(co),x[4])) * R(1,0) +
                                x[1] * sign(ce)*(pow(abs(ce),x[3])) * sign(so)*(pow(abs(so),x[4])) * R(1,1)+
                                    x[2] * sign(se)*(pow(abs(se),x[3])) * R(1,2) + x[6];

                     point[2]=x[0] * sign(ce)*(pow(abs(ce),x[3])) * sign(co)*(pow(abs(co),x[4])) * R(2,0) +
                                x[1] * sign(ce)*(pow(abs(ce),x[3])) * sign(so)*(pow(abs(so),x[4])) * R(2,1)+
                                    x[2] * sign(se)*(pow(abs(se),x[3])) * R(2,2) + x[7];

                     point2D=from3Dto2D(point);

                     cv::Point target_point((int)point2D[0],(int)point2D[1]);

                     if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
                     {
                         yError("Not acceptable pixels!");
                     }
                     else
                        imgOut.pixel(target_point.x, target_point.y)=color;

                 }
             }
        }
    }

    /*******************************************************************************/
    Vector from3Dto2D(const Vector &point3D)
    {
        Vector point2D(3,0.0);
        Vector point_aux(4,1.0);
        point_aux.setSubvector(0,point3D);
        point2D=K*H*point_aux;
        return point2D.subVector(0,1)/point2D[2];
    }

    /***********************************************************************/
    bool showTrajectory()
    {
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

        addSuperq(object,imgOut,255);
        Vector hand_in_pose(11,0.0);

        if (chosen_hand=="right")
        {
            hand_in_pose.setSubvector(0,hand);
            hand_in_pose.setSubvector(5,solR);
            addSuperq(hand_in_pose,imgOut,0);
        }
        else
        {
            hand_in_pose.setSubvector(0,hand1);
            hand_in_pose.setSubvector(5,solL);
            addSuperq(hand_in_pose,imgOut,0);
        }

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

        cout<<"Point to be observed: "<<waypoint.toString()<<endl;

        igaze->lookAtFixationPoint(waypoint);
        portImgOut.write();
    }

    /***********************************************************************/
    bool reachPose(const int i)
    {
        bool f;

        if (chosen_hand=="right")
        {
            if (firstRun)
            {
                action->pushAction(home_xR,"karate_hand");

                action->checkActionsDone(f,true);

                firstRun=false;
            }

            Vector point(6,0.0);
            point=trajectory[i];

            Vector orient(4,0.0);
            orient=dcm2axis(euler2dcm(point.subVector(3,5)));

            cout<<"[Go waypoint "<<i<<"]: point to be reached: "<<point.subVector(0,2).toString()<<endl;
            cout<<"[Go waypoint "<<i<<"]: orientation: "<<orient.toString()<<endl;

            // grasp (wait until it's done)
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);

            Vector x_tmp(3,0.0);
            Vector o_tmp(4,0.0);

            icart_arm->getPose(x_tmp, o_tmp);
            double ep, eo;
            ep=norm(x_tmp-point.subVector(0,2));
            eo=norm(dcm2euler(axis2dcm(o_tmp))-point.subVector(3,5));
            yError()<<"Error in position :"<< ep<<" Error in orientation "<<eo;

            cout<<"[Reached pose on object]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;
            showTrajectory();

        }
        else if ((chosen_hand=="left") && (left_or_right=="both"))
        {
            if (firstRun)
            {
                action2->pushAction(home_xL,"karate_hand");

                action2->checkActionsDone(f,true);

                firstRun=false;
            }

            Vector point(6,0.0);
            point=trajectory[i];

            Vector orient(4,0.0);
            orient=dcm2axis(euler2dcm(point.subVector(3,5)));

            cout<<"[Go waypoint "<<i<<"]: point to be reached: "<<point.subVector(0,2).toString()<<endl;
            cout<<"[waypoint "<<i<<"]: orientation: "<<orient.toString()<<endl;

            // grasp (wait until it's done)
            action2->pushAction(point.subVector(0,2), orient);
            action2->checkActionsDone(f,true);

            Vector x_tmp(3,0.0);
            Vector o_tmp(4,0.0);

            icart_arm2->getPose(x_tmp, o_tmp);
            double ep, eo;
            ep=norm(x_tmp-point.subVector(0,2));
            eo=norm(dcm2euler(axis2dcm(o_tmp))-point.subVector(3,5));
            yError()<<"Error in position :"<< ep<<" Error in orientation "<<eo;

            cout<<"[Reached pose on object]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;

            showTrajectory();
        }
        else if ((chosen_hand=="left") && (left_or_right!="both"))
        {
            if (firstRun)
            {
                action->pushAction(home_xL,"karate_hand");

                action->checkActionsDone(f,true);

                firstRun=false;
            }

            Vector point(6,0.0);
            point=trajectory[i];

            Vector orient(4,0.0);
            orient=dcm2axis(euler2dcm(point.subVector(3,5)));

            cout<<"[Go waypoint "<<i<<"]: point to be reached: "<<point.subVector(0,2).toString()<<endl;
            cout<<"[Go waypoint "<<i<<"]: orientation: "<<orient.toString()<<endl;

            // grasp (wait until it's done)
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);

            Vector x_tmp(3,0.0);
            Vector o_tmp(4,0.0);

            icart_arm->getPose(x_tmp, o_tmp);

            cout<<"[Reached pose on object]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;
            showTrajectory();
        }

        return true;
    }

    /***********************************************************************/
    bool graspObject()
    {
        if (chosen_hand=="right")
        {
            bool f;
            action->pushAction("close_hand");
            action->checkActionsDone(f,true);
            firstRun=true;
        }
        else if (chosen_hand=="left" && (left_or_right=="both"))
        {
            bool f;
            action2->pushAction("close_hand");
            action2->checkActionsDone(f,true);
            firstRun=true;
        }
        else if (chosen_hand=="left" && (left_or_right!="both"))
        {
            bool f;
            action->pushAction("close_hand");
            action->checkActionsDone(f,true);
            firstRun=true;
        }

        cout<<"[Object grasped!]"<<endl;

        showTrajectory();
        return true;
    }

    /***********************************************************************/
    void liftObject()
    {
        bool f;
        Vector point(6,0.0);
        point=trajectory[1];

        Vector orient(4,0.0);
        orient=dcm2axis(euler2dcm(point.subVector(3,5)));
        point[2]+=0.2;

        if (chosen_hand=="right")
        {
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);

            point=trajectory[1];
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);
        }
        else if ((chosen_hand=="left") && (left_or_right=="both"))
        {
            action2->pushAction(point.subVector(0,2), orient);
            action2->checkActionsDone(f,true);

            point=trajectory[1];
            action2->pushAction(point.subVector(0,2), orient);
            action2->checkActionsDone(f,true);
        }
        else if ((chosen_hand=="left") && (left_or_right!="both"))
        {
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);

            point=trajectory[1];
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);
        }

        cout<<"[Object lifted!"<<endl;
        showTrajectory();
    }

    /***********************************************************************/
    bool comeBack()
    {
        bool f;
        firstRun=false;

        Vector point(6,0.0);
        Vector x_tmp(3,0.0);
        Vector o_tmp(4,0.0);
        point=trajectory[0];

        Vector orient(4,0.0);
        orient=dcm2axis(euler2dcm(point.subVector(3,5)));

        cout<<"[waypoint 1-back]: point to be reached: "<<point.subVector(0,2).toString()<<endl;
        cout<<"[waypoint 1-back]: orientation: "<<orient.toString()<<endl;

        if (chosen_hand=="right")
        {
            action->pushAction("karate_hand");
            action->checkActionsDone(f,true);
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);

            icart_arm->getPose(x_tmp, o_tmp);

            action->pushAction(home_xR,"karate_hand");

            cout<<"[Reached pose]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;

            action->checkActionsDone(f,true);

            icart_arm->getPose(x_tmp, o_tmp);

            cout<<"[Come back home]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;
        }
        else if ((chosen_hand=="left") && (left_or_right=="both"))
        {
            action2->pushAction("karate_hand");
            action2->checkActionsDone(f,true);
            action2->pushAction(point.subVector(0,2), orient);
            action2->checkActionsDone(f,true);

            icart_arm2->getPose(x_tmp, o_tmp);

            cout<<"[Reached pose]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;

            action2->pushAction(home_xL,"karate_hand");

            action2->checkActionsDone(f,true);

            icart_arm2->getPose(x_tmp, o_tmp);

            cout<<"[Come back home]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;
        }
        else if (chosen_hand=="left" && (left_or_right!="both"))
        {
            action->pushAction("karate_hand");
            action->checkActionsDone(f,true);
            action->pushAction(point.subVector(0,2), orient);
            action->checkActionsDone(f,true);

            icart_arm->getPose(x_tmp, o_tmp);

            cout<<"[Reached pose]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;

            action->pushAction(home_xL,"karate_hand");

            action->checkActionsDone(f,true);

            icart_arm->getPose(x_tmp, o_tmp);

            cout<<"[Come back home]: "<<x_tmp.toString()<<" "<<dcm2euler(axis2dcm(o_tmp)).toString()<<endl;
        }

        showTrajectory();

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
   rf.setDefault("grasp_model_type","springy");
   rf.setDefault("grasp_model_file_right","grasp_model_right.ini");
   rf.setDefault("grasp_model_file_left","grasp_model_left.ini");
   rf.setDefault("hand_sequences_file","hand_sequences.ini");
   rf.setDefault("name","actionPrimitivesMod");
   rf.configure(argc,argv);

   GraspingModule mod;
   return mod.runModule(rf);
}

