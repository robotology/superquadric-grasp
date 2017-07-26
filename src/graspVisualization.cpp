/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/math/Math.h>

#include "graspVisualization.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


/***********************************************************************/
GraspVisualization::GraspVisualization(int _rate,const string &_eye,const Matrix _K, const string _left_or_right,
                                       const Property &_complete_sol, const Vector &_object, Vector &_hand, Vector &_hand1, Property &_vis_par):
                                       RateThread(_rate), eye(_eye), K(_K), left_or_right(_left_or_right), complete_sol(_complete_sol),
                                       object(_object), hand(_hand), hand1(_hand1), vis_par(_vis_par)
{

}

/***********************************************************************/
bool GraspVisualization::showTrajectory(const string &hand_str)
{
    int count=0;
    ImageOf<PixelRgb> *imgIn=portImgIn.read(false);
    if (imgIn==NULL)
        return false;

    ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
    imgOut=*imgIn;

    cv::Mat imgInMat=cv::cvarrToMat((IplImage*)imgIn->getIplImage());
    cv::Mat imgOutMat=cv::cvarrToMat((IplImage*)imgOut.getIplImage());
    imgInMat.copyTo(imgOutMat);

    Property *frame_info=portFrameIn.read(false);
    

    PixelRgb color(255,255,0);

    Vector waypoint(6,0.0);
    Vector waypoint2D(2,0.0);

    Vector x(3,0.0);
    Vector y(3,0.0);
    Vector z(3,0.0);
    double length=0.04;
    Vector dir_x(3,0.0);
    Vector dir_y(3,0.0);
    Vector dir_z(3,0.0);
    Vector x2D(2,0.0);
    Vector y2D(2,0.0);
    Vector z2D(2,0.0);  

    //addSuperq(object,imgOut,255);


    if (trajectory_right.size()>0 || trajectory_left.size()>0)
    {        
        Vector x_vis(3);
        Vector o_vis(4);

        if (frame_info!=NULL)
        {
            Bottle &pose_b=frame_info->findGroup("depth");
            Bottle *pose=pose_b.get(1).asList();
            x_vis[0]=pose->get(0).asDouble();
            x_vis[1]=pose->get(1).asDouble();
            x_vis[2]=pose->get(2).asDouble();

            o_vis[0]=pose->get(3).asDouble();
            o_vis[1]=pose->get(4).asDouble();
            o_vis[2]=pose->get(5).asDouble();
            o_vis[3]=pose->get(6).asDouble();

            H=axis2dcm(o_vis);
            H.setSubcol(x_vis,0,3);
            H(3,3)=1;
            H=SE3inv(H);
        }

        trajectory.clear();

        if (hand_str=="right")
        {
            hand_in_poseR.setSubvector(0,hand);
            hand_in_poseR.setSubvector(5,solR);
            if (show_hand)
                addSuperq(hand_in_poseR,imgOut,0);

            if (show_only_pose)
            {
                for (size_t i=0; i<trajectory_right.size(); i++)
                    trajectory.push_back(trajectory_right[trajectory_right.size()-1]);
            }
            else
            {
                for (size_t i=0; i<trajectory_right.size(); i++)
                    trajectory.push_back(trajectory_right[i]);
            }
        }
        else if (hand_str=="left")
        {
            hand_in_poseL.setSubvector(0,hand1);
            hand_in_poseL.setSubvector(5,solL);
            if (show_hand)               
                addSuperq(hand_in_poseL,imgOut,0);

            if (show_only_pose)
            {
                for (size_t i=0; i<trajectory_left.size(); i++)
                    trajectory.push_back(trajectory_left[trajectory_left.size()-1]);
            }
            else
            {
                for (size_t i=0; i<trajectory_left.size(); i++)
                    trajectory.push_back(trajectory_left[i]);
            }
        }
        else
        {
            hand_in_poseR.setSubvector(0,hand);
            hand_in_poseR.setSubvector(5,solR);
            if (show_hand)
                addSuperq(hand_in_poseR,imgOut,0);

            if (show_only_pose)
            {
                for (size_t i=0; i<trajectory_right.size(); i++)
                    trajectory.push_back(trajectory_right[trajectory_right.size()-1]);
            }
            else
            {
                for (size_t i=0; i<trajectory_right.size(); i++)
                    trajectory.push_back(trajectory_right[i]);
            }

            hand_in_poseL.setSubvector(0,hand1);
            hand_in_poseL.setSubvector(5,solL);
            if (show_hand)
                addSuperq(hand_in_poseL,imgOut,0);

            if (show_only_pose)
            {
                for (size_t i=0; i<trajectory_left.size(); i++)
                    trajectory.push_back(trajectory_left[trajectory_left.size()-1]);
            }
            else
            {
                for (size_t i=0; i<trajectory_left.size(); i++)
                    trajectory.push_back(trajectory_left[i]);
            }
        }

        addSuperq(object,imgOut,255);

        for (size_t i=0; i<trajectory.size(); i++)
        {           
            waypoint=trajectory[i];

            Matrix H_tmp=euler2dcm(waypoint.subVector(3,5));

            dir_x=H_tmp.subcol(0,0,3);
            dir_y=H_tmp.subcol(0,1,3);
            dir_z=H_tmp.subcol(0,2,3);

            x[0]=waypoint[0]+length*dir_x[0]; x[1]=waypoint[1]+length*dir_x[1]; x[2]=waypoint[2]+length*dir_x[2];
            y[0]=waypoint[0]+length*dir_y[0]; y[1]=waypoint[1]+length*dir_y[1]; y[2]=waypoint[2]+length*dir_y[2];
            z[0]=waypoint[0]+length*dir_z[0]; z[1]=waypoint[1]+length*dir_z[1]; z[2]=waypoint[2]+length*dir_z[2];
            
            waypoint2D=from3Dto2D(waypoint.subVector(0,2), H);
            x2D=from3Dto2D(x, H);
            y2D=from3Dto2D(y, H);
            z2D=from3Dto2D(z, H);

            cv::Point  target_point((int)waypoint2D[0],(int)waypoint2D[1]);
            cv::Point  target_pointx((int)x2D[0],(int)x2D[1]);
            cv::Point  target_pointy((int)y2D[0],(int)y2D[1]);
            cv::Point  target_pointz((int)z2D[0],(int)z2D[1]);

            if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
            {
                count++;
            }
            else
                imgOut.pixel(target_point.x, target_point.y)= color;

            if ((target_pointx.x<0) || (target_pointx.y<0) || (target_pointx.x>=320) || (target_pointx.y>=240))
            {
                count++;
            }
            else
                cv::line(imgOutMat,target_point,target_pointx,cv::Scalar(255,0,0));

            if ((target_pointy.x<0) || (target_pointy.y<0) || (target_pointy.x>=320) || (target_pointy.y>=240))
            {
                count++;
            }
            else
                cv::line(imgOutMat,target_point,target_pointy,cv::Scalar(0,255,0));

            if ((target_pointz.x<0) || (target_pointz.y<0) || (target_pointz.x>=320) || (target_pointz.y>=240))
            {
                count++;
            }
            else
                cv::line(imgOutMat,target_point,target_pointz,cv::Scalar(0,0,255));
        }
    }

    portImgOut.write();
}

/***********************************************************************/
void GraspVisualization::addSuperq(const Vector &x, ImageOf<PixelRgb> &imgOut,const int &col)
{
    Property *frame_info=portFrameIn.read(false);

    int count=0;

    PixelRgb color(col,0,0);

    if (col==0)
    {
        color.r=0;
        color.b=255;
    }

    Vector pos, orient;
    double co,so,ce,se;
    Stamp *stamp=NULL;

    Matrix R=euler2dcm(x.subVector(8,10));
    R=R.transposed();

    if ((norm(x)>0.0))
    {
        Vector x_vis(3);
        Vector o_vis(4);

        if (frame_info!=NULL)
        {
            Bottle &pose_b=frame_info->findGroup("depth");
            Bottle *pose=pose_b.get(1).asList();
            x_vis[0]=pose->get(0).asDouble();
            x_vis[1]=pose->get(1).asDouble();
            x_vis[2]=pose->get(2).asDouble();

            o_vis[0]=pose->get(3).asDouble();
            o_vis[1]=pose->get(4).asDouble();
            o_vis[2]=pose->get(5).asDouble();
            o_vis[3]=pose->get(6).asDouble();

            H=axis2dcm(o_vis);
            H.setSubcol(x_vis,0,3);
            H(3,3)=1;
            H=SE3inv(H);
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

                 point2D=from3Dto2D(point, H);

                 cv::Point target_point((int)point2D[0],(int)point2D[1]);

                 if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
                 {
                     count++;
                 }
                 else
                    imgOut.pixel(target_point.x, target_point.y)=color;
             }
         }
    }
}

/*******************************************************************************/
Vector GraspVisualization::from3Dto2D(const Vector &point3D, Matrix &H)
{
    Vector point2D(3,0.0);
    Vector point_aux(4,1.0);
    point_aux.setSubvector(0,point3D);
    point2D=K*H*point_aux;
    return point2D.subVector(0,1)/point2D[2];
}

/***********************************************************************/
bool GraspVisualization::threadInit()
{
    yInfo()<<"[GraspVisualization]: Thread initing ... ";

    portImgIn.open("/superquadric-grasp/img:i");
    portImgOut.open("/superquadric-grasp/img:o");
    portFrameIn.open("/superquadric-grasp/frame:i");
    portGaze.open("/superquadric-grasp/motor:o");

    R.resize(4,4);
    H.resize(4,4);
    point2D.resize(2,0.0);
    point.resize(3,0.0);
    point1.resize(3,0.0);
    superq.resize(11,0.0);

    poseR.resize(6,0.0);
    poseL.resize(6,0.0);
    solR.resize(6,0.0);
    solL.resize(6,0.0);

    hand_in_poseR.resize(11,0.0);
    hand_in_poseL.resize(11,0.0);

    trajectory.clear();

    setPar(vis_par, true);

    H.resize(4,4);
    H.eye();

    return true;
}

/***********************************************************************/
void GraspVisualization::threadRelease()
{
    yInfo()<<"[GraspVisualization]: Thread releasing ... ";

    if (!portImgIn.isClosed())
        portImgIn.close();

    if (!portImgOut.isClosed())
        portImgOut.close();

    if (!portFrameIn.isClosed())
        portFrameIn.close();

    if (!portGaze.isClosed())
        portGaze.close();
}

/***********************************************************************/
void GraspVisualization::run()
{
    double t0=Time::now();
    getPoses(complete_sol);

    showTrajectory(left_or_right);

    if ((norm(object)>0.0) && (look_object==true))
    {
        Vector obj_to_see=object.subVector(5,7);
        look(obj_to_see);
    }

    t_vis=Time::now()-t0;
}

/***********************************************************************/
void GraspVisualization::look(Vector &point3d)
{
    Bottle b;
    b.addList().read(point3d);

    Property &cmd=portGaze.prepare();
    cmd.clear();

    cmd.put("control-frame","depth");
    cmd.put("target-type","cartesian");
    cmd.put("target-location",b.get(0));

    portGaze.writeStrict();

}

/***********************************************************************/
void GraspVisualization::getPoses(const yarp::os::Property &poses)
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
    //else
    //    yError()<<"[GraspVisualization]: No trajectory right found!";

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
    // else
    //   yError()<<"[GraspVisualization]: No trajectory left found!";

    Bottle &pose=poses.findGroup("solution_right");
    if (!pose.isNull())
    {
        Bottle *p=pose.get(1).asList();

        for (size_t i=0; i<p->size(); i++)
            solR[i]=p->get(i).asDouble();
    }
    //else
    //     yError()<<"[GraspVisualization]: No solution right found!";

    Bottle &pose1=poses.findGroup("solution_left");
    if (!pose1.isNull())
    {
        Bottle *p=pose1.get(1).asList();

        for (size_t i=0; i<p->size(); i++)
            solL[i]=p->get(i).asDouble();
    }
    //else
    //     yError()<<"[GraspVisualization]: No solution left found!";
}

/***********************************************************************/
double GraspVisualization::getTime()
{
    LockGuard lg(mutex);
    return t_vis;
}

/***********************************************************************/
void GraspVisualization::setPar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);

    string show_h=newOptions.find("show_hand").asString();

    if (newOptions.find("show_hand").isNull() && (first_time==true))
    {
        show_hand=false;
    }
    else if (!newOptions.find("show_hand").isNull())
    {
        if ((show_h=="on") || (show_h=="off"))
        {
            show_hand=(show_h=="on");
        }
        else
        {
            show_hand=false;
        }
    }

    string l_o=newOptions.find("look_object").asString();

    if (newOptions.find("look_object").isNull() && (first_time==true))
    {
        look_object=false;
    }
    else if (!newOptions.find("look_object").isNull())
    {
        if ((l_o=="on") || (l_o=="off"))
        {
            look_object=(l_o=="on");
        }
        else
        {
            look_object=false;
        }
    }

    string show_pose=newOptions.find("show_only_pose").asString();

    if (newOptions.find("show_only_pose").isNull() && (first_time==true))
    {
        show_only_pose=false;
    }
    else if (!newOptions.find("show_only_pose").isNull())
    {
        if ((show_pose=="on") || (show_pose=="off"))
        {
            show_only_pose=(show_pose=="on");
        }
        else
        {
            show_only_pose=false;
        }
    }
}

/***********************************************************************/
Property GraspVisualization::getPar()
{
    LockGuard lg(mutex);

    Property advOptions;
    if (show_hand)
        advOptions.put("show_hand","on");
    else
        advOptions.put("show_hand","off");
    if (look_object)
        advOptions.put("look_object","on");
    else
        advOptions.put("look_object","off");
    if (show_only_pose)
        advOptions.put("show_only_pose","on");
    else
        advOptions.put("show_only_pose","off");

    return advOptions;
}


