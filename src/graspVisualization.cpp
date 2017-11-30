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
GraspVisualization::GraspVisualization(int _rate,const string &_eye,IGazeControl *_igaze, bool &_executed, const Matrix _K, const string _left_or_right,
                                       const Property &_complete_sol, const Vector &_object, Vector &_hand, Vector &_hand1, Property &_vis_par , double &_quality_right, double &_quality_left):
                                       RateThread(_rate), eye(_eye), igaze(_igaze), executed(_executed), K(_K), left_or_right(_left_or_right), complete_sol(_complete_sol),
                                       object(_object), hand(_hand), hand1(_hand1), vis_par(_vis_par), quality_right(_quality_right), quality_left(_quality_left)
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

    addSuperq(object,imgOut,255);

     //if (show_hand)
    //yDebug()<<"Hand in pose R"<<hand_in_poseR.toString();
    //        addSuperq(hand_in_poseR,imgOut,0);

    if (trajectory_right.size()>0 || trajectory_left.size()>0)
    {
        trajectory.clear();

        if (hand_str=="right")
        {
            hand_in_poseR.setSubvector(0,hand);
            hand_in_poseR.setSubvector(5,solR);

            yDebug()<<"Hand in pose R"<<hand_in_poseR.toString();
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

        for (size_t i=0; i<trajectory.size(); i++)
        {
            waypoint=trajectory[i];

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

        if (hand_str=="both")
        {
            stringstream q_r, q_l;
            q_r<<round( quality_right * 100.0 ) / 100.0;
            q_l<<round( quality_left * 100.0 ) / 100.0;

            stringstream right, left;
            right<<"cost right";
            left<<"cost left";

            int thickness=2.0;
            int font=cv::FONT_HERSHEY_SIMPLEX;
            double fontScale=0.5;

            cv::Scalar red(230,0,0);
            cv::Scalar blue(0,240,0);

            cv::Scalar iol_green(22,88,248);
            cv::Scalar iol_red(244,16, 46);

            if ((quality_right<quality_left) && (quality_right!=0.0) && (quality_left!=0.0))
            {
                cv::putText(imgOutMat, q_r.str(), cv::Point(200,85), font, fontScale, iol_green, thickness);
                cv::putText(imgOutMat, q_l.str(), cv::Point(50,85), font, fontScale, iol_red, thickness);
                cv::putText(imgOutMat, right.str(), cv::Point(200,55), font, fontScale, iol_green, thickness);
                cv::putText(imgOutMat, left.str(), cv::Point(50,55), font, fontScale, iol_red, thickness);
            }
            else if ((quality_right>quality_left) && (quality_right!=0.0) && (quality_left!=0.0))
            {
                cv::putText(imgOutMat, q_r.str(), cv::Point(200,85), font, fontScale, iol_red, thickness);
                cv::putText(imgOutMat, q_l.str(), cv::Point(50,85), font, fontScale, iol_green, thickness);
                cv::putText(imgOutMat, right.str(), cv::Point(200,55), font, fontScale, iol_red, thickness);
                cv::putText(imgOutMat, left.str(), cv::Point(50,55), font, fontScale, iol_green, thickness);
            }
        }
            
    }

    portImgOut.write();
}

/***********************************************************************/
void GraspVisualization::addSuperq(const Vector &x, ImageOf<PixelRgb> &imgOut,const int &col)
{
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
                     count++;
                 }
                 else
                    imgOut.pixel(target_point.x, target_point.y)=color;
             }
         }
    }
}

/*******************************************************************************/
Vector GraspVisualization::from3Dto2D(const Vector &point3D)
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

    yDebug()<<"Initial gaze";
    Vector center(3,0.0);
    center[0]= -0.35;
    igaze->lookAtFixationPoint(center);

    //igaze->setTrackingMode(true);

    stop_fixate=false;

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

    igaze->stopControl();
}

/***********************************************************************/
void GraspVisualization::run()
{
    double t0=Time::now();
    getPoses(complete_sol);

    showTrajectory(left_or_right);

    Vector shift_rot(3,0.0);
    shift_rot[1]=0.2;
    

    if ((norm(object)>0.0) && (look_object==true) && (executed==false))
    {
        Vector obj_shift(3,0.0);
        obj_shift=object.subVector(5,7)+shift_rot;
        look_object=!igaze->lookAtFixationPoint(obj_shift);
        //igaze->lookAtFixationPoint(obj_shift);
        igaze->setTrackingMode(true);

        stop_fixate=false;       
    }

    t_vis=Time::now()-t0;
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

    Bottle &pose=poses.findGroup("solution_right");
    if (!pose.isNull())
    {
        Bottle *p=pose.get(1).asList();

        for (size_t i=0; i<p->size(); i++)
            solR[i]=p->get(i).asDouble();
    }

    Bottle &pose1=poses.findGroup("solution_left");
    if (!pose1.isNull())
    {
        Bottle *p=pose1.get(1).asList();

        for (size_t i=0; i<p->size(); i++)
            solL[i]=p->get(i).asDouble();
    }
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


