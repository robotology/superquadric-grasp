/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <yarp/math/Math.h>

#include "graspVisualization.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


/***********************************************************************/
GraspVisualization::GraspVisualization(int _rate,const string &_eye,IGazeControl *_igaze,bool &_executed, const Matrix _K, const string _left_or_right,
                                       const deque<Property> &_complete_sol, const Vector &_object, const deque<Vector> &_obstacles, Vector &_hand, Vector &_hand1, Property &_vis_par , deque<double> &_cost_vis_r, deque<double> &_cost_vis_l, int &_best_scenario):
                                       RateThread(_rate), eye(_eye), igaze(_igaze),executed(_executed), K(_K), left_or_right(_left_or_right), complete_sol(_complete_sol),
                                       object(_object), obstacles(_obstacles), hand(_hand), hand1(_hand1), vis_par(_vis_par), cost_vis_r(_cost_vis_r), cost_vis_l(_cost_vis_l), best_scenario(_best_scenario)
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

    double length=0.04;

    PixelRgb color(255,255,0);

    Vector x(3,0.0);
    Vector y(3,0.0);
    Vector z(3,0.0);
    Vector x2D(2,0.0);
    Vector y2D(2,0.0);
    Vector z2D(2,0.0);
    Vector dir_x(3,0.0);
    Vector dir_y(3,0.0);
    Vector dir_z(3,0.0);
    Vector waypoint(6,0.0);
    Vector waypoint2D(2,0.0);

    if (norm(object)>0.0)
        addSuperq(object,imgOut,255);

    for (size_t i=0; i<obstacles.size(); i++)
        addSuperq(obstacles[i],imgOut,255);

    if (trajectory_right.size()>0 || trajectory_left.size()>0)
    {
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

        int half_traj_size=trajectory.size()/2;

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
            {
                if (best_scenario>=0)
                {
                    if (i==best_scenario*2 || i==best_scenario*2 +1 || i==(best_scenario)*2 +half_traj_size || i==(best_scenario)*2 + half_traj_size+1)
                        cv::line(imgOutMat,target_point,target_pointx,cv::Scalar(255,0,0), 2, cv::LINE_AA);
                    else
                        cv::line(imgOutMat,target_point,target_pointx,cv::Scalar(255,0,0),1, cv::LINE_AA);
                }
                else
                    cv::line(imgOutMat,target_point,target_pointx,cv::Scalar(255,0,0),1, cv::LINE_AA);

            }

            if ((target_pointy.x<0) || (target_pointy.y<0) || (target_pointy.x>=320) || (target_pointy.y>=240))
            {
                count++;
            }
            else
            {
                if (best_scenario>=0)
                {
                    if (i==best_scenario*2 || i==best_scenario*2 +1 || i==(best_scenario)*2 +half_traj_size || i==(best_scenario)*2 + half_traj_size+1)
                        cv::line(imgOutMat,target_point,target_pointy,cv::Scalar(0,255,0), 2, cv::LINE_AA);
                    else
                         cv::line(imgOutMat,target_point,target_pointy,cv::Scalar(0,255,0),1 , cv::LINE_AA);
                }
                else
                    cv::line(imgOutMat,target_point,target_pointy,cv::Scalar(0,255,0), 1,  cv::LINE_AA);
            }

            if ((target_pointz.x<0) || (target_pointz.y<0) || (target_pointz.x>=320) || (target_pointz.y>=240))
            {
                count++;
            }
            else
            {
                if (best_scenario>=0)
                {
                    if (i==best_scenario*2 || i==best_scenario*2 +1 || i==(best_scenario)*2 +half_traj_size || i==(best_scenario)*2 + half_traj_size+1)
                        cv::line(imgOutMat,target_point,target_pointz,cv::Scalar(0,0,255),2, cv::LINE_AA);
                    else
                        cv::line(imgOutMat,target_point,target_pointz,cv::Scalar(0,0,255),1 , cv::LINE_AA);
                }
                else
                    cv::line(imgOutMat,target_point,target_pointz,cv::Scalar(0,0,255), 1, cv::LINE_AA);
            }
        }

        showHistogram(imgOut);
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

/***********************************************************************/
void GraspVisualization::showHistogram( ImageOf<PixelRgb> &imgOut)
{
    cv::Scalar color_right, color_left;

    cv::Mat imgOutMat=cv::cvarrToMat((IplImage*)imgOut.getIplImage());

    int widthHist;

    if (best_scenario!=-1)
        widthHist=imgOut.height()/(6*cost_vis_r.size());
    else
        widthHist=imgOut.height()/(10*cost_vis_r.size());

    int j=0;
    int minHeight=0;
    int classHeight;
    int maxHeight=(int)(imgOut.width()*0.7);

    double max_cost=0.0;

    cv::Mat imgConfMat=cv::cvarrToMat(imgOut.getIplImage());

    for (size_t k=0; k<cost_vis_r.size(); k++)
    {
        if (cost_vis_r[k]>max_cost)
        {
            max_cost=cost_vis_r[k];
        }

        if (cost_vis_l[k]>max_cost)
        {
            max_cost=cost_vis_l[k];
        }
    }

    for (size_t k=0; k<cost_vis_r.size(); k++)
    {
        stringstream kss;
        stringstream q_r, q_l;
        kss<<k;

        if (cost_vis_r[k]>0.0)
        {
            classHeight=std::max(minHeight+30,(int)(maxHeight*cost_vis_r[k]/max_cost)+30);

            if (best_scenario != -1)
                colorMap(color_right, cost_vis_r[k], max_cost);
            else
            {
                if (cost_vis_r[k]<cost_vis_l[k])
                    color_right=histColorsCode[7];
                else
                    color_right=histColorsCode[0];
            }

            cv::rectangle(imgConfMat, cv::Point(classHeight,j*widthHist), cv::Point(minHeight,(j+1)*widthHist),
            color_right,CV_FILLED);

            q_r<<round(cost_vis_r[k]*100)/100;
            cv::putText(imgOutMat,"R-"+kss.str(),cv::Point(3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,20,0),1, cv::LINE_AA);

            cv::putText(imgOutMat,q_r.str(),cv::Point(classHeight+3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,20,0),1, cv::LINE_AA);
        }
        else
        {
            classHeight=maxHeight;

            cv::rectangle(imgConfMat, cv::Point(classHeight,j*widthHist), cv::Point(minHeight,(j+1)*widthHist),
            cv::Scalar(235,0,0),CV_FILLED);

            q_r<<round(cost_vis_r[k]*100)/100;
            cv::putText(imgOutMat,"R-"+kss.str(),cv::Point(3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,20,0),1, cv::LINE_AA);

            cv::putText(imgOutMat,"Not solved!",cv::Point(classHeight+3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,20,0),1, cv::LINE_AA);
        }

        j++;

        if (cost_vis_l[k]>0.0)
        {
            classHeight=std::max(minHeight+30,(int)(maxHeight*cost_vis_l[k]/max_cost)+30);

            if (best_scenario != -1)
                colorMap(color_left, cost_vis_l[k], max_cost);
            else
            {
                if (cost_vis_l[k]<cost_vis_r[k])
                    color_left=histColorsCode[7];
                else
                    color_left=histColorsCode[0];
            }
            cv::rectangle(imgConfMat,cv::Point(classHeight,j*widthHist),cv::Point(minHeight,(j+1)*widthHist),
            color_left,CV_FILLED);

            q_l<<round(cost_vis_l[k]*100)/100;
            cv::putText(imgOutMat,"L-"+kss.str(),cv::Point(3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,20,0),1, cv::LINE_AA);

            cv::putText(imgOutMat,q_l.str(),cv::Point(classHeight+3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,0,0),1, cv::LINE_AA);
        }
        else
        {
            classHeight=maxHeight;
            cv::rectangle(imgConfMat, cv::Point(classHeight,j*widthHist), cv::Point(minHeight,(j+1)*widthHist),
            cv::Scalar(235,0,0),CV_FILLED);

            cv::putText(imgOutMat,"L-"+kss.str(),cv::Point(3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,20,0),1, cv::LINE_AA);

            cv::putText(imgOutMat,"Not solved!",cv::Point(classHeight+3,(j+1)*widthHist-widthHist/3),
            cv::FONT_HERSHEY_DUPLEX,0.35,cv::Scalar(0,20,0),1, cv::LINE_AA);
        }

        j++;

        if (k==best_scenario)
        {
            if (cost_vis_r[k]< cost_vis_l[k] && cost_vis_r[k]>0.0)
            {
                classHeight=std::max(minHeight+30,(int)(maxHeight*cost_vis_r[k]/max_cost)+30);
                cv::rectangle(imgConfMat, cv::Point(classHeight,(j-2)*widthHist), cv::Point(minHeight,(j-1)*widthHist),
                           cv::Scalar(81,121,233),2, cv::LINE_AA);
            }
            else if (cost_vis_l[k]>0.0)
            {
                classHeight=std::max(minHeight+30,(int)(maxHeight*cost_vis_l[k]/max_cost)+30);
                cv::rectangle(imgConfMat, cv::Point(classHeight,(j-1)*widthHist), cv::Point(minHeight,(j)*widthHist),
                           cv::Scalar(81,121,233),2, cv::LINE_AA);
            }
        }
        else if (best_scenario==-1)
        {
            if (cost_vis_r[0]< cost_vis_l[0] && cost_vis_r[k]>0.0)
            {
                classHeight=std::max(minHeight+30,(int)(maxHeight*cost_vis_r[k]/max_cost)+30);
                cv::rectangle(imgConfMat, cv::Point(classHeight,(0)*widthHist), cv::Point(minHeight,(1)*widthHist),
                           cv::Scalar(81,121,233),2, cv::LINE_AA);
            }
            else if (cost_vis_l[k]>0.0)
            {
                classHeight=std::max(minHeight+30,(int)(maxHeight*cost_vis_l[k]/max_cost)+30);
                cv::rectangle(imgConfMat, cv::Point(classHeight,(1)*widthHist), cv::Point(minHeight,(2)*widthHist),
                           cv::Scalar(81,121,233),2, cv::LINE_AA);
            }
        }
    }
}

/*******************************************************************************/
void GraspVisualization::colorMap(cv::Scalar &color, double &cost_vis, double &max_cost)
{
    if (cost_vis/max_cost==1)
        color=histColorsCode[0];
    else if ((cost_vis/max_cost<1) && (cost_vis/max_cost>0.7))
        color=histColorsCode[1];
    else if ((cost_vis/max_cost<1) && (cost_vis/max_cost>=0.7))
        color=histColorsCode[2];
    else if ((cost_vis/max_cost<0.7) && (cost_vis/max_cost>=0.5))
        color=histColorsCode[3];
    else if ((cost_vis/max_cost<0.7) && (cost_vis/max_cost>=0.5))
        color=histColorsCode[4];
    else if ((cost_vis/max_cost<0.5) && (cost_vis/max_cost>=0.3))
        color=histColorsCode[5];
    else if ((cost_vis/max_cost<0.3) && (cost_vis/max_cost>=0.15))
        color=histColorsCode[6];
    else if ((cost_vis/max_cost<0.15) && (cost_vis/max_cost>=0.0))
        color=histColorsCode[7];
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

    Vector center(3,0.0);
    center[0]= -0.35;
    igaze->lookAtFixationPoint(center);

    stop_fixate=false;

    histColorsCode.push_back(cv::Scalar(235,  0, 0));
    histColorsCode.push_back(cv::Scalar(255,131, 0));
    histColorsCode.push_back(cv::Scalar(255,157, 0));
    histColorsCode.push_back(cv::Scalar(255,212, 0));
    histColorsCode.push_back(cv::Scalar(255,223,76));
    histColorsCode.push_back(cv::Scalar(166,223,76));
    histColorsCode.push_back(cv::Scalar(100,223,76));
    histColorsCode.push_back(cv::Scalar( 69,223,76));

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
        igaze->setTrackingMode(true);

        stop_fixate=false;
    }

    t_vis=Time::now()-t0;
}

/***********************************************************************/
void GraspVisualization::getPoses(const deque<Property> &p)
{
    LockGuard lg(mutex);
    Vector tmp(6,0.0);
    trajectory_right.clear();
    trajectory_left.clear();

    for (size_t i=0; i<p.size(); i++)
    {
        Property poses=p[i];
        Bottle &bottle_right_poses=poses.findGroup("trajectory_right");

        if (!bottle_right_poses.isNull())
        {
            Bottle *p=bottle_right_poses.get(1).asList();
            for (size_t i=0; i<p->size(); i++)
            {
                Bottle *bottle_right_pose=p->get(i).asList();


                for (size_t j=0; j<bottle_right_pose->size(); j++)
                {
                    tmp[j]=bottle_right_pose->get(j).asDouble();
                }

                trajectory_right.push_back(tmp);
            }
        }

        Bottle &bottle_left_poses=poses.findGroup("trajectory_left");

        if (!bottle_left_poses.isNull())
        {
            Bottle *p=bottle_left_poses.get(1).asList();
            for (size_t i=0; i<p->size(); i++)
            {
                Bottle *bottle_left_pose=p->get(i).asList();

                for (size_t j=0; j<bottle_left_pose->size(); j++)
                {
                    tmp[j]=bottle_left_pose->get(j).asDouble();
                }
                trajectory_left.push_back(tmp);
            }
        }

        Bottle &bottle_sol_right=poses.findGroup("solution_right");
        if (!bottle_sol_right.isNull())
        {
            Bottle *p=bottle_sol_right.get(1).asList();

            for (size_t i=0; i<p->size(); i++)
                solR[i]=p->get(i).asDouble();
        }

        Bottle &bottle_sol_left=poses.findGroup("solution_left");
        if (!bottle_sol_left.isNull())
        {
            Bottle *p=bottle_sol_left.get(1).asList();

            for (size_t i=0; i<p->size(); i++)
                solL[i]=p->get(i).asDouble();
        }
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
