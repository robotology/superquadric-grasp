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

#include "superqVisualization.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


/***********************************************************************/
GraspVisualization::GraspVisualization(int _rate,const string &_eye,IGazeControl *_igaze, const Matrix _K, const string _left_or_right):
                                       RateThread(_rate), eye(_eye), igaze(_igaze), K(_K), left_or_right(_left_or_right)
{

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

   //igaze->lookAtFixationPoint(waypoint);
   portImgOut.write();
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
                     yError("Not acceptable pixels IN SUPERQ!");
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
bool GraspVisualization::threadInit()
{
    yInfo()<<"[GraspVisualization]: Thread initing ... ";

    portImgOut.open("/superquadric-grasp/img:o");

    R.resize(4,4);
    H.resize(4,4);
    point2D.resize(2,0.0);
    point.resize(3,0.0);
    point1.resize(3,0.0);
    superq.resize(11,0.0);

    poseR.resize(6,0.0);
    poseL.resize(6,0.0);

    return true;
}

/***********************************************************************/
void GraspVisualization::run()
{
    double t0=Time::now();
    if (left_or_right=="both")
        showPoses(poseR, poseL, 2,0);
    else if (left_or_right=="right")
        showPoses(poseR, poseR, 1,0);
    else
        showPoses(poseL, poseL, 1,0);

    //showTrajectory();

    t_vis=Time::now()-t0;
}

/***********************************************************************/
void getPoses(yarp::os::Property &poses)
{
    LockGuard lg(mutex);

    Bottle &pose=poses.findGroup("pose_right");
    if (!poses.isNull())
    {
        for (size_t i=0; i<pose.size(); i++)
            poseR[i]=pose.get(i).asDouble();
    }
    else
        yError()<<"[GraspVisualization]: No pose right received!";

    Bottle &pose2=poses.findGroup("pose_right");
    if (!poses2.isNull())
    {
        for (size_t i=0; i<pose2.size(); i++)
            poseL[i]=pose2.get(i).asDouble();
    }
    else
        yError()<<"[GraspVisualization]: No pose right received!";
}

