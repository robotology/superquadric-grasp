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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include "superquadric.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
void grasping_NLP::init(const Vector &objectext, Vector &handext, const deque<Vector> &obstacleext, int &n_handpoints, const string &str_hand)
{
    hand=handext;
    object=objectext;

    for (size_t i=0; i<obstacleext.size(); i++)
        obstacles.push_back(obstacleext[i]);

    if (obstacleext.size()!= num_superq)
        num_superq = obstacleext.size();

    H_o2w.resize(4,4);
    H_h2w.resize(4,4);
    H_x.resize(4,4);
    euler.resize(3,0.0);

    euler[0]=object[8];
    euler[1]=object[9];
    euler[2]=object[10];
    H_o2w=euler2dcm(euler);
    euler[0]=object[5];
    euler[1]=object[6];
    euler[2]=object[7];
    H_o2w.setSubcol(euler,0,3);

    H_o2w=H_o2w.transposed();

    euler[0]=hand[8];
    euler[1]=hand[9];
    euler[2]=hand[10];
    H_h2w=euler2dcm(euler);
    euler[0]=hand[5];
    euler[1]=hand[6];
    euler[2]=hand[7];
    H_h2w.setSubcol(euler,0,3);

    for(int i=0; i<(int)sqrt(n_handpoints); i++)
    {
        for (double theta=0; theta<=2*M_PI; theta+=M_PI/((int)sqrt(n_handpoints)))
        {
            Vector point=computePointsHand(hand,i, (int)sqrt(n_handpoints), str_hand, theta);
            Vector point_tr(4,0.0);

            euler[0]=hand[8];
            euler[1]=hand[9];
            euler[2]=hand[10];
            H_h2w=euler2dcm(euler);
            euler[0]=hand[5];
            euler[1]=hand[6];
            euler[2]=hand[7];
            H_h2w.setSubcol(euler,0,3);

            if (str_hand=="right")
            {
                if (point[0] + point[2] < 0)
                {
                    Vector point_tmp(4,1.0);
                    point_tmp.setSubvector(0,point);
                    point_tr=H_h2w*point_tmp;
                    point=point_tr.subVector(0,2);
                    points_on.push_back(point);
                }
            }
            else
            {
                if (point[0] - point[2] < 0)
                {
                    Vector point_tmp(4,1.0);
                    point_tmp.setSubvector(0,point);
                    point_tr=H_h2w*point_tmp;
                    point=point_tr.subVector(0,2);
                    points_on.push_back(point);
                }
            }

        }
    }

    yDebug()<<"Points on hand size "<<points_on.size();

    aux_objvalue=0.0;
}

/****************************************************************/
Vector grasping_NLP::computePointsHand(Vector &hand, int j, int l, const string &str_hand, double &theta)
{
    Vector point(3,0.0);
    double omega;
    double ce,se,co,so;

    if (findMax(object.subVector(0,2))> findMax(hand.subVector(0,2)))
        hand[1]=findMax(object.subVector(0,2));

    omega=j*M_PI/(l);

    ce=cos(theta);
    co=cos(omega);
    so=sin(omega);

    point[0]=hand[0] * sign(ce)*(pow(abs(ce),hand[3])) * sign(co)*(pow(abs(co),hand[4]));
    point[1]=hand[1] * sign(se)*(pow(abs(se),hand[3]));
    point[2]=hand[2] * sign(ce)*(pow(abs(ce),hand[3])) * sign(so)*(pow(abs(so),hand[4]));

    return point;
}

/****************************************************************/
bool grasping_NLP::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                  Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style)
{
    n=6;
    if (num_superq==0)
        m=7;
    else
        m=7+(num_superq);

    nnz_jac_g=n*m;
    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    x_v.resize(n,0.0);
    bounds.resize(n,2);
    bounds_constr(m,2);

    return true;
}

/****************************************************************/
bool grasping_NLP::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                     Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    for (Ipopt::Index i=0; i<n; i++)
    {
       x_l[i]=bounds(i,0);
       x_u[i]=bounds(i,1);
    }

    for (Ipopt::Index i=0; i<m; i++)
    {
       g_l[i]=bounds_constr(i,0);
       g_u[i]=bounds_constr(i,1);
    }

    return true;
}

/****************************************************************/
 bool grasping_NLP::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
 {
     for(Ipopt::Index i=0;i<n;i++)
     {
         x[i]=hand[i+5];
     }

     return true;
 }

 /****************************************************************/
 bool grasping_NLP::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
 {
     F(x,points_on,new_x);
     obj_value=aux_objvalue;

     return true;
 }

 /****************************************************************/
 double grasping_NLP::F(const Ipopt::Number *x, deque<Vector> &points_on, bool new_x)
 {
     double value=0.0;

     for(size_t i=0;i<points_on.size();i++)
         value+= pow( pow(f(object,x,points_on[i]),object[3])-1,2 );

     euler[0]=x[3];
     euler[1]=x[4];
     euler[2]=x[5];
     H_x=euler2dcm(euler);
     euler[0]=x[0];
     euler[1]=x[1];
     euler[2]=x[2];
     H_x.setSubcol(euler,0,3);

     Matrix H(4,4);
     H=H_x*H_h2w;

     Vector xv(3,0.0);
     xv[0]=H(0,3);
     xv[1]=H(1,3);
     xv[2]=H(2,3);
     value*=object[0]*object[1]*object[2]/points_on.size();

     aux_objvalue=value;
 }

 /****************************************************************/
 double grasping_NLP::f(Vector &obj, const Ipopt::Number *x, Vector &point)
 {
     Matrix H(4,4);

     Vector point_tr(4,0.0);
     Vector point_tmp(4,1.0);
     point_tmp.setSubvector(0,point);

     euler[0]=x[3];
     euler[1]=x[4];
     euler[2]=x[5];
     H_x=euler2dcm(euler);
     euler[0]=x[0];
     euler[1]=x[1];
     euler[2]=x[2];
     H_x.setSubcol(euler,0,3);

     point_tr=H_x*point_tmp;

     double num1=H_o2w(0,0)*point_tr[0]+H_o2w(0,1)*point_tr[1]+H_o2w(0,2)*point_tr[2]-obj[5]*H_o2w(0,0)-obj[6]*H_o2w(0,1)-obj[7]*H_o2w(0,2);
     double num2=H_o2w(1,0)*point_tr[0]+H_o2w(1,1)*point_tr[1]+H_o2w(1,2)*point_tr[2]-obj[5]*H_o2w(1,0)-obj[6]*H_o2w(1,1)-obj[7]*H_o2w(1,2);
     double num3=H_o2w(2,0)*point_tr[0]+H_o2w(2,1)*point_tr[1]+H_o2w(2,2)*point_tr[2]-obj[5]*H_o2w(2,0)-obj[6]*H_o2w(2,1)-obj[7]*H_o2w(2,2);

     double tmp=pow(abs(num1/obj[0]),2.0/obj[4]) + pow(abs(num2/obj[1]),2.0/obj[4]);

     return pow( abs(tmp),obj[4]/obj[3]) + pow( abs(num3/obj[2]),(2.0/obj[3]));
 }

 /****************************************************************/
 double grasping_NLP::F_v(Vector &x, deque<Vector> &points_on)
 {
     double value=0.0;

     for(size_t i=0;i<points_on.size();i++)
        value+= pow( pow(f_v(object,x,points_on[i]),object[3])-1,2 );

     value*=object[0]*object[1]*object[2]/points_on.size();

     return value;
 }

  /****************************************************************/
 double grasping_NLP::f_v(Vector &obj, Vector &x, Vector &point)
 {
     Vector point_tr(4,0.0);
     Vector point_tmp(4,1.0);
     point_tmp.setSubvector(0,point);
     euler[0]=x[3];
     euler[1]=x[4];
     euler[2]=x[5];
     H_x=euler2dcm(euler);
     euler[0]=x[0];
     euler[1]=x[1];
     euler[2]=x[2];
     H_x.setSubcol(euler,0,3);

     point_tr=H_x*point_tmp;

     double num1=H_o2w(0,0)*point_tr[0]+H_o2w(0,1)*point_tr[1]+H_o2w(0,2)*point_tr[2]-obj[5]*H_o2w(0,0)-obj[6]*H_o2w(0,1)-obj[7]*H_o2w(0,2);
     double num2=H_o2w(1,0)*point_tr[0]+H_o2w(1,1)*point_tr[1]+H_o2w(1,2)*point_tr[2]-obj[5]*H_o2w(1,0)-obj[6]*H_o2w(1,1)-obj[7]*H_o2w(1,2);
     double num3=H_o2w(2,0)*point_tr[0]+H_o2w(2,1)*point_tr[1]+H_o2w(2,2)*point_tr[2]-obj[5]*H_o2w(2,0)-obj[6]*H_o2w(2,1)-obj[7]*H_o2w(2,2);

     double tmp=pow(abs(num1/obj[0]),2.0/obj[4]) + pow(abs(num2/obj[1]),2.0/obj[4]);

     return pow( abs(tmp),obj[4]/obj[3]) + pow( abs(num3/obj[2]),(2.0/obj[3]));
 }

 /********************************************************************/
 double grasping_NLP::f_v2(Vector &obj, Vector &x, Vector &point_tr)
 {
     double num1=H_o2w(0,0)*point_tr[0]+H_o2w(0,1)*point_tr[1]+H_o2w(0,2)*point_tr[2]-obj[5]*H_o2w(0,0)-obj[6]*H_o2w(0,1)-obj[7]*H_o2w(0,2);
     double num2=H_o2w(1,0)*point_tr[0]+H_o2w(1,1)*point_tr[1]+H_o2w(1,2)*point_tr[2]-obj[5]*H_o2w(1,0)-obj[6]*H_o2w(1,1)-obj[7]*H_o2w(1,2);
     double num3=H_o2w(2,0)*point_tr[0]+H_o2w(2,1)*point_tr[1]+H_o2w(2,2)*point_tr[2]-obj[5]*H_o2w(2,0)-obj[6]*H_o2w(2,1)-obj[7]*H_o2w(2,2);

     double tmp=pow(abs(num1/obj[0]),2.0/obj[4]) + pow(abs(num2/obj[1]),2.0/obj[4]);

     return pow( abs(tmp),obj[4]/obj[3]) + pow( abs(num3/obj[2]),(2.0/obj[3]));
 }

 /****************************************************************/
 bool grasping_NLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number *grad_f)
 {
     Vector x_tmp(6,0.0);
     double grad_p, grad_n;
     double eps=1e-6;

     for(Ipopt::Index i=0;i<n;i++)
        x_tmp[i]=x[i];

     for(Ipopt::Index j=0;j<n;j++)
     {
         x_tmp[j]=x_tmp[j]+eps;

         grad_p=F_v(x_tmp,points_on);

         x_tmp[j]=x_tmp[j]-eps;

         grad_n=F_v(x_tmp,points_on);

         grad_f[j]=(grad_p-grad_n)/eps;
     }

     return true;
 }

 /****************************************************************/
 bool grasping_NLP::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
             Ipopt::Index m, Ipopt::Number *g)
 {
     euler[0]=x[3];
     euler[1]=x[4];
     euler[2]=x[5];
     H_x=euler2dcm(euler);
     euler[0]=x[0];
     euler[1]=x[1];
     euler[2]=x[2];
     H_x.setSubcol(euler,0,3);

     Matrix H(4,4);
     H=H_x*H_h2w;

     g[0]=H(2,2);
     g[1]=H(0,0);
     g[2]=H(2,1);
     g[3]=H(1,0);
     g[4]=H(0,2);

     yDebug()<<"g "<<g[0] <<g[1] << g[2]<<g[3]<<g[4]<<g[5];
     

     Vector x_min;
     double minz=10.0;

     for (size_t i=0; i<points_on.size(); i++)
     {
         Vector pnt(4,1.0);
         pnt.setSubvector(0,points_on[i]);
         Vector point=H_x*pnt;

         if (point[2]<minz)
         {
             minz=point[2];
             x_min=point;
         }
     }

     g[5]=plane(0,0)*x_min[0]+plane(1,0)*x_min[1]+plane(2,0)*x_min[2]+plane(3,0);

     Vector robotPose(3,0.0);
     Vector x_tmp(6,0.0);
     x_tmp[0]=x[0];
     x_tmp[1]=x[1];
     x_tmp[2]=x[2];
     x_tmp[3]=x[3];
     x_tmp[4]=x[4];
     x_tmp[5]=x[5];

     if (l_o_r=="right")
        robotPose=x_tmp.subVector(0,2)-hand[0]*(H.getCol(2).subVector(0,2));
     else
         robotPose=x_tmp.subVector(0,2)+hand[0]*(H.getCol(2).subVector(0,2));

     g[6]=object[0]*object[1]*object[2]*(pow(f_v2(object,x_tmp, robotPose), object[3]) -1);

     if (num_superq>0)
     {
         for (size_t j=0; j<num_superq; j++)
         {
             g[7+j]=0;

             for(size_t i=0;i<points_on.size();i++)
                g[7+j]+= pow(f(obstacles[j],x,points_on[i]),obstacles[j][3])-1;

             g[7+j]*=obstacles[j][0]*obstacles[j][1]*obstacles[j][2];
         }
     }

     //yDebug()<<"g "<<g[0] <<g[1] << g[2]<<g[3]<<g[4]<<g[5];

     return true;
 }

 /****************************************************************/
 double grasping_NLP::G_v(Vector &x, int i, Ipopt::Index m)
 {
     Vector g(m,0.0);

     Matrix H_x,H;
     H_x.resize(4,4);
     H.resize(4,4);
     euler[0]=x[3];
     euler[1]=x[4];
     euler[2]=x[5];
     H_x=euler2dcm(euler);
     euler[0]=x[0];
     euler[1]=x[1];
     euler[2]=x[2];
     H_x.setSubcol(euler,0,3);

     H=H_x*H_h2w;

     g[0]=H(2,2);
     g[1]=H(0,0);
     g[2]=H(1,2);
     g[3]=H(1,0);
     g[4]=H(0,2);

     Vector x_min(3,0.0);
     double minz=10.0;

     for (size_t i1=0; i1<points_on.size(); i1++)
     {
         Vector pnt(4,1.0);
         pnt.setSubvector(0,points_on[i1]);
         Vector point=H_x*pnt;

         if (point[2]<minz)
         {
             minz=point[2];
             x_min=point;
         }
     }

     g[5]=plane(0,0)*x_min[0]+plane(1,0)*x_min[1]+plane(2,0)*x_min[2]+plane(3,0);

     Vector robotPose(3,0.0);
     Vector x_tmp(3,0.0);
     x_tmp[0]=x[0];
     x_tmp[1]=x[1];
     x_tmp[2]=x[2];

     if (l_o_r=="right")
        robotPose=x_tmp-hand[0]*(H.getCol(2).subVector(0,2));
     else
         robotPose=x_tmp+hand[0]*(H.getCol(2).subVector(0,2));

     g[6]=object[0]*object[1]*object[2]*(pow(f_v2(object,x_tmp, robotPose), object[3]) -1);

     if (num_superq>0)
     {
         for (size_t j=0; j<num_superq; j++)
         {
             g[7+j]=0;

             for(size_t i=0;i<points_on.size();i++)
                g[7+j]+= pow(f_v(obstacles[j],x,points_on[i]),obstacles[j][3])-1;

             g[7+j]*=obstacles[j][0]*obstacles[j][1]*obstacles[j][2];
         }
     }
     return g[i];
 }

 /****************************************************************/
 bool grasping_NLP::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                 Ipopt::Index *jCol, Ipopt::Number *values)
 {
     Vector x_tmp(6,0.0);
     double grad_p, grad_n;
     double eps=1e-6;

     if(values!=NULL)
     {
         for(Ipopt::Index i=0;i<n;i++)
            x_tmp[i]=x[i];

         int count=0;
         for(Ipopt::Index i=0;i<m; i++)
         {
             for(Ipopt::Index j=0;j<n;j++)
             {
                 x_tmp[j]=x_tmp[j]+eps;

                 grad_p=G_v(x_tmp,i,m);
                 x_tmp[j]=x_tmp[j]-eps;

                 grad_n=G_v(x_tmp,i,m);

                 values[count]=(grad_p-grad_n)/(eps);
                 count++;
             }
         }
     }
     else
    {
        if (num_superq==0)
        {
            for (size_t j=0; j<m; j++)
            {
                for (size_t i=0; i<n ; i++)
                {
                    jCol[j*(n) + i]= i;
                    iRow[j*(n)+ i] = j;
                }
            }
        }
        else
        {
            for (size_t j=0; j<m; j++)
            {
                for (size_t i=0; i<n ; i++)
                {
                    jCol[j*(n) + i]= i;
                    iRow[j*(n)+ i] = j;
                }
            }
        }
     }

 return true;

 }

/****************************************************************/
void grasping_NLP::configure(ResourceFinder *rf, const string &left_or_right, const Vector &disp, const Vector &pl)
{
    Matrix x0_tmp(6,1);
    x0.resize(6,0.0);

    readMatrix("x0"+left_or_right,x0_tmp,1,rf);

    for(size_t i=0; i< 6; i++)
        x0[i]=x0_tmp(i,0);

    bounds.resize(6,2);    
    readMatrix("bounds_"+left_or_right,bounds, 6, rf);

    max_superq= rf->check("max_superq", Value(4)).asInt();

    bounds_constr.resize(7 + max_superq - 1,2);
    readMatrix("bounds_constr_"+left_or_right,bounds_constr,7 + max_superq - 1 , rf);

    yDebug()<<">>>>>>>>>>>>>>>>>> BOUNDS CONSTR "<<bounds_constr.toString();

    plane.resize(4,1);

    l_o_r=left_or_right;
    displacement=disp;
    plane.setCol(0,pl); 
}

/****************************************************************/
bool grasping_NLP::readMatrix(const string &tag, Matrix &matrix, const int &dimension, ResourceFinder *rf)
{
   string tag_x=tag+"_x";
   string tag_y=tag+"_y";
   bool check_x;

   if(tag=="x0" || tag=="plane")
   {
       if (Bottle *b=rf->find(tag.c_str()).asList())
       {
           Vector col;
           if (b->size()>=dimension)
           {
               for(size_t i=0; i<b->size();i++)
                   col.push_back(b->get(i).asDouble());

               matrix.setCol(0, col);
           }
           return true;
       }
   }
   else
   {
       if(tag=="bounds_right" || tag=="bounds_constr_right" || tag=="bounds_left" || tag=="bounds_constr_left")
       {
           tag_x=tag+"_l";
           tag_y=tag+"_u";
       }

       if (Bottle *b=rf->find(tag_x.c_str()).asList())
       {
           Vector col;
           if (b->size()>=dimension)
           {
               for(size_t i=0; i<b->size();i++)
                   col.push_back(b->get(i).asDouble());

               matrix.setCol(0, col);
           }
           check_x=true;

       }
       if (Bottle *b=rf->find(tag_y.c_str()).asList())
       {
           Vector col;
           if (b->size()>=dimension)
           {
               for(size_t i=0; i<b->size();i++)
                   col.push_back(b->get(i).asDouble());
               matrix.setCol(1, col);
           }
           if(check_x==true)
               return true;
       }
   }
return false;
}

/****************************************************************/
void grasping_NLP::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                      const Ipopt::Number *x, const Ipopt::Number *z_L,
                      const Ipopt::Number *z_U, Ipopt::Index m,
                      const Ipopt::Number *g, const Ipopt::Number *lambda,
                      Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                      Ipopt::IpoptCalculatedQuantities *ip_cq)
{
   solution.resize(n);

   euler[0]=x[3];
   euler[1]=x[4];
   euler[2]=x[5];
   H_x=euler2dcm(euler);
   euler[0]=x[0];
   euler[1]=x[1];
   euler[2]=x[2];
   H_x.setSubcol(euler,0,3);

   //Matrix H;
   //H.resize(4,4);
   //H=H_x*H_h2w;  // H_h2w is the identity

   cout<<endl;
   //if (notAlignedPose(H_x))
   //    alignPose(H_x);

   solution.setSubvector(3,dcm2euler(H_x));

   for (Ipopt::Index i=0; i<3; i++)
       solution[i]=H_x(i,3);

    robot_pose.resize(6,0.0);
    robot_pose.setSubvector(3,dcm2euler(H_x));

    if (l_o_r=="right")
    {
        robot_pose.setSubvector(0,solution.subVector(0,2)-(hand[0]+displacement[2])*(H_x.getCol(2).subVector(0,2)));
        robot_pose.setSubvector(0,robot_pose.subVector(0,2)-displacement[0]*(H_x.getCol(0).subVector(0,2)));
        robot_pose.setSubvector(0,robot_pose.subVector(0,2)-displacement[1]*(H_x.getCol(1).subVector(0,2)));
    }
    else
    {
        robot_pose.setSubvector(0,solution.subVector(0,2)+(hand[0]+displacement[2])*(H_x.getCol(2).subVector(0,2)));
        robot_pose.setSubvector(0,robot_pose.subVector(0,2)-displacement[0]*(H_x.getCol(0).subVector(0,2)));
        robot_pose.setSubvector(0,robot_pose.subVector(0,2)-displacement[1]*(H_x.getCol(1).subVector(0,2)));
    }

    final_F_value=0.0;

    for(size_t i=0;i<points_on.size();i++)
    {
        final_F_value+= pow( pow(f_v(object,solution,points_on[i]),object[3])-1,2 );
    }

    final_F_value/=points_on.size();

    Vector x_aux(11,0.0);

    for (size_t i=0; i<11; i++)
        x_aux[i]=x[i];

    final_obstacles_value=computeFinalObstacleValues(x_aux);

}

/****************************************************************/
Vector grasping_NLP::get_result() const
{
   return solution;
}

/****************************************************************/
Vector grasping_NLP::get_hand() const
{
   return hand;
}

/****************************************************************/
double grasping_NLP::get_final_F() const
{
   return final_F_value;
}

/****************************************************************/
deque<double> grasping_NLP::get_final_constr_values() const
{
   return final_obstacles_value;
}

/****************************************************************/
deque<double> grasping_NLP::computeFinalObstacleValues(Vector &x_aux) 
{
    deque<double> values;
    double constr_value=0.0;

    for (size_t i=0; i<obstacles.size(); i++)
    {
        constr_value=0.0;

        Vector obstacle=obstacles[i];

        for (size_t j=0; j<points_on.size(); j++)
        {           
            constr_value+= pow( pow(f_v(obstacle,x_aux,points_on[i]),obstacle[3])-1,2 );
        }
        constr_value/=points_on.size();

        values.push_back(constr_value);
    }

    return values;

}

/****************************************************************/
bool grasping_NLP::notAlignedPose(Matrix &final_H)
{
    if ((final_H(2,1)< 0.0 && final_H(2,1) > -0.3) ||  (final_H(2,1) < -0.6 && final_H(2,1) > -1.0))
    {
        yInfo()<<"Not aligned pose ";
        cout<<endl;

        if ((final_H(2,1)< 0.0 && final_H(2,1) > -0.3))
            top_grasp=true;
        else
            top_grasp=false;

        return true;
    }
    else
         return false;
}

/****************************************************************/
void grasping_NLP::alignPose(Matrix &final_H)
{
    Matrix rot_x(3,3);
    rot_x.eye();

    Vector axis(4,0.0);
    double theta;

    cout<<endl;
    yDebug()<<"H_final before ";
    yDebug()<<final_H.toString();

    if (top_grasp)
    {
        if (l_o_r=="right")
            theta=M_PI/2 - acos(-final_H(2,1));
        else
            theta=-(M_PI/2 - acos(-final_H(2,1)));

        yDebug()<<"theta "<<theta;
        rot_x(1,1)=rot_x(2,2)=cos(theta);
        rot_x(2,1)=sin(theta);
        rot_x(1,2) = -rot_x(2,1);

    }
    else
    {
        Vector z_axis(3,0.0);
        z_axis(2)=-1;

        Vector cross_prod=cross(final_H.getCol(1).subVector(0,2), z_axis);

        axis.setSubvector(0,cross_prod/norm(cross_prod));

        axis(3)=acos(dot(final_H.getCol(1).subVector(0,2), z_axis)/(norm(final_H.getCol(1).subVector(0,2))));

        rot_x=axis2dcm(axis).submatrix(0,2,0,2);
    }

    cout<<endl;
    yDebug()<<"H final later ";
    yDebug()<<(rot_x * final_H.submatrix(0,2,0,2)).toString();
    cout<<endl;

    final_H.setSubmatrix(rot_x * final_H.submatrix(0,2,0,2), 0, 0);

}




























