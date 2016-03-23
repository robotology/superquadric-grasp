
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


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class Object
{
public:
    Vector x;
    BufferedPort<Bottle> portInObject;

    /****************************************************************/
    void init(ResourceFinder &rf)
    {
        //string namePorts = rf.check("namePorts", Value("superQuadrics"), "Getting module name").asString();
        //portInObject.open("/"+namePorts+"/obj:i");

    }

    /****************************************************************/
    void getObject(int &object_provided, ResourceFinder &rf)
    {
        if(object_provided==1)
            readObject("object",x,11,rf);
        else
        {
            Bottle *pointIn=portInObject.read(true);
             if (pointIn!=NULL)
             {
                 int bsize=pointIn->size();
                 for(size_t i=0; i<bsize; i++)
                 {
                     Bottle *pointList=pointIn->get(i).asList();

                     x[0]=pointList->get(0).asDouble();
                     x[1]=pointList->get(1).asDouble();
                     x[2]=pointList->get(2).asDouble();
                     x[3]=pointList->get(3).asDouble();
                     x[4]=pointList->get(4).asDouble();
                     x[5]=pointList->get(5).asDouble();
                     x[6]=pointList->get(6).asDouble();
                     x[7]=pointList->get(7).asDouble();
                     x[8]=pointList->get(8).asDouble();
                     x[9]=pointList->get(9).asDouble();
                     x[10]=pointList->get(10).asDouble();

                     cout<<"object received "<<x.toString().c_str()<<endl;
                 }
              }
           portInObject.close();
        }
    }

    /****************************************************************/
    bool readObject(const string &tag, Vector &x, const int &dimension, ResourceFinder &rf)
    {
        if (Bottle *b=rf.find(tag.c_str()).asList())
        {
            if (b->size()>=dimension)
            {
                for(size_t i=0; i<b->size();i++)
                    x.push_back(b->get(i).asDouble());
            }
            return true;
        }
    }
};


class Hand : public Object
{
public:
    Vector x;

    /****************************************************************/
    void init(Object &obj, ResourceFinder &rf)
    {
        readObject("hand",x,11,rf);
        //x[1]=findMax(obj.x.subVector(0,2));
    }
};

class grasping_NLP : public Ipopt::TNLP
{   
    int l;
    bool first_gof;
    bool first_gogr;
    double aux_objvalue;

    Vector obj;
    Vector x_v;
    Matrix bounds;
    Vector x0;
    deque<Vector> points_on;

    Vector aux_gradf;
    Vector old_x_f;
    Vector old_x_gr;

    Hand hnd;

public:
    double t0;
    deque<double> t;
    string gradient_comp;
    Vector solution;
    Vector robot_pose;

    Matrix H_o2w;
    Matrix H_h2w;
    Matrix H_x;
    Vector euler;

    /****************************************************************/
    void init(Object &obj_received, ResourceFinder &rf)
    {
        obj.resize(11,0.0);
        obj=obj_received.x;

        hnd.init(obj_received,rf);

        H_o2w.resize(4,4);
        H_h2w.resize(4,4);
        H_x.resize(4,4);
        euler.resize(3,0.0);

        l=rf.find("points_hand").asInt();

        for(size_t i=0; i<l;i++)
            points_on.push_back(computePointsHand(hnd,i, l));

        for(size_t i=0; i<l;i++)
        {
            Vector point=points_on[i];
            cout<<point.toString()<<endl;
        }

        aux_objvalue=0.0;
        first_gof=true;
        first_gogr=true;

    }

    /****************************************************************/
    void checkZbound()
    {
        euler[0]=obj[8];
        euler[1]=obj[9];
        euler[2]=obj[10];
        H_o2w=euler2dcm(euler);
        euler[0]=obj[5];
        euler[1]=obj[6];
        euler[2]=obj[7];
        H_o2w.setSubcol(euler,0,3);

        Vector obj_rot(3,0.0);
        Vector aux(4,0.0);
        Vector aux2(4,0.0);

        aux2[0]=obj[0];
        aux2[3]=1;
        aux=H_o2w*aux2;
        obj_rot[0]=norm(aux.subVector(0,2)-obj.subVector(5,7));

        aux2[0]=0;
        aux2[1]=obj[1];
        aux2[3]=1;
        aux=H_o2w*aux2;
        obj_rot[1]=norm(aux.subVector(0,2)-obj.subVector(5,7));

        aux2[1]=0;
        aux2[2]=obj[2];
        aux2[3]=1;
        aux=H_o2w*aux2;
        obj_rot[2]=norm(aux.subVector(0,2)-obj.subVector(5,7));

        int j;
        Vector prod(3,0.0);
        Vector x,y,z;
        x.resize(3,0.0);
        y.resize(3,0.0);
        z.resize(3,0.0);
        x=H_o2w.getCol(0).subVector(0,2);
        y=H_o2w.getCol(1).subVector(0,2);
        z=H_o2w.getCol(2).subVector(0,2);

        aux.resize(3,0.0);
        aux[0]=1;
        prod[0]=abs(x[0]*aux[0]+x[1]*aux[1]+x[2]*aux[2]);
        aux[0]=0; aux[1]=1;
        prod[1]=abs(y[0]*aux[0]+y[1]*aux[1]+y[2]*aux[2]);
        aux[1]=0; aux[2]=1;
        prod[2]=abs(z[0]*aux[0]+z[1]*aux[1]+z[2]*aux[2]);

        for(size_t i=0; i<3; i++)
        {
            if(findMax(prod)==prod[i])
                j=i;
        }

        cout<<j<<endl;
        cout<<"obj rot "<<obj_rot.toString()<<endl;

        if(hnd.x[2]<obj_rot[j])
            bounds(0,7)=obj[7];
        else
            bounds(0,7)=obj[7]+hnd.x[2]-obj_rot[j];
    }

    /****************************************************************/
    Vector computePointsHand(Hand &hnd, int j, int l)
    {
        Vector point(3,0.0);
        double theta;

        if(j<l/3)
        {
            theta=j*M_PI/8;
            point[0]=cos(theta)*hnd.x[0];
            point[1]=sin(theta)*hnd.x[1];
            point[2]=0.0;

        }
        if(j>=l/3 && j<l/3*2)
        {
            theta=j*M_PI/16;
            point[0]=cos(theta)*hnd.x[0];
            point[1]=0.0;
            point[2]=sin(theta)*hnd.x[2];


        }
        if(j>=l/3*2 && j<l)
        {
            theta=j*M_PI/16+M_PI;
            point[0]=0.0;
            point[1]=cos(theta)*hnd.x[1];
            point[2]=sin(theta)*hnd.x[2];

        }


        Vector point_tr(4,0.0);

        euler[0]=hnd.x[8];
        euler[1]=hnd.x[9];
        euler[2]=hnd.x[10];
        H_h2w=euler2dcm(euler);
        euler[0]=hnd.x[5];
        euler[1]=hnd.x[6];
        euler[2]=hnd.x[7];
        H_h2w.setSubcol(euler,0,3);

        Vector point_tmp(4,1.0);
        point_tmp.setSubvector(0,point);
        point_tr=H_h2w*point_tmp;
        point=point_tr.subVector(0,2);

        return point;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style)
    {
        n=6;
        m=5;
        nnz_jac_g=30;
        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        x_v.resize(6,0.0);
        bounds.resize(6,2);

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
           x_l[i]=bounds(i,0);
           x_u[i]=bounds(i,1);
        }

        g_l[0]=-1.0;
        g_u[0]=0.2;
        g_l[1]=-1.0;
        g_u[1]=-1.0;
        g_l[2]=0.0;
        g_u[2]=1.0;

        g_l[3]=-1.0;
        g_u[3]=0.0;
        g_l[4]=-0.5;
        g_u[4]=0.5;

        return true;
    }

    /****************************************************************/
     bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
     {
         for(Ipopt::Index i=0;i<n;i++)
         {
             x[i]=hnd.x[i+5];
             old_x_f[i]=x[i];
             old_x_gr[i]=x[i];
         }
         return true;
     }

     /****************************************************************/
     bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value)
     {
         F(x,points_on);
         obj_value=aux_objvalue;

         return true;
     }

     /****************************************************************/
     double F(const Ipopt::Number *x, deque<Vector> &points_on)
     {
         Vector x_new(11,0.0);

         for(size_t i=0;i<11;i++)
            x_new[i]=x[i];

         if(!(x_new==old_x_f)|| first_gof)
         {
             first_gof=false;
             double value=0.0;

             for(size_t i=0;i<points_on.size();i++)
                 value+= pow( pow(f(obj,x,points_on[i]),obj[3])-1,2 );

              value/=points_on.size();
              aux_objvalue=value;

              for(size_t i=0;i<11;i++)
                  old_x_f[i]=x[i];
         }
     }


      /****************************************************************/
     double f(Vector &obj, const Ipopt::Number *x, Vector &point)
     {
         euler[0]=obj[8];
         euler[1]=obj[9];
         euler[2]=obj[10];
         H_o2w=euler2dcm(euler);
         euler[0]=obj[5];
         euler[1]=obj[6];
         euler[2]=obj[7];
         H_o2w.setSubcol(euler,0,3);

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

         double num1, num2, num3;
         num1=H_o2w(0,0)*point_tr[0]+H_o2w(0,1)*point_tr[1]+H_o2w(0,2)*point_tr[2]-obj[5]*H_o2w(0,0)-obj[6]*H_o2w(0,1)-obj[7]*H_o2w(0,2);
         num2=H_o2w(1,0)*point_tr[0]+H_o2w(1,1)*point_tr[1]+H_o2w(1,2)*point_tr[2]-obj[5]*H_o2w(1,0)-obj[6]*H_o2w(1,1)-obj[7]*H_o2w(1,2);
         num3=H_o2w(2,0)*point_tr[0]+H_o2w(2,1)*point_tr[1]+H_o2w(2,2)*point_tr[2]-obj[5]*H_o2w(2,0)-obj[6]*H_o2w(2,1)-obj[7]*H_o2w(2,2);

         double tmp=pow(abs(num1/obj[0]),2.0/obj[4]) + pow(abs(num2/obj[1]),2.0/obj[4]);

         return pow( abs(tmp),obj[4]/obj[3]) + pow( abs(num3/obj[2]),(2.0/obj[3]));
     }

     /****************************************************************/
     double F_v(Vector &x, deque<Vector> &points_on)
     {
         double value=0.0;

         for(size_t i=0;i<points_on.size();i++)
            value+= pow( pow(f_v(obj,x,points_on[i]),obj[3])-1,2 );l;

         return value/points_on.size();
     }

      /****************************************************************/
     double f_v(Vector &obj, Vector &x, Vector &point)
     {
         euler[0]=obj[8];
         euler[1]=obj[9];
         euler[2]=obj[10];
         H_o2w=euler2dcm(euler);
         euler[0]=obj[5];
         euler[1]=obj[6];
         euler[2]=obj[7];
         H_o2w.setSubcol(euler,0,3);

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

         double num1, num2, num3;
         num1=H_o2w(0,0)*point_tr[0]+H_o2w(0,1)*point_tr[1]+H_o2w(0,2)*point_tr[2]-obj[5]*H_o2w(0,0)-obj[6]*H_o2w(0,1)-obj[7]*H_o2w(0,2);
         num2=H_o2w(1,0)*point_tr[0]+H_o2w(1,1)*point_tr[1]+H_o2w(1,2)*point_tr[2]-obj[5]*H_o2w(1,0)-obj[6]*H_o2w(1,1)-obj[7]*H_o2w(1,2);
         num3=H_o2w(2,0)*point_tr[0]+H_o2w(2,1)*point_tr[1]+H_o2w(2,2)*point_tr[2]-obj[5]*H_o2w(2,0)-obj[6]*H_o2w(2,1)-obj[7]*H_o2w(2,2);

         double tmp=pow(abs(num1/obj[0]),2.0/obj[4]) + pow(abs(num2/obj[1]),2.0/obj[4]);

         return pow( abs(tmp),obj[4]/obj[3]) + pow( abs(num3/obj[2]),(2.0/obj[3]));
     }

     /****************************************************************/
     bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Number *grad_f)
     {
         Vector x_tmp(6,0.0);
         Vector x_new(11,0.0);
         double grad_p, grad_n;
         double eps=1e-6;

         for(Ipopt::Index i=0;i<n;i++)
         {
             x_tmp[i]=x[i];
             x_new[i]=x[i];
         }

         if(!(x_new==old_x_gr)|| first_gogr)
         {
             first_gogr=false;

             for(Ipopt::Index j=0;j<n;j++)
             {
                 x_tmp[j]=x_tmp[j]+eps;

                 grad_p=F_v(x_tmp,points_on);

                 x_tmp[j]=x_tmp[j]-eps;

                 grad_n=F_v(x_tmp,points_on);

                 aux_gradf[j]=(grad_p-grad_n)/eps;
                 old_x_gr[j]=x[j];
             }
         }

         for(Ipopt::Index j=0;j<n;j++)
         {
              grad_f[j]=aux_gradf[j];
         }

         return true;
     }

     /****************************************************************/
     bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Number *g)
     {
         euler[0]=obj[8];
         euler[1]=obj[9];
         euler[2]=obj[10];
         H_o2w=euler2dcm(euler);
         euler[0]=obj[5];
         euler[1]=obj[6];
         euler[2]=obj[7];
         H_o2w.setSubcol(euler,0,3);

         Matrix H;
         H.resize(4,4);
         euler[0]=x[3];
         euler[1]=x[4];
         euler[2]=x[5];
         H_x=euler2dcm(euler);

         euler[0]=hnd.x[8];
         euler[1]=hnd.x[9];
         euler[2]=hnd.x[10];
         H_h2w=euler2dcm(euler);
         H=H_x*H_h2w;

         g[0]=H(2,2);
         g[1]=H(0,0);
         g[2]=H(1,1);
         g[3]=H(2,1);
         g[4]=H(2,0);

         return true;
     }

     /****************************************************************/
     double G_v(Vector &x, int i)
     {
         Vector g(3,0.0);

         Matrix H_x,H;
         H_x.resize(4,4);
         H.resize(4,4);
         euler[0]=x[3];
         euler[1]=x[4];
         euler[2]=x[5];
         H_x=euler2dcm(euler);

         euler[0]=hnd.x[8];
         euler[1]=hnd.x[9];
         euler[2]=hnd.x[10];
         H_h2w=euler2dcm(euler);
         H=H_x*H_h2w;

         g[0]=H(2,2);
         g[1]=H(0,0);
         g[2]=H(1,1);
         g[3]=H(1,2);
         g[4]=H(2,0);

         return g[i];
     }

     /****************************************************************/
     bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
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

                     grad_p=G_v(x_tmp,i);
                     x_tmp[j]=x_tmp[j]-eps;

                     grad_n=G_v(x_tmp,i);

                     values[count]=(grad_p-grad_n)/(eps);
                     count++;

                 }
             }

         }
         else
         {
                jCol[0]=0;
                jCol[1]=1;
                jCol[2]=2;
                jCol[3]=3;
                jCol[4]=4;
                jCol[5]=5;
                jCol[6]=0;
                jCol[7]=1;
                jCol[8]=2;
                jCol[9]=3;
                jCol[10]=4;
                jCol[11]=5;
                jCol[12]=0;
                jCol[13]=1;
                jCol[14]=2;
                jCol[15]=3;
                jCol[16]=4;
                jCol[17]=5;

                jCol[18]=0;
                jCol[19]=1;
                jCol[20]=2;
                jCol[21]=3;
                jCol[22]=4;
                jCol[23]=5;

                jCol[24]=0;
                jCol[25]=1;
                jCol[26]=2;
                jCol[27]=3;
                jCol[28]=4;
                jCol[29]=5;

                iRow[0]=iRow[1]=iRow[2]=iRow[3]=iRow[4]=iRow[5]=0;
                iRow[6]=iRow[7]=iRow[8]=iRow[9]=iRow[10]=iRow[11]=1;
                iRow[12]=iRow[13]=iRow[14]=iRow[15]=iRow[16]=iRow[17]=2;

                iRow[18]=iRow[19]=iRow[20]=iRow[21]=iRow[22]=iRow[23]=3;
                iRow[24]=iRow[25]=iRow[26]=iRow[27]=iRow[28]=iRow[29]=4;

         }

     return true;
     }

    /****************************************************************/
    void configure(ResourceFinder &rf)
    {
        aux_gradf.resize(6,0.0);
        old_x_f.resize(6,0.0);
        old_x_gr.resize(6,0.0);
        Matrix x0_tmp;
        x0_tmp.resize(6,1);
        x0.resize(6,0.0);
        readMatrix("x0",x0_tmp,1,rf);
        for(size_t i=0; i< 6; i++)
            x0[i]=x0_tmp(i,0);

        bounds.resize(6,2);
        readMatrix("bounds",bounds, 6, rf);
        checkZbound();
    }

    /****************************************************************/
   bool readMatrix(const string &tag, Matrix &matrix, const int &dimension, ResourceFinder &rf)
   {
       string tag_x=tag+"_x";
       string tag_y=tag+"_y";
       bool check_x;

       if(tag=="x0")
       {
           if (Bottle *b=rf.find(tag.c_str()).asList())
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
           if(tag=="bounds")
           {
               tag_x=tag+"_l";
               tag_y=tag+"_u";
           }

           if (Bottle *b=rf.find(tag_x.c_str()).asList())
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
           if (Bottle *b=rf.find(tag_y.c_str()).asList())
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
   void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
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

       euler[0]=hnd.x[8];
       euler[1]=hnd.x[9];
       euler[2]=hnd.x[10];
       H_h2w=euler2dcm(euler);
       euler[0]=hnd.x[5];
       euler[1]=hnd.x[6];
       euler[2]=hnd.x[7];
       H_h2w.setSubcol(euler,0,3);

       Matrix H;
       H.resize(4,4);
       H=H_x*H_h2w;
       solution.setSubvector(3,dcm2euler(H.transposed()));

       for (Ipopt::Index i=0; i<3; i++)
           solution[i]=x[i];

        robot_pose.resize(6,0.0);
        robot_pose=solution;
        robot_pose.setSubvector(0,solution.subVector(0,2)-hnd.x[0]*(H.transposed().getCol(2).subVector(0,2)));

        cout<<"robot pose "<< robot_pose.toString()<<endl;

       for(size_t i=0;i<l;i++)
       {
           Vector point(3,0.0);
           point=points_on[i];

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
           cout<<point_tr.subVector(0,2).toString()<<endl;
       }
   }

   /****************************************************************/
   Vector get_result() const
   {
       return solution;
   }

};

/****************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.configure(argc,argv);
    double t,t0;

    string nameFileOut, nameFileSolution;
    string mu_strategy,nlp_scaling_method;
    double tol,constr_viol_tol, sum;
    int acceptable_iter,max_iter;
    int object_provided;
    Object obj_main;

    nameFileOut=rf.find("nameFileOut").asString().c_str();
    if(rf.find("nameFileOut").isNull())
       nameFileOut="test";

    nameFileSolution=rf.find("nameFileSolution").asString().c_str();
    if(rf.find("nameFileSolution").isNull())
       nameFileSolution="solution.txt";

    tol=rf.find("tol").asDouble();
    if(rf.find("tol").isNull())
       tol=1e-4;

    constr_viol_tol=rf.find("constr_tol").asDouble();
    if(rf.find("constr_tol").isNull())
       constr_viol_tol=1e-2;

    acceptable_iter=rf.find("acceptable_iter").asInt();
    if(rf.find("acceptable_iter").isNull())
       acceptable_iter=0;

    max_iter=rf.find("max_iter").asInt();
    if(rf.find("max_iter").isNull())
       max_iter=2e19;

    mu_strategy=rf.find("mu_strategy").asString().c_str();
    if(rf.find("mu_strategy").isNull())
       mu_strategy="monotone";

    nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
    if(rf.find("nlp_scaling_method").isNull())
       nlp_scaling_method="none";    

    object_provided=rf.find("object_prov").asInt();
    if(rf.find("object_prov").isNull())
        object_provided=1;

    obj_main.init(rf);
    obj_main.getObject(object_provided,rf);


    //algorithm settings
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",tol);
    app->Options()->SetNumericValue("constr_viol_tol",constr_viol_tol);
    app->Options()->SetIntegerValue("acceptable_iter",acceptable_iter);
    app->Options()->SetStringValue("mu_strategy",mu_strategy);
    app->Options()->SetIntegerValue("max_iter",max_iter);
    app->Options()->SetStringValue("nlp_scaling_method",nlp_scaling_method);
    //app->Options()->SetNumericValue("nlp_scaling_max_gradient",10);
    //app->Options()->SetNumericValue("nlp_scaling_min_value",1e-2);
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test","first-order");
    app->Options()->SetStringValue("derivative_test_print_all","yes");
    app->Options()->SetStringValue("output_file",nameFileOut+".out");
    //app->Options()->SetStringValue("print_user_options","yes");
    //app->Options()->SetStringValue("print_options_documentation","no");
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    t0=Time::now();
    Ipopt::SmartPtr<grasping_NLP>  grasp_nlp= new grasping_NLP;
    grasp_nlp->init(obj_main,rf);
    grasp_nlp->configure(rf);

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(grasp_nlp));
    t=Time::now()-t0;

    if(status==Ipopt::Solve_Succeeded)
    {
        Vector sol;
        sol=grasp_nlp->get_result();
        cout<<"The optimal solution is: "<<sol.toString().c_str()<<endl;
        ofstream fout(nameFileSolution.c_str());

        if(fout.is_open())
        {
            fout<<"x: "<<endl<<endl;
            fout<<sol.toString().c_str()<<endl<<endl;

            fout<<"hand pose "<<endl<<endl;
            fout<<grasp_nlp->robot_pose.toString()<<endl<<endl;

            fout<<"t for ipopt: "<<endl<<endl;
            fout<<t<<endl<<endl;

            fout<<"method to compute gradient:"<<endl<<endl;
            fout<<grasp_nlp->gradient_comp<<endl;

            fout<<"object: "<<endl;
            fout<<obj_main.x.toString()<<endl;
        }
    }
    else
        cout<<"Problem not solved!"<<endl; 

    Vector sol;
    sol=grasp_nlp->get_result();
    cout<<"The optimal solution is: "<<sol.toString().c_str()<<endl;

    cout<<"t "<<t<<endl;

    return 0;
}
