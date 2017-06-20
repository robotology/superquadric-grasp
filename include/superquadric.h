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

#ifndef __SUPERQUADRIC_H__
#define __SUPERQUADRIC_H__

#include <string>
#include <deque>
#include <map>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

class grasping_NLP : public Ipopt::TNLP
{
    double aux_objvalue;
    yarp::sig::Vector obj;
    yarp::sig::Vector x_v;
    yarp::sig::Matrix bounds;
    yarp::sig::Matrix bounds_constr;
    yarp::sig::Vector x0;
    std::deque<yarp::sig::Vector> points_on;
    yarp::sig::Matrix H_o2w;
    yarp::sig::Matrix H_h2w;
    yarp::sig::Matrix H_x;
    yarp::sig::Vector euler;
    yarp::sig::Vector displacement;

public:
    yarp::sig::Vector hand;
    yarp::sig::Vector object;
    yarp::sig::Matrix plane;
    yarp::sig::Vector solution;
    yarp::sig::Vector robot_pose;
    std::string l_o_r;

    /****************************************************************/
    void init(const yarp::sig::Vector &objectext, yarp::sig::Vector &handext, int &n_handpoints, const std::string &str_hand);

    /****************************************************************/
    void checkZbound();

    /****************************************************************/
    yarp::sig::Vector computePointsHand(yarp::sig::Vector &hand, int j, int l, const std::string &str_hand, double &theta);

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);

    /****************************************************************/
     bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

    /****************************************************************/
     bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value);

    /****************************************************************/
     int bigComponent();

    /****************************************************************/
     double F(const Ipopt::Number *x, std::deque<yarp::sig::Vector> &points_on, bool new_x);

    /****************************************************************/
     double f(yarp::sig::Vector &obj, const Ipopt::Number *x, yarp::sig::Vector &point);

    /****************************************************************/
     double F_v(yarp::sig::Vector &x, std::deque<yarp::sig::Vector> &points_on);

    /****************************************************************/
     double f_v(yarp::sig::Vector &obj, yarp::sig::Vector &x, yarp::sig::Vector &point);

     /********************************************************************/
     double f_v2(yarp::sig::Vector &obj, yarp::sig::Vector &x, yarp::sig::Vector &point_tr);

    /****************************************************************/
     bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Number *grad_f);
    /****************************************************************/
     bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Number *g);

    /****************************************************************/
    double G_v(yarp::sig::Vector &x, int i);

    /****************************************************************/
     bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                     Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                     Ipopt::Index *jCol, Ipopt::Number *values);

    /****************************************************************/
    void configure(yarp::os::ResourceFinder *rf, const std::string &left_or_right, const yarp::sig::Vector &disp, const yarp::sig::Vector &pl);

    /****************************************************************/
    bool readMatrix(const std::string &tag, yarp::sig::Matrix &matrix, const int &dimension, yarp::os::ResourceFinder *rf);

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                          const Ipopt::Number *x, const Ipopt::Number *z_L,
                          const Ipopt::Number *z_U, Ipopt::Index m,
                          const Ipopt::Number *g, const Ipopt::Number *lambda,
                          Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                          Ipopt::IpoptCalculatedQuantities *ip_cq);

   /****************************************************************/
   yarp::sig::Vector get_result() const;

   /****************************************************************/
   yarp::sig::Vector get_hand() const;













};



#endif
