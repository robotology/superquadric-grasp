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

 /**
  * This class computes the grasping pose for a given hand
  * and a superquadric modeling an objct by solving an optimization
  * problem with the Ipopt software package.
  */
class grasping_NLP : public Ipopt::TNLP
{
    /** Quantities for computing the solution */
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
    /** Hand ellipsoid */
    yarp::sig::Vector hand;
    /** Object superquadric */
    yarp::sig::Vector object;
    std::deque<yarp::sig::Vector> obstacles;
    yarp::sig::Matrix plane;
    /** Vector with the pose computed with the hand ellipsoid */
    yarp::sig::Vector solution;
    /** Final robot pose */
    yarp::sig::Vector robot_pose;
    /** Variable for setting hand of interest */
    std::string l_o_r;
    double final_F_value;
    int num_superq;
    int max_superq;

    /** Initialization function
    * @param objectext is the object superquadric
    * @param handext is the hand ellipsoid
    * @param n_handpoints is the number of points to sample on the hand ellipsoid
    * @param str_hand is the name of the hand
    */
    /****************************************************************/
    void init(const yarp::sig::Vector &objectext, yarp::sig::Vector &handext,const std::deque<yarp::sig::Vector> &obstacleext, int &n_handpoints, const std::string &str_hand);

    /**  Check if z axis has the maximum dimension for adjusting superquadric representation */
    /****************************************************************/
    void checkZbound();

    /** Samples points on the hand ellipsoid
    * @param hand is the hand ellipsoid
    * @param j is an index
    * @param i is an index
    * @param str_hand is the name of the hand
    * @return a vector with the points sampled on the hand ellipsoid
    */
    /****************************************************************/
    yarp::sig::Vector computePointsHand(yarp::sig::Vector &hand, int j, int l, const std::string &str_hand, double &theta);

    /** Get info for the nonlinear problem to be solved with ipopt
    * @param n is the dimension of the variable
    * @param m is the number of constraints
    * @param nnz_jac_g is the dimensions of the jacobian
    * @param nnz_h_lag is an ipopt variable
    * @param index_styl is an ipopt variable
    * @return true
    */
    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);

    /** Get variable bounds for the nonlinear problem to be solved with ipopt
    * @param n is the dimension of the variable
    * @param m is the number of constraints
    * @param x_l is the lower bound of the variable
    * @param x_u is the upper bound of the variable
    * @param g_l is the lower bound of the constraints
    * @param g_u is the upper bound of the constraints
    * @return true
    */
    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);

    /** Get the starting point for the nonlinear problem to be solved with ipopt
    * @param n is the dimension of the variable
    * @param init_x is the starting point of the optimization problem
    * @param x is the variable
    * @param init_z is an ipopt variable
    * @param z_L is an ipopt variable
    * @param z_U is an ipopt variable
    * @param m is the number of constraints
    * @param init_lambda is an ipopt variable
    * @param lambda is an ipopt variable
    * @return true
    */
    /****************************************************************/
     bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

    /** Cost function of the nonlinear problem to be solved with ipopt
    * @param n is the dimension of the variable
    * @param x is the variable
    * @param new_x takes into account is the variable has been updated or not
    * @param obj_value is the value of the cost function
    * @retun true
    */
    /****************************************************************/
     bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value);

    /** Auxiliary function for computing cost function of the nonlinear problem to be solved with ipopt
    * @param x is the variable
    * @param points_on is object point cloud
    * @param new_x takes into account is the variable has been updated or not
    * @retun the cost function value
    */
    /****************************************************************/
     double F(const Ipopt::Number *x, std::deque<yarp::sig::Vector> &points_on, bool new_x);

     /** Auxiliary function for computing cost function of the nonlinear problem to be solved with ipopt
    * @param obj is the Vector of the object
    * @param x is the variable
    * @param point is one point of the point cloud
    * @return a part of the cost function value
    */
    /****************************************************************/
     double f(yarp::sig::Vector &obj, const Ipopt::Number *x, yarp::sig::Vector &point);

    /** Auxiliary function for computing the gradient of cost function of the nonlinear problem
    * @param x is the variable
    * @param points_on is one point of object point cloud
    * @return cost function value
    */
    /****************************************************************/
     double F_v(yarp::sig::Vector &x, std::deque<yarp::sig::Vector> &points_on);

     /** Auxiliary function for computing the gradient cost function of the nonlinear problem
     * @param obj is the Vector of the object
     * @param x is the variable
     * @param point is one point of the point cloud
     * @return a part of the cost function value
     */
     /****************************************************************/
     double f_v(yarp::sig::Vector &obj, yarp::sig::Vector &x, yarp::sig::Vector &point);

     /** Auxiliary function for computing the gradient cost function of the nonlinear problem
     * @param obj is the Vector of the object
     * @param x is the variable
     * @param point_tr is one point of the point cloud
     * @return a part of the cost function value
     */
     /********************************************************************/
     double f_v2(yarp::sig::Vector &obj, yarp::sig::Vector &x, yarp::sig::Vector &point_tr);

     /** Gradient of the cost function of the nonlinear problem
     * @param x is the variable
     * @param n is the dimension of the variable
     * @param new_x takes into account is the variable has been updated or not
     * @param grad_f is the gradient of the cost function
     */
    /****************************************************************/
     bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Number *grad_f);

     /** Constraints of the nonlinear problem
     * @param n is the dimension of the variable
     * @param x is the variable
     * @param m is the number of constraints
     * @param new_x takes into account is the variable has been updated or not
     * @param g is the values of the constraints
     * @return true
     */
    /****************************************************************/
     bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Number *g);

     /** Auxiliary function for computing the constraints of the nonlinear problem
    * @param x is the variable
    * @param i is the number of the constraints
    * @return the constraints value
    */
    /****************************************************************/
    double G_v(yarp::sig::Vector &x, int i, Ipopt::Index m);

    /**  Jacobian of the constraints of the nonlinear problem
    * @param n is the dimension of the variable
    * @param x is the variable
    * @param m is the number of constraints
    * @param new_x takes into account is the variable has been updated or not
    * @param iRow contains the jacobian raws
    * @param iCol contains the jacobian columns
    * @param values contains the jacobian values
    * @return true
    */
    /****************************************************************/
     bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                     Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                     Ipopt::Index *jCol, Ipopt::Number *values);

    /** Configure function
    * @param rf is the resource finder
    * @param left_or_right can be right, left or both
    * @param disp is the hand ellispoid displacement
    * @param pl is the plane representing the table
    */
    /****************************************************************/
    void configure(yarp::os::ResourceFinder *rf, const std::string &left_or_right, const yarp::sig::Vector &disp, const yarp::sig::Vector &pl);

    /** Function for reading matrices from config files
    * @param tag is the name of the quantity to be read from text
    * @param matrix is the matrix to be filled
    * @param dimensions is the matrix dimensions
    * @param rf is the resource finder
    * @return true/false on success/failure
    */
    /****************************************************************/
    bool readMatrix(const std::string &tag, yarp::sig::Matrix &matrix, const int &dimension, yarp::os::ResourceFinder *rf);

    /** Finalize the solution
    * @param n is the dimension of the variable
    * @param x is the variable
    * @param m is the number of constraints
    * @param init_z is an ipopt variable
    * @param z_L is an ipopt variable
    * @param z_U is an ipopt variable
    * @param status says if the problem has been solved or not
    * @param obj_value is the final cost function values
    */
    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                          const Ipopt::Number *x, const Ipopt::Number *z_L,
                          const Ipopt::Number *z_U, Ipopt::Index m,
                          const Ipopt::Number *g, const Ipopt::Number *lambda,
                          Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                          Ipopt::IpoptCalculatedQuantities *ip_cq);

   /** Extract the solution
   * @return the superquadric as a Vector
   */
   /****************************************************************/
   yarp::sig::Vector get_result() const;

   /** Get the hand ellipsoid pose
   * @return the hand ellipsoid in the final pose
   */
   /****************************************************************/
   yarp::sig::Vector get_hand() const;

   /** Return the final cost function of the solution
   *@return a double value of the final cost function
   */
   /****************************************************************/
   double get_final_F() const;
};



#endif
