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
#ifndef __COMPUTATION_H__
#define __COMPUTATION_H__

#include <yarp/dev/all.h>

#include "superquadric.h"

 /**
  * This class computes the grasping pose for grasping and object
  * once the superquadric modeling the object is provided.
  * The solution is given by solving an optimization problem
  * with the Ipopt software package.
  */
/*******************************************************************************/
class GraspComputation
{
protected:

    double tol_single, constr_viol_tol_single;
    double tol_multiple, constr_viol_tol_multiple;
    int max_iter, acceptable_iter, object_provided;
    std::string mu_strategy,nlp_scaling_method;
    double max_cpu_time;

    int n_pointshand;
    double distance, distance1;
    std::string dir;
    yarp::sig::Vector displacement;
    yarp::sig::Vector plane;

    yarp::os::Property ipopt_par;
    yarp::os::Property pose_par;
    yarp::os::Property trajectory_par;

    bool go_on;
    double t0, t_grasp;

    yarp::os::Mutex mutex;

    yarp::os::ResourceFinder *rf;

    int print_level;

public:

    std::string left_right;
    double hand_length_right;
    double hand_length_left;

    /** Vector for representing one hand ellipsoid*/
    yarp::sig::Vector &hand;
    /** Vector for representing one hand ellipsoid*/
    yarp::sig::Vector &hand1;
    /** Complete solution computed */
    yarp::os::Property &complete_sol;
    /** Object superquadric */
    const yarp::sig::Vector &object;
    const std::deque<yarp::sig::Vector> &obstacles;
    int count_file_old;
    int count_file;
    bool multiple_superq;
    std::string best_hand;
    double final_value_R, final_value_L, cos_zr, cos_zl, cos_xr, cos_xl;
    std::deque<double> final_obstacles_value_R, final_obstacles_value_L;

    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;
    yarp::sig::Vector poseR, solR;
    yarp::sig::Vector poseL, solL;

    // For selecting the best hand
    std::string robot;
    double torso_pitch_max;
    int context_right;
    int context_left;


    yarp::dev::ICartesianControl *icart_right;
    yarp::dev::ICartesianControl *icart_left;

    yarp::dev::PolyDriver robotDevice_right;
    yarp::dev::PolyDriver  robotDevice_left;

    double &cost_right, &cost_left;

    /*******************************************************************************/
    GraspComputation(const yarp::os::Property &_ipopt_par, const yarp::os::Property &_pose_par,
                     const yarp::os::Property &_trajectory_par, const std::string &_left_or_right,
                     yarp::sig::Vector &_hand, yarp::sig::Vector &_hand1, yarp::os::ResourceFinder *_rf,
                     yarp::os::Property &_complete_sol, const yarp::sig::Vector &_object, const std::deque<yarp::sig::Vector> &_obstacle, double &_cost_right, double &_cost_left, bool &_multiple_superq);

    /** Set parameters for computing the solution with ipopt
     * @param newOptions is a Property with the new options to be set
     * @param first_time takes into account if it is the first the options are set or not
     */
    /***********************************************************************/
    void setIpoptPar(const yarp::os::Property &newOptions, bool first_time);

    /** Get parameters used for computing the solution with ipopt
    * @return a Property with all the options for ipopt
    */
    /***********************************************************************/
    yarp::os::Property getIpoptPar();

    /** Set parameters for correctly compute the grasping pose
     * @param newOptions is a Property with the new options to be set
     * @param first_time takes into account if it is the first the options are set or not
     */
    /***********************************************************************/
    void setPosePar(const yarp::os::Property &newOptions, bool first_time);

    /** Get parameters for correctly compute the grasping pose
     * @return a Property with all the options for pose computation
     */
    /***********************************************************************/
    yarp::os::Property getPosePar();

    /** Set parameters for correctly compute the trajectory for approaching the desired pose
     * @param newOptions is a Property with the new options to be set
     * @param first_time takes into account if it is the first the options are set or not
     */
    /***********************************************************************/
    void setTrajectoryPar(const yarp::os::Property &newOptions, bool first_time);

    /** Get parameters used for correctly compute the trajectory for approaching the desired pose
     * @return a Property with all the options for pose computation
    */
    /***********************************************************************/
    yarp::os::Property getTrajectoryPar();

    /** Init function
    * @return true/false on success/failure
    */
    /***********************************************************************/
    bool init();

    /** Run function */
    /***********************************************************************/
    void run();

    /** Compute a given pose for the selected hand
    *  @param hand is the hand ellipsoid
    *  @param left_or_right if the string of the hand: right, left or both
    *  @return true/false on success/failure
    */
    /***********************************************************************/
    bool computePose(yarp::sig::Vector &hand, const std::string &left_or_right);

    /** Compute the trajectory for the selected hand
    * @param chosen_hand is the hand selected for moving
    * @direction is an option for building the trajectory. It can be "z" or "xz",
    * according to which direction is used for shifting the trajectory waypoints.
    */
    /***********************************************************************/
    bool computeTrajectory(const std::string &chosen_hand, const std::string &direction);

    /** Extract the solution from ipopt interface
    * @param hand is the selected hand
    */
    /***********************************************************************/
    void getSolution(const std::string &hand);

    /** Return computation time for getting the pose
    * @return the period value
    */
    /***********************************************************************/
    double getTime();

    /** Properly fill a property with the computed solution
    * @param hand is the hand string
    * @return the Property with the information inside
    */
    /**********************************************************************/
    yarp::os::Property fillProperty(const std::string &hand);

    /** Set a a parameter equal to a value
     * @param tag is the name of the parameter
     * @param value is the new value of parameter
     */
    /**********************************************************************/
    void setPar(const std::string &tag, const std::string &value);

    /***********************************************************************/
    void bestPose();

    /***********************************************************************/
    bool setCartesianRight();

    /***********************************************************************/
    bool setCartesianLeft();
};

#endif
