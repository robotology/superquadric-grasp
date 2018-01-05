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

	/** Hand to be enabled with the code**/
    std::string left_right;

	/** Entire trajectory (final pose and waypoint) for the right hand**/
    std::deque<yarp::sig::Vector> trajectory_right;
	/** Entire trajectory (final pose and waypoint) for the left hand**/
    std::deque<yarp::sig::Vector> trajectory_left;
	/** Robot hand pose computed by the solver for the right hand**/
    yarp::sig::Vector poseR;
	/** Hand ellipsoid pose computed by the solver for the right hand**/
	yarp::sig::Vector solR;
	/** Robot hand pose computed by the solver for the left hand**/
    yarp::sig::Vector poseL;
	/** Hand ellipsoid pose computed by the solver for the left hand**/
	yarp::sig::Vector solL;

	/** Tolerance of the Ipopt optimization problem**/
    double tol;
	/** Constraint tolerance of the Ipopt optimization problem**/
	double constr_viol_tol;
	/** Maximum iteration allowed for the Ipopt optimization problem**/
    int max_iter;
	/** Acceptable iter of the Ipopt optimization problem**/
	int acceptable_iter;
	int object_provided;
	/** Mu strategy of the Ipopt optimization problem**/
    std::string mu_strategy;
	/** NLP scaling method of the Ipopt optimization problem**/
	std::string nlp_scaling_method;
	/** Max cpu time allowed for the Ipopt optimization problem**/
    double max_cpu_time;

	/** Number of points sampled on the hand ellipsoid for the Ipopt optimization problem**/
    int n_pointshand;
	/** Distance for shifting the waypoint along x axis of the hand reference frame**/
    double distance;
	/** Distance for shifting the waypoint along z axis of the hand reference frame**/
	double distance1;
	/** Direction for generating the waypoint for the approach: it could be on x and z axes ("xz") or only z axis ("z") of the hand reference frame.**/
    std::string dir;
	/** Distance of the robot pose with respect to the hand ellipsoid along x axis of the hand reference frame**/
    yarp::sig::Vector displacement;
	/** Parameters of the implicit function describing the plane on which the object is located in the root reference frame**/
    yarp::sig::Vector plane;

	/** Parameters for the Ipopt optimization problem**/
    yarp::os::Property ipopt_par;
	/** Parameters for pose computation**/
    yarp::os::Property pose_par;
	/** Parameters for trajectory computation**/
    yarp::os::Property trajectory_par;

    bool go_on;
    double t0, t_grasp;

    yarp::os::Mutex mutex;

    yarp::os::ResourceFinder *rf;

	/** Print level for the Ipopt optimization problem**/
    int print_level;

public:
    
    /** Vector for representing one hand ellipsoid*/
    yarp::sig::Vector &hand;
    /** Vector for representing one hand ellipsoid*/
    yarp::sig::Vector &hand1;
    /** Complete solution computed */
    yarp::os::Property &complete_sol;
    /** Object superquadric */
    const yarp::sig::Vector &object;
    int count_file_old;
    int count_file;
	/** Best hand for grasping the object **/
    std::string best_hand;
	/** Final cost function value for right hand**/
    double final_value_R;
	/** Final cost function value for left hand**/
	double final_value_L;
	/** Cosing between z axes of the root and right hand reference frame**/
	double cos_zr;
	/** Cosing between z axes of the root and right hand reference frame**/
	double cos_zl;

	/** Quality of pose right**/
    double &quality_right;
	/** Quality of pose left**/
	double &quality_left;

    /*******************************************************************************/
    GraspComputation(const yarp::os::Property &_ipopt_par, const yarp::os::Property &_pose_par,
                     const yarp::os::Property &_trajectory_par, const std::string &_left_or_right,
                     yarp::sig::Vector &_hand, yarp::sig::Vector &_hand1, yarp::os::ResourceFinder *_rf,
                     yarp::os::Property &_complete_sol, const yarp::sig::Vector &_object, double &_quality_right, double &_quality_left);

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
};

#endif
