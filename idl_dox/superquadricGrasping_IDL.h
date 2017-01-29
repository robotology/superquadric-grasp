// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_superquadricGrasping_IDL
#define YARP_THRIFT_GENERATOR_superquadricGrasping_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

class superquadricGrasping_IDL;


class superquadricGrasping_IDL : public yarp::os::Wire {
public:
  superquadricGrasping_IDL();
  /**
   *  Start entire pipeline: pose computation and
   *  grasping
   * @return true/false on success/failure
   */
  virtual bool start();
  /**
   *  Come back home
   * @return true/false on success/failure
   */
  virtual bool go_home(const std::string& hand);
  /**
   *  Say if you want the robot to lift the object or not
   * @param yes or no
   * @return true/false on success/failure
   */
  virtual bool lift_object(const std::string& lift_or_not);
  /**
   *  Say if the robot is going to lift the object or not
   * @return yes or no
   */
  virtual std::string get_lift_object();
  /**
   *  Remove computed poses
   * @return true/false on success/failure
   */
  virtual bool clear_poses();
  /**
   *  Select the kind of grasping you want the robot to perform:
   * @param power or precision, for respectively power or precision grasp
   * @return true/false on success/failure
   */
  virtual bool grasping_method(const std::string& lift_or_not);
  /**
   *  Say the kind of selected grasping:
   * @return power or precision
   */
  virtual std::string get_grasping_method();
  /**
   *  Say if enabled depth2kin calibration
   * @return yes or no
   */
  virtual std::string get_calibrate_cam();
  /**
   *  Enable or not depth2kin calibration
   * @param yes or no
   * @return true/false on success/failure
   */
  virtual bool calibrate_cam(const std::string& calib_or_not);
  /**
   *  Choose the hand to use to grasp the object
   * @param hand name (left or right)
   * @return true/false on success/failure
   */
  virtual bool choose_hand(const std::string& hand);
  /**
   *  Get the chosen hand
   * @return left or right
   */
  virtual std::string get_chosen_hand();
  /**
   *  Choose the distance on x axis for approach
   * @param the distance
   * @return true/false on success/failure
   */
  virtual bool trajectory_distance_x(const double dis);
  /**
   *  Choose the distance on z axis for approach
   * @param the distance
   * @return true/false on success/failure
   */
  virtual bool trajectory_distance_z(const double dis);
  /**
   *  Get the distance on x axis for approach
   * @return the distance value
   */
  virtual double get_trajectory_distance_x();
  /**
   *  Get the distance on z axis for approach
   * @return true/false on success/failure
   */
  virtual double get_trajectory_distance_z();
  /**
   *  Change hand displacement for grasping
   * @param displacement value (as a Vector)
   * @return true/false on success/failure
   */
  virtual bool hand_displacement(const yarp::sig::Vector& hand);
  /**
   *  Get hand displacement for grasping
   * @return the vector of displacement
   */
  virtual std::vector<double>  get_hand_displacement();
  /**
   *  Change pose shift for grasping
   * @param shift value (as a Vector)
   * @return true/false on success/failure
   */
  virtual bool set_shift(const yarp::sig::Vector& shift);
  /**
   *  Get shift displacement for grasping
   * @return the vector of shift
   */
  virtual std::vector<double>  get_shift();
  /**
   *  Stop all robot movements but not the module
   * @return true/false on success/failure
   */
  virtual bool stop();
  /**
   *  Compute, show and send pose
   * @return computed poses
   */
  virtual std::vector<double>  compute_pose();
  /**
   * Set parameters of trajectory computation and
   *  poses reaching
   * @params a Property object containing the parameters you want to change
   * @return true/false on success/failure
   */
  virtual bool set_trajectory_options(const yarp::os::Property& options);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
