// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_superquadricGrasp_IDL
#define YARP_THRIFT_GENERATOR_superquadricGrasp_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Property.h>

class superquadricGrasp_IDL;


/**
 * superquadricGrasp_IDL
 * IDL Interface to \ref superquadric-grasp services.
 */
class superquadricGrasp_IDL : public yarp::os::Wire {
public:
  superquadricGrasp_IDL();
  /**
   *  Remove computed poses.
   * @return true/false on success/failure.
   */
  virtual bool clear_poses();
  /**
   *  Choose the hand to use to grasp the object.
   * @param hand name (left or right).
   * @return true/false on success/failure.
   */
  virtual bool set_hand(const std::string& hand);
  /**
   *  Get the chosen hand.
   * @return left or right.
   */
  virtual std::string get_hand();
  /**
   *  Set if to save or not the computed poses
   *  and trajectory.
   * @param entry can be "on" or "off".
   * @return true/false on success/failure.
   */
  virtual bool set_save_poses(const std::string& entry);
  /**
   *  Get if the saving process is on or off.
   * @return "on" or "off".
   */
  virtual std::string get_save_poses();
  /**
   * Set the  parameters of the module. The user
   * must pay attention in changing them.
   * @param options is a Property containing the
   * parameters the user want to change.
   * @param field is a string specifying which
   * can of parameter we are going to change.
   * Field can be: "pose", "trajectory", "optimization",
   * "visualization" or "execution".
   * You can set the  parameters typing, for instance:
   * command:  set_options ((n_pointshand <points-value>)
   * (hand_displacement_x <displacement-value>)) pose.
   * @return true/false on success/failure.
   */
  virtual bool set_options(const yarp::os::Property& options, const std::string& field);
  /**
   * Get the  parameters of the module. The user must
   * pay attention in changing them.
   * @param field can be "pose", "trajectory",
   * "optimization", "statistics", "visualization" or "execution".
   * depending on which parameters we are interested in.
   * @return the Property including all the  parameter values.
   */
  virtual yarp::os::Property get_options(const std::string& field);
  /**
   * Return the estimated grasping poses given
   *  an estimated superquadric.
   * @param estimated_superq is a Property containing
   *  the superquadric.
   * @param hand is the hand for which we want
   *  to solve the grasping problem (right, left or both).
   * @return a property containing the solution.
   *  Note: the estimated superquadric must be
   *  provide in the following format:  (dimensions (x0 x1 x2))
   *  (exponents (x3 x4)) (center (x5 x6 x7)) (orientation (x8 x9 x10 x11))
   *  where x0, x1,x2 are the semi axes of the superquadric,
   *  x3, x4 are the responsible for the shape, x5 x6 x7 are the coordinates
   *  of the superquadric center and x8 x9 x10 x11
   *  are the axis-angle representation of the superquadric orientation.
   *  The solution is given in the form: (pose_right (h0 h1 h2 h3 h4 h5 h6))
   *  (trajectory_right (t0 t1 t2 t3 t4 t5) ... ) for the right hand,
   *  and the same for the left hand (according to the  value of the
   *  string hand are input parameter. The quantity "pose_right" is the pose
   *  computed for the robot hand (x0,x1,x2,  are the 3D coordinates of the end-effector
   *  and x3,x4,x5 are the Euler angles representing the end-effector orientation)
   *  The quantity "trajectory_right"  includes all the waypoint of the
   *  computed trajectory, in the form center of the end-effector
   *  (t0,t1,t2)+ orientation (Euler angles, t3,t4,t5).
   */
  virtual yarp::os::Property get_grasping_pose(const yarp::os::Property& estimated_superq, const std::string& hand);
  /**
   *  Set if the visualization has to be enabled.
   * @return  true/false on success/failure.
   */
  virtual bool set_visualization(const std::string& e);
  /**
   *  Get if visualization is enabled.
   * @return "on" or "off".
   */
  virtual std::string get_visualization();
  /**
   *  Move the right or the left arm (according to the
   *  string e).
   * @return "on" or "off" if e is right or left.
   */
  virtual bool move(const std::string& e);
  virtual bool look_center();
  virtual bool look_obj();
  /**
   *  Move the right or the left arm  back to home position (according to the
   *  string e).
   * @return "on" or "off" if e is right or left.
   */
  virtual bool go_home(const std::string& e);
  /**
   *  Move the right or the left arm  to the basket (according to the
   *  string e).
   * @return "on" or "off" if e is right or left.
   */
  virtual bool go_to_basket(const std::string& e);
  /**
   *  Get the name of the best hand for grasping the object
   * @return right or left
   */
  virtual std::string get_best_hand();
  /**
   *  Check if the motion has been completed
   * @return true/false on success/failure
   */
  virtual bool check_motion();
  /**
   *  Check if the motion back to home has been completed
   * @return true/false on success/failure
   */
  virtual bool check_home();
  /**
   * Calibrate plane height via superquadric computation
   * @return true/false on success/failure.
   */
  virtual bool calibrate();
  virtual bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
