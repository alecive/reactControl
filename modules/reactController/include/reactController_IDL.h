// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_reactController_IDL
#define YARP_THRIFT_GENERATOR_reactController_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Vector.h>

class reactController_IDL;


/**
 * reactController_IDL
 * IDL Interface to \ref reactController services.
 */
class reactController_IDL : public yarp::os::Wire {
public:
  reactController_IDL();
  /**
   * Sets a new 3D Cartesian target
   * @param _xd Vector that specifies the new target
   *            (put it between brackets if asking for it through rpc).
   * @return true/false on success/failure.
   */
  virtual bool set_xd(const yarp::sig::Vector& _xd);
  /**
   * Sets a new 3D Cartesian target relative to the current end-effector configuration
   * @param _rel_xd Vector that specifies the new target relative to the current
   *                end-effector configuration -- e.g. (0.0 0.0 0.05) should move
   *                the end effector 5cm up
   *                (put it between brackets if asking for it through rpc).
   * @return true/false on success/failure.
   */
  virtual bool set_relative_xd(const yarp::sig::Vector& _rel_xd);
  /**
   * Sets tolerance.
   * @param _tol the solver exits if norm(x_d-x)<tol.
   * @return true/false on success/failure.
   */
  virtual bool set_tol(const double _tol);
  /**
   * Gets the tolerance.
   * @return the current tolerance value.
   */
  virtual double get_tol();
  /**
   * Sets the max velocity at the joints.
   * @param _v_max the max velocity to be set.
   * @return true/false on success/failure.
   */
  virtual bool set_v_max(const double _v_max);
  /**
   * Gets the max velocity.
   * @return the max velocity at the joints.
   */
  virtual double get_v_max();
  /**
   * Sets Trajectory Speed.
   * @param _traj_speed  the speed of the trajectory
   * @return true/false on success/failure.
   */
  virtual bool set_traj_speed(const double _traj_speed);
  /**
   * Sets verbosity.
   * @param _verbosity  the verbosity of the controller
   * @return true/false on success/failure.
   */
  virtual bool set_verbosity(const int32_t _verbosity);
  /**
   * Gets verbosity.
   * @return the verbosity of the controller
   */
  virtual int32_t get_verbosity();
  /**
   * Setups a new particle with a given initial position and constant velocity
   * @param _x_0_vel 6D Vector that specifies the new initial position and the
   *                 velocity. It has not been splitted into two separate vectors
   *                 because to my knowledge it is not possible
   *                 (put it between brackets if asking for it through rpc).
   * @return true/false on success/failure.
   */
  virtual bool setup_new_particle(const yarp::sig::Vector& _x_0_vel);
  /**
   * Stops the particle motion, sets the output vector to the given value.
   * @param _x_0     3D Vector that specifies the new value of the output vector.
   *                 (put it between brackets if asking for it through rpc).
   * @return true/false on success/failure.
   */
  virtual bool reset_particle(const yarp::sig::Vector& _x_0);
  /**
   * Stops the particle motion at the current state.
   * @return true/false on success/failure.
   */
  virtual bool stop_particle();
  /**
   * Gets the particle state.
   * @return the particle 3D position.
   */
  virtual yarp::sig::Vector get_particle();
  /**
   * Enables the torso. WARNING: if this command is sent while the robot
   * is performing a reaching, the flag will not be enabled. You have to
   * wait for the robot to stop (or stop it manually).
   * @return true/false on success/failure.
   */
  virtual bool enable_torso();
  /**
   * Disables the torso WARNING: if this command is sent while the robot
   * is performing a reaching, the flag will not be enabled. You have to
   * wait for the robot to stop (or stop it manually).
   * @return true/false on success/failure.
   */
  virtual bool disable_torso();
  /**
   * Disables the controller
   * @return true/false on success/failure.
   */
  virtual bool stop();
  /**
   * Gets the state of the reactCtrlThread.
   * @return an integer that represent the state of the controller. As of now,
   *         it can be one of the following:
   *         STATE_WAIT  (0) -> wait for a new command
   *         STATE_REACH (1) -> a reaching is being performed
   *         STATE_IDLE  (2) -> idle state, it falls back automatically to STATE_WAIT
   */
  virtual int32_t get_state();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

