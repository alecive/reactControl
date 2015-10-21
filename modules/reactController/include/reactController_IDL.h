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
   * Sets Trajectory Time.
   * @param _traj_time  the time within which the solver has to solve the global task
   * @return true/false on success/failure.
   */
  virtual bool set_traj_time(const double _traj_time);
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
   */
  virtual bool setup_new_particle(const yarp::sig::Vector& _x_0_vel);
  virtual yarp::sig::Vector get_particle();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

