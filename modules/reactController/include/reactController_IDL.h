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
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

