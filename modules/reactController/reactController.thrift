# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# reactController.thrift

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
* reactController_IDL
*
* IDL Interface to \ref reactController services.
*/
service reactController_IDL
{
  /**
  * Sets a new 3D Cartesian target
  * @param _xd Vector that specifies the new target
  *            (put it between brackets if asking for it through rpc).
  * @return true/false on success/failure.
  */
  bool set_xd(1:Vector _xd);

  /**
  * Sets a new 3D Cartesian target relative to the current end-effector configuration
  * @param _rel_xd Vector that specifies the new target relative to the current
  *                end-effector configuration -- e.g. (0.0 0.0 0.05) should move
  *                the end effector 5cm up
  *                (put it between brackets if asking for it through rpc).
  * @return true/false on success/failure.
  */
  bool set_relative_xd(1:Vector _rel_xd);

  /**
  * Sets tolerance.
  * @param _tol the solver exits if norm(x_d-x)<tol.
  * @return true/false on success/failure.
  */
  bool set_tol(1:double _tol);

  /**
  * Gets the tolerance.
  * @return the current tolerance value.
  */
  double get_tol();

  /**
  * Sets the max velocity at the joints.
  * @param _v_max the max velocity to be set.
  * @return true/false on success/failure.
  */
  bool set_v_max(1:double _v_max);

  /**
  * Gets the max velocity.
  * @return the max velocity at the joints.
  */
  double get_v_max();

  /**
  * Sets Trajectory Speed.
  * @param _traj_speed  the speed of the trajectory
  * @return true/false on success/failure.
  */
  bool set_traj_speed(1:double _traj_speed);

  /**
  * Sets verbosity.
  * @param _verbosity  the verbosity of the controller
  * @return true/false on success/failure.
  */
  bool set_verbosity(1:i32 _verbosity);  

  /**
  * Gets verbosity.
  * @return the verbosity of the controller
  */
  i32 get_verbosity();  

  /**
  * Setups a new particle with a given initial position and constant velocity
  * @param _x_0_vel 6D Vector that specifies the new initial position and the
  *                 velocity. It has not been splitted into two separate vectors
  *                 because to my knowledge it is not possible
  *                 (put it between brackets if asking for it through rpc).
  * @return true/false on success/failure.
  **/
  bool setup_new_particle(1:Vector _x_0_vel);

  /**
  * Stops the particle motion.
  * @return true/false on success/failure.
  **/
  bool stop_particle();

  /**
  * Gets the particle state.
  * @return the particle 3D position.
  **/
  Vector get_particle();

  /**
  * Enables the torso
  * @return true/false on success/failure.
  **/
  bool enable_torso();

  /**
  * Disables the torso
  * @return true/false on success/failure.
  **/
  bool disable_torso();
}
