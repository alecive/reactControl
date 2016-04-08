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
  * Sets a moving target along a circular trajectory in the y and z axes, relative to the current end-effector position
  * @param _radius  Radius of the circle in meters.
  * @param _frequency Frequency according to the formula 
  *      xd[1]+=_radius*cos(2.0*M_PI*frequency*t);
  *      xd[2]+=_radius*sin(2.0*M_PI*frequency*t);
  *
  * @return true/false on success/failure.
  */
  bool set_relative_circular_xd(1:double _radius, 2: double _frequency)
  
  
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
  * Stops the particle motion, sets the output vector to the given value.
  * @param _x_0     3D Vector that specifies the new value of the output vector.
  *                 (put it between brackets if asking for it through rpc).
  * @return true/false on success/failure.
  **/
  bool reset_particle(1:Vector _x_0);

  /**
  * Stops the particle motion at the current state.
  * @return true/false on success/failure.
  **/
  bool stop_particle();

  /**
  * Gets the particle state. 
  * @return the particle 3D position.
  **/
  Vector get_particle();

  /**
  * Enables the torso. WARNING: if this command is sent while the robot 
  * is performing a reaching, the flag will not be enabled. You have to 
  * wait for the robot to stop (or stop it manually).
  * @return true/false on success/failure.
  **/
  bool enable_torso();

  /**
  * Disables the torso WARNING: if this command is sent while the robot 
  * is performing a reaching, the flag will not be enabled. You have to 
  * wait for the robot to stop (or stop it manually).
  * @return true/false on success/failure.
  **/
  bool disable_torso();

   
  /**
  * Disables the controller and switches to position mode
  * @return true/false on success/failure.
  **/
  bool stop();

  /**
  * Gets the state of the reactCtrlThread. 
  * @return an integer that represent the state of the controller. As of now,
  *         it can be one of the following:
  *         STATE_WAIT  (0) -> wait for a new command
  *         STATE_REACH (1) -> a reaching is being performed
  *         STATE_IDLE  (2) -> idle state, it falls back automatically to STATE_WAIT
  **/
  i32 get_state();
}
