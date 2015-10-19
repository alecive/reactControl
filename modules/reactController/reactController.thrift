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
  * Sets Trajectory Time.
  * @param _traj_time  the time within which the solver has to solve the global task
  * @return true/false on success/failure.
  */
  bool set_traj_time(1:double _traj_time);
}
