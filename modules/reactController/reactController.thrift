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
  * Starts the blinking behavior (if it was not started before).
  * @return true/false on success/failure.
  */
  bool set_xd();
}
