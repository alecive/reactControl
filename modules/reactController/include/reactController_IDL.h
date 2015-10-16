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
   * Starts the blinking behavior (if it was not started before).
   * @return true/false on success/failure.
   */
  virtual bool set_xd(const yarp::sig::Vector& _xd);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

