// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <reactController_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class reactController_IDL_set_xd : public yarp::os::Portable {
public:
  yarp::sig::Vector _xd;
  bool _return;
  void init(const yarp::sig::Vector& _xd);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class reactController_IDL_set_relative_xd : public yarp::os::Portable {
public:
  yarp::sig::Vector _rel_xd;
  bool _return;
  void init(const yarp::sig::Vector& _rel_xd);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class reactController_IDL_set_tol : public yarp::os::Portable {
public:
  double _tol;
  bool _return;
  void init(const double _tol);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class reactController_IDL_set_traj_time : public yarp::os::Portable {
public:
  double _traj_time;
  bool _return;
  void init(const double _traj_time);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class reactController_IDL_set_verbosity : public yarp::os::Portable {
public:
  int16_t _verbosity;
  bool _return;
  void init(const int16_t _verbosity);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool reactController_IDL_set_xd::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_xd",1,2)) return false;
  if (!writer.write(_xd)) return false;
  return true;
}

bool reactController_IDL_set_xd::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void reactController_IDL_set_xd::init(const yarp::sig::Vector& _xd) {
  _return = false;
  this->_xd = _xd;
}

bool reactController_IDL_set_relative_xd::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_relative_xd",1,3)) return false;
  if (!writer.write(_rel_xd)) return false;
  return true;
}

bool reactController_IDL_set_relative_xd::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void reactController_IDL_set_relative_xd::init(const yarp::sig::Vector& _rel_xd) {
  _return = false;
  this->_rel_xd = _rel_xd;
}

bool reactController_IDL_set_tol::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_tol",1,2)) return false;
  if (!writer.writeDouble(_tol)) return false;
  return true;
}

bool reactController_IDL_set_tol::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void reactController_IDL_set_tol::init(const double _tol) {
  _return = false;
  this->_tol = _tol;
}

bool reactController_IDL_set_traj_time::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_traj_time",1,3)) return false;
  if (!writer.writeDouble(_traj_time)) return false;
  return true;
}

bool reactController_IDL_set_traj_time::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void reactController_IDL_set_traj_time::init(const double _traj_time) {
  _return = false;
  this->_traj_time = _traj_time;
}

bool reactController_IDL_set_verbosity::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_verbosity",1,2)) return false;
  if (!writer.writeI16(_verbosity)) return false;
  return true;
}

bool reactController_IDL_set_verbosity::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void reactController_IDL_set_verbosity::init(const int16_t _verbosity) {
  _return = false;
  this->_verbosity = _verbosity;
}

reactController_IDL::reactController_IDL() {
  yarp().setOwner(*this);
}
bool reactController_IDL::set_xd(const yarp::sig::Vector& _xd) {
  bool _return = false;
  reactController_IDL_set_xd helper;
  helper.init(_xd);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool reactController_IDL::set_xd(const yarp::sig::Vector& _xd)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool reactController_IDL::set_relative_xd(const yarp::sig::Vector& _rel_xd) {
  bool _return = false;
  reactController_IDL_set_relative_xd helper;
  helper.init(_rel_xd);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool reactController_IDL::set_relative_xd(const yarp::sig::Vector& _rel_xd)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool reactController_IDL::set_tol(const double _tol) {
  bool _return = false;
  reactController_IDL_set_tol helper;
  helper.init(_tol);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool reactController_IDL::set_tol(const double _tol)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool reactController_IDL::set_traj_time(const double _traj_time) {
  bool _return = false;
  reactController_IDL_set_traj_time helper;
  helper.init(_traj_time);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool reactController_IDL::set_traj_time(const double _traj_time)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool reactController_IDL::set_verbosity(const int16_t _verbosity) {
  bool _return = false;
  reactController_IDL_set_verbosity helper;
  helper.init(_verbosity);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool reactController_IDL::set_verbosity(const int16_t _verbosity)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool reactController_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "set_xd") {
      yarp::sig::Vector _xd;
      if (!reader.read(_xd)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_xd(_xd);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_relative_xd") {
      yarp::sig::Vector _rel_xd;
      if (!reader.read(_rel_xd)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_relative_xd(_rel_xd);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_tol") {
      double _tol;
      if (!reader.readDouble(_tol)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_tol(_tol);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_traj_time") {
      double _traj_time;
      if (!reader.readDouble(_traj_time)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_traj_time(_traj_time);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_verbosity") {
      int16_t _verbosity;
      if (!reader.readI16(_verbosity)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_verbosity(_verbosity);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> reactController_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("set_xd");
    helpString.push_back("set_relative_xd");
    helpString.push_back("set_tol");
    helpString.push_back("set_traj_time");
    helpString.push_back("set_verbosity");
    helpString.push_back("help");
  }
  else {
    if (functionName=="set_xd") {
      helpString.push_back("bool set_xd(const yarp::sig::Vector& _xd) ");
      helpString.push_back("Sets a new 3D Cartesian target ");
      helpString.push_back("@param _xd Vector that specifies the new target ");
      helpString.push_back("           (put it between brackets if asking for it through rpc). ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_relative_xd") {
      helpString.push_back("bool set_relative_xd(const yarp::sig::Vector& _rel_xd) ");
      helpString.push_back("Sets a new 3D Cartesian target relative to the current end-effector configuration ");
      helpString.push_back("@param _rel_xd Vector that specifies the new target relative to the current ");
      helpString.push_back("               end-effector configuration -- e.g. (0.0 0.0 0.05) should move ");
      helpString.push_back("               the end effector 5cm up ");
      helpString.push_back("               (put it between brackets if asking for it through rpc). ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_tol") {
      helpString.push_back("bool set_tol(const double _tol) ");
      helpString.push_back("Sets tolerance. ");
      helpString.push_back("@param _tol the solver exits if norm(x_d-x)<tol. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_traj_time") {
      helpString.push_back("bool set_traj_time(const double _traj_time) ");
      helpString.push_back("Sets Trajectory Time. ");
      helpString.push_back("@param _traj_time  the time within which the solver has to solve the global task ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_verbosity") {
      helpString.push_back("bool set_verbosity(const int16_t _verbosity) ");
      helpString.push_back("Sets verbosity. ");
      helpString.push_back("@param _verbosity  the verbosity of the controller ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


