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
    helpString.push_back("help");
  }
  else {
    if (functionName=="set_xd") {
      helpString.push_back("bool set_xd(const yarp::sig::Vector& _xd) ");
      helpString.push_back("Starts the blinking behavior (if it was not started before). ");
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


