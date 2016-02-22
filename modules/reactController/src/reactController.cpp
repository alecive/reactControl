/* 
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone <alessandro.roncone@iit.it>
 * website: www.robotcub.org
 * author website: http://alecive.github.io
 * 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
*/

/**
\defgroup reactController reactController

A module able to do stuff.

Date first release: 30/10/2015

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
None for now.

\section lib_sec Libraries 
None for now.

\section parameters_sec Parameters

--context    \e path
- Where to find the called resource.

--from       \e from
- The name of the .ini file with the configuration parameters.

--name       \e name
- The name of the module (default reactController).

--robot      \e rob
- The name of the robot (either "icub" or "icubSim"). Default icubSim.

--rate       \e rate
- The period used by the thread. Default 100ms.

--verbosity  \e verb
- Verbosity level (default 0). The higher is the verbosity, the more
  information is printed out.

\author: Alessandro Roncone
*/ 

#include <yarp/os/Log.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/math/Math.h>
 
#include <iostream>
#include <string.h> 

#include "reactCtrlThread.h"
#include "particleThread.h"
#include "reactController_IDL.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::math;

using namespace std;

/**
* \ingroup reactController
*
*  
*/
class reactController: public RFModule, public reactController_IDL
{
private:
    reactCtrlThread *rctCtrlThrd;
    particleThread    *prtclThrd;
    RpcServer            rpcSrvr;

    string robot;       // Name of the robot
    string  name;       // Name of the module
    string  part;       // Part to use

    int     verbosity;  // Verbosity level
    int   rctCtrlRate;  // rate of the reactCtrlThread
    int     prtclRate;  // rate of the particleThread

    bool disableTorso;  // flag to know if the torso has to be used or not

    string controlMode; //either "velocity" (original) or "positionDirect" (new option after problems with oscillations)
    
    double  trajSpeed;  // trajectory speed
    double        tol;  // Tolerance of the ipopt task. The solver exits if norm2(x_d-x)<tol.
    double  globalTol;  // global tolerance of the task. The controller exits if norm(x_d-x)<globalTol
    double       vMax;  // max velocity set for the joints
    
    string referenceGen; // either "uniformParticle" - constant velocity with particleThread - or "minJerk" 
    bool ipOptMemoryOn; // whether ipopt should account for the real motor model
    
    bool tactileCollisionPointsOn; //if on, will be reading collision points from /skinEventsAggregator/skin_events_aggreg:o
    bool visualCollisionPointsOn; //if on, will be reading predicted collision points from visuoTactileRF/pps_activations_aggreg:o
    
    bool boundSmoothnessFlag; //for ipopt - whether changes in velocity commands need to be smooth
    double boundSmoothnessValue; //actual allowed change in every joint velocity commands in deg/s from one time step to the next. Note: this is not adapted to the thread rate set by the rctCtrlRate param
    
    bool visualizeTargetInSim; // will use the yarp rpc /icubSim/world to visualize the target
    bool visualizeParticleInSim; // will use the yarp rpc /icubSim/world to visualize the particle (trajectory - intermediate targets)
    bool visualizeCollisionPointsInSim; // will visualize the (potential) collision points in iCub simulator 

public:
    reactController()
    {
        rctCtrlThrd=0;
        prtclThrd=0;

        robot =         "icubSim";
        name  = "reactController";
        part  =        "left_arm";

        verbosity    =     0;
        rctCtrlRate  =    10;    
        prtclRate    =    10;
        disableTorso = false;
        controlMode = "velocity";        
        trajSpeed    =   0.1;
        tol          =  1e-5;
        globalTol    =  1e-2;
        vMax         =  30.0;
        
        referenceGen = "uniformParticle";
        bool ipOptMemoryOn = false;
        
        tactileCollisionPointsOn = false;
        visualCollisionPointsOn = false;
        
        boundSmoothnessFlag = false;
        boundSmoothnessValue = 30; // 30 deg/s change in a time step is huge - would have no effect 
        
        if(robot == "icubSim"){
            visualizeTargetInSim = true;
            visualizeParticleInSim = true;
            visualizeCollisionPointsInSim = true;
        }
        else{
            visualizeTargetInSim = false;
            visualizeParticleInSim = false;
            visualizeCollisionPointsInSim = false;
        }
    }

    bool set_xd(const yarp::sig::Vector& _xd)
    {
        if (_xd.size()>=3)
        {
            yInfo("");
            yInfo("[reactController] received new x_d: %s", _xd.toString(3,3).c_str());
            return rctCtrlThrd->setNewTarget(_xd, false);
        }
        return false;
    }

    bool set_relative_xd(const yarp::sig::Vector& _rel_xd)
    {
        if (_rel_xd.size()>=3)
        {
            yInfo("");
            yInfo("[reactController] received new relative x_d: %s", _rel_xd.toString(3,3).c_str());
            return rctCtrlThrd->setNewRelativeTarget(_rel_xd);
        }
        return false;
    }

    bool set_relative_circular_xd(const double _radius, const double _frequency)
    {
            yInfo("");
            yInfo("[reactController] received new relative circular x_d: radius %f, frequency: %f.",_radius,_frequency);
            if ((_radius>=0.0) && (_radius <= 0.3) && (_frequency >=0.0) && (_frequency<=1.0)  )
                return rctCtrlThrd->setNewCircularTarget(_radius,_frequency);   
            else{
                yWarning("[reactController] set_relative_circular_xd(): expecting radius <0,0.3>, frequency <0,1>");    
                return false;
            }
    }
    
    bool set_tol(const double _tol)
    {
        return rctCtrlThrd->setTol(_tol);
    }

    double get_tol()
    {
        return rctCtrlThrd->getTol();
    }

    bool set_v_max(const double _v_max)
    {
        return rctCtrlThrd->setVMax(_v_max);
    }

    double get_v_max()
    {
        return rctCtrlThrd->getVMax();
    }

    bool set_traj_speed(const double _traj_speed)
    {
        return rctCtrlThrd->setTrajSpeed(_traj_speed);
    }

    int get_verbosity()
    {
        return rctCtrlThrd->getVerbosity();
    }

    int get_state()
    {
        return rctCtrlThrd->getState();
    }

    bool set_verbosity(const int32_t _verbosity)
    {
        yInfo("[reactController] Setting verbosity to %i",_verbosity);
        return rctCtrlThrd->setVerbosity(_verbosity);
    }

    bool setup_new_particle(const yarp::sig::Vector& _x_0_vel)
    {
        if (referenceGen == "uniformParticle"){
            yarp::sig::Vector _x_0 = _x_0_vel.subVector(0,2);
            yarp::sig::Vector _vel = _x_0_vel.subVector(3,5);
            yInfo("[reactController] Setting up new particle.. x_0: %s\tvel: %s\n",
                _x_0.toString(3,3).c_str(), _vel.toString(3,3).c_str());
            return prtclThrd->setupNewParticle(_x_0,_vel);
        }
        else{
            yWarning("[reactController] to command the particle, referenceGen needs to be set to uniformParticle");
            return false;
        }
    }

    bool reset_particle(const yarp::sig::Vector& _x_0)
    {
        if (referenceGen == "uniformParticle"){
            if (_x_0.size()<3)
            {
                return false;
            }
            yInfo("[reactController] Resetting particle to %s..",_x_0.toString(3,3).c_str());
            return prtclThrd->resetParticle(_x_0);
        }
        else{
            yWarning("[reactController] to command the particle, referenceGen needs to be set to uniformParticle");
            return false;
        }
    }

    bool stop_particle()
    {
        if (referenceGen == "uniformParticle"){
            yInfo("[reactController] Stopping particle..");
            return prtclThrd->stopParticle();
        }
        else{
            yWarning("[reactController] to command the particle, referenceGen needs to be set to uniformParticle");
            return false;
        }
    }

    yarp::sig::Vector get_particle()
    {
        if (referenceGen == "uniformParticle"){
            yInfo("[reactController] Getting particle..");
            return prtclThrd->getParticle();
        }
        else{
            yWarning("[reactController] to command the particle, referenceGen needs to be set to uniformParticle");
            yarp::sig::Vector v(3,0.0);
            return v;
        }
    }

    bool enable_torso()
    {
        yInfo("[reactController] Enabling torso..");
        return rctCtrlThrd->enableTorso();
    }

    bool disable_torso()
    {
        yInfo("[reactController] Disabling torso..");
        return rctCtrlThrd->disableTorso();
    }

    bool stop()
    {
        yInfo("[reactController] Stopping control..");
        return rctCtrlThrd->stopControl();
    }

    bool configure(ResourceFinder &rf)
    {
        //******************************************************
        //******************* NAME ******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                yInfo("[reactController] Module name set to %s", name.c_str());
            }
            else yInfo("[reactController] Module name set to default, i.e. %s", name.c_str());
            setName(name.c_str());

        //******************* ROBOT ******************
            if (rf.check("robot"))
            {
                robot = rf.find("robot").asString();
                yInfo("[reactController] Robot is: %s", robot.c_str());
            }
            else yInfo("[reactController] Could not find robot option in the config file; using %s as default",robot.c_str());

         //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                yInfo("[reactController] verbosity set to %i", verbosity);
            }
            else yInfo("[reactController] Could not find verbosity option in the config file; using %i as default",verbosity);

        //****************** rctCtrlRate ******************
            if (rf.check("rctCtrlRate"))
            {
                rctCtrlRate = rf.find("rctCtrlRate").asInt();
                yInfo("[reactController] rctCTrlThread working at %i ms.",rctCtrlRate);
            }
            else yInfo("[reactController] Could not find rctCtrlRate in the config file; using %i as default",rctCtrlRate);

         //******************* PART ******************
            if (rf.check("part"))
            {
                part = rf.find("part").asString();
                if (part=="left")
                {
                    part="left_arm";
                }
                else if (part=="right")
                {
                    part="right_arm";
                }
                else if (part!="left_arm" && part!="right_arm")
                {
                    part="left_arm";
                    yWarning("[reactController] part was not in the admissible values. Using %s as default.",part.c_str());
                }
                yInfo("[reactController] part to use is: %s", part.c_str());
            }
            else yInfo("[reactController] Could not find part option in the config file; using %s as default",part.c_str());

        //********************** CONFIGS ***********************
            if (rf.check("disableTorso"))
            {
                if(rf.find("disableTorso").asString()=="on"){
                    disableTorso = true;
                    yInfo("[reactController] disableTorso flag set to on.");
                }
                else{
                    disableTorso = false;
                    yInfo("[reactController] disableTorso flag set to off.");
                }
            }
            else
            {
                 yInfo("[reactController] Could not find disableTorso flag (on/off) in the config file; using %d as default",disableTorso);
            }
        
        //*** we will command the robot in velocity or in positionDirect
           if (rf.check("controlMode"))
            {
                controlMode = rf.find("controlMode").asString();
                if(controlMode!="velocity" && controlMode!="positionDirect")
                {
                    controlMode="velocity";
                    yWarning("[reactController] controlMode was not in the admissible values (velocity / positionDirect). Using %s as default.",controlMode.c_str());
                }
                else 
                    yInfo("[reactController] controlMode to use is: %s", controlMode.c_str());
            }
            else yInfo("[reactController] Could not find controlMode option in the config file; using %s as default",controlMode.c_str());
        
        //****************** prtclRate ******************
            if (rf.check("prtclRate"))
            {
                prtclRate = rf.find("prtclRate").asInt();
                yInfo("[reactController] particleThread working at %i ms.",prtclRate);
            }
            else yInfo("[reactController] Could not find prtclRate in the config file; using %i as default",prtclRate);

        //****************** trajSpeed ******************
            if (rf.check("trajSpeed"))
            {
                trajSpeed = rf.find("trajSpeed").asDouble();
                yInfo("[reactController] trajSpeed set to %g s.",trajSpeed);
            }
            else yInfo("[reactController] Could not find trajSpeed in the config file; using %g as default",trajSpeed);

        //****************** vMax ******************
            if (rf.check("vMax"))
            {
                vMax = rf.find("vMax").asDouble();
                yInfo("[reactController] vMax (max joint vel) set to %g [deg/s].",vMax);
            }
            else yInfo("[reactController] Could not find vMax (max joint vel) in the config file; using %g [deg/s] as default",vMax);

        //****************** tol ******************
            if (rf.check("tol"))
            {
                tol = rf.find("tol").asDouble();
                yInfo("[reactController] tol set to %g m.",tol);
            }
            else yInfo("[reactController] Could not find tol in the config file; using %g as default",tol);

        //****************** globalTol ******************
            if (rf.check("globalTol"))
            {
                globalTol = rf.find("globalTol").asDouble();
                yInfo("[reactController] globalTol set to %g m.",globalTol);
            }
            else yInfo("[reactController] Could not find globalTol in the config file; using %g as default",globalTol);
            
         //*** generating positions for end-effector - trajectory between current pos and final target
           if (rf.check("referenceGen"))
            {
                referenceGen = rf.find("referenceGen").asString();
                if(referenceGen!="uniformParticle" && referenceGen!="minJerk")
                {
                    referenceGen="uniformParticle";
                    yWarning("[reactController] referenceGen was not in the admissible values (uniformParticle / minJerk). Using %s as default.",referenceGen.c_str());
                }
                else 
                    yInfo("[reactController] referenceGen to use is: %s", referenceGen.c_str());
            }
            else yInfo("[reactController] Could not find referenceGen option in the config file; using %s as default",referenceGen.c_str());
        
            //********************** ipopt using memory - motor model ***********************
            if (rf.check("ipOptMemoryOn"))
            {
                if(rf.find("ipOptMemoryOn").asString()=="on"){
                    ipOptMemoryOn = true;
                    yInfo("[reactController] ipOptMemoryOn flag set to on.");
                }
                else{
                    ipOptMemoryOn = false;
                    yInfo("[reactController] ipOptMemoryOn flag set to off.");
                }
            }
            else
            {
                 yInfo("[reactController] Could not find ipOptMemoryOn flag (on/off) in the config file; using %d as default",ipOptMemoryOn);
            }  
        
         //************** getting collision points either from aggregated skin events or from pps (predictions from vision)
        if (rf.check("tactileCollisionPoints"))
        {
            if(rf.find("tactileCollisionPoints").asString()=="on"){
                tactileCollisionPointsOn = true;
                yInfo("[reactController] tactileCollisionPoints flag set to on.");
            }
            else{
                tactileCollisionPointsOn = false;
                yInfo("[reactController] tactileCollisionPoints flag set to off.");
            }
        }
        else
        {
            yInfo("[reactController] Could not find tactileCollisionPoints flag (on/off) in the config file; using %d as default",tactileCollisionPointsOn);
        }
        
        if (rf.check("visualCollisionPoints"))
        {
            if(rf.find("visualCollisionPoints").asString()=="on"){
                visualCollisionPointsOn = true;
                yInfo("[reactController] visualCollisionPoints flag set to on.");
            }
            else{
                visualCollisionPointsOn = false;
                yInfo("[reactController] visualCollisionPoints flag set to off.");
            }
        }
        else
        {
            yInfo("[reactController] Could not find visualCollisionPoints flag (on/off) in the config file; using %d as default",visualCollisionPointsOn);
        }
        
        if (rf.check("boundSmoothnessFlag"))
        {
            if(rf.find("boundSmoothnessFlag").asString()=="on"){
                boundSmoothnessFlag = true;
                yInfo("[reactController] boundSmoothnessFlag flag set to on.");
                
            }
            else{
                boundSmoothnessFlag = false;
                yInfo("[reactController] boundSmoothnessFlag flag set to off.");
            }
        }
        else
        {
            yInfo("[reactController] Could not find boundSmoothnessFlag flag (on/off) in the config file; using %d as default",boundSmoothnessFlag);
        }
        if (rf.check("boundSmoothnessValue"))
        {
              boundSmoothnessValue = rf.find("boundSmoothnessValue").asDouble();
               yInfo("[reactController] boundSmoothnessValue set to %g deg/s (allowed change in joint vel in a time step).",boundSmoothnessValue);
        }
        else yInfo("[reactController] Could not find boundSmoothnessValue in the config file; using %g as default",boundSmoothnessValue);
           
         //********************** Visualizations in simulator ***********************
            if (robot == "icubSim"){
                if (rf.check("visualizeTargetInSim"))
                {
                    if(rf.find("visualizeTargetInSim").asString()=="on"){
                        visualizeTargetInSim = true;
                        yInfo("[reactController] visualizeTargetInSim flag set to on.");
                    }
                    else{
                        visualizeTargetInSim = false;
                        yInfo("[reactController] visualizeTargetInSim flag set to off.");
                    }
                }
                else
                {
                    yInfo("[reactController] Could not find visualizeTargetInSim flag (on/off) in the config file; using %d as default",visualizeTargetInSim);
                }
                
                if (rf.check("visualizeParticleInSim"))
                {
                    if(rf.find("visualizeParticleInSim").asString()=="on"){
                        visualizeParticleInSim = true;
                        yInfo("[reactController] visualizeParticleInSim flag set to on.");
                    }
                    else{
                        visualizeParticleInSim = false;
                        yInfo("[reactController] visualizeParticleInSim flag set to off.");
                    }
                }
                else
                {
                    yInfo("[reactController] Could not find visualizeParticleInSim flag (on/off) in the config file; using %d as default",visualizeParticleInSim);
                }
                if (rf.check("visualizeCollisionPointsInSim"))
                {
                    if(rf.find("visualizeCollisionPointsInSim").asString()=="on"){
                        visualizeCollisionPointsInSim = true;
                        yInfo("[reactController] visualizeCollisionPointsInSim flag set to on.");
                    }
                    else{
                        visualizeCollisionPointsInSim = false;
                        yInfo("[reactController] visualizeCollisionPointsInSim flag set to off.");
                    }
                }
                else
                {
                    yInfo("[reactController] Could not find visualizeCollisionPointsInSim flag (on/off) in the config file; using %d as default",visualizeCollisionPointsInSim);
                }
            }
            else{
                visualizeTargetInSim = false;
                yInfo("[reactController] visualizeTargetInSim flag set to off.");
                visualizeParticleInSim = false;
                yInfo("[reactController] visualizeParticleInSim flag set to off.");
                visualizeCollisionPointsInSim = false;
                yInfo("[reactController] visualizeCollisionPointsInSim flag set to off.");

            }

        //************* THREAD *************
        if(referenceGen == "uniformParticle"){
            prtclThrd = new particleThread(prtclRate, name, verbosity);
            if (!prtclThrd->start())
            {
                delete prtclThrd;
                prtclThrd=0;

                yError("[reactController] particleThread wasn't instantiated!!");
                return false;
            }
        }
        else
            prtclThrd = NULL;
            
        rctCtrlThrd = new reactCtrlThread(rctCtrlRate, name, robot, part, verbosity,
                                          disableTorso, controlMode, trajSpeed, 
                                          globalTol, vMax, tol, referenceGen, ipOptMemoryOn,
                                          tactileCollisionPointsOn,visualCollisionPointsOn,
                                          boundSmoothnessFlag,boundSmoothnessValue,
                                          visualizeTargetInSim, visualizeParticleInSim,
                                          visualizeCollisionPointsInSim, prtclThrd);
        if (!rctCtrlThrd->start())
        {
            delete rctCtrlThrd;
            rctCtrlThrd = 0;

            if (prtclThrd)
            {
                prtclThrd->stop();
                delete prtclThrd;
                prtclThrd=0;
            }

            yError("[reactController] reactCtrlThread wasn't instantiated!!");
            return false;
        }

        rpcSrvr.open(("/"+name+"/rpc:i").c_str());
        attach(rpcSrvr);

        return true;
    }

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    bool close()
    {
        yInfo("REACT CONTROLLER: Stopping threads..");
        if (rctCtrlThrd)
        {
            yInfo("REACT CONTROLLER: Stopping rctCtrlThrd...");
            rctCtrlThrd->stop();
            delete rctCtrlThrd;
            rctCtrlThrd=0;
        }
        if (prtclThrd)
        {
            yInfo("REACT CONTROLLER: Stopping prtclThrd...");
            prtclThrd->stop();
            delete prtclThrd;
            prtclThrd=0;
        }
        rpcSrvr.close();

        return true;
    }

    double getPeriod()  { return 1.0; }
    bool updateModule() { return true; }
};

/**
* Main function.
*/
int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("react-control");
    rf.setDefaultConfigFile("reactController.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {   
        yInfo(""); 
        yInfo("Options:");
        yInfo("");
        yInfo("   --context     path:  where to find the called resource");
        yInfo("   --from        from:  the name of the .ini file.");
        yInfo("   --name        name:  the name of the module (default reactController).");
        yInfo("   --robot       robot: the name of the robot. Default icubSim.");
        yInfo("   --part        part:  the arm to use. Default left_arm.");
        yInfo("   --rate        rate:  the period used by the thread. Default 100ms.");
        yInfo("   --verbosity   int:   verbosity level (default 0).");
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        yError("No Network!!!");
        return -1;
    }

    reactController rctCtrl;
    return rctCtrl.runModule(rf);
}
// empty line to make gcc happy
