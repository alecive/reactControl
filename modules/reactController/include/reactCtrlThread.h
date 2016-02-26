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

#ifndef __REACTCONTROLLERTHREAD_H__
#define __REACTCONTROLLERTHREAD_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/filters.h>

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <deque>

#include "reactIpOpt.h"
#include "particleThread.h"
#include "avoidanceHandler.h"

using namespace yarp::dev;

using namespace std;

class reactCtrlThread: public yarp::os::RateThread
{
    
public:
    // CONSTRUCTOR
    reactCtrlThread(int , const string & , const string & , const string &_ ,
                    int , bool , string , double , double , double , double , string , bool , bool , bool , bool , bool , double , bool , bool , bool , particleThread * );
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    // Enables the torso
    bool enableTorso();

    // Disables the torso
    bool disableTorso();

    // Sets the new target
    bool setNewTarget(const yarp::sig::Vector& _x_d, const bool _movingCircle);

    // Sets the new target relative to the current position
    bool setNewRelativeTarget(const yarp::sig::Vector&);

    // Sets a moving target along a circular trajectory in the y and z axes, relative to the current end-effector position
    bool setNewCircularTarget(const double _radius,const double _frequency);

    // Sets the tolerance
    bool setTol(const double );

    // Gets the tolerance
    double getTol();

    // Sets the vMax
    bool setVMax(const double );

    // Gets the vMax
    double getVMax();

    // [DEPRECATED] Sets the trajectory time 
    bool setTrajTime(const double );

    // Sets the trajectory speed
    bool setTrajSpeed(const double );

    // Sets the verbosity
    bool setVerbosity(const int );

    // gets the verbosity
    int getVerbosity() { return verbosity; };

    // gets the state of the controller
    int getState() { return state; };

    /**
    * Stops the control of the robot
    */
    bool stopControl();    
    
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;
    // Which arm to use: either left_arm or right_arm
    string part;
    // Which arm to use (short version): either left or right
    string part_short;
    // Flag to know if the torso shall be used or not
    bool useTorso;
    // robot will be commanded in velocity or in positionDirect
    string controlMode;
    // Trajectory speed (default 0.1 m/s)
    double trajSpeed;
    // Tolerance of the ipopt task. The solver exits if norm2(x_d-x)<tol.
    double tol;
    // Global tolerance of the task. The controller exits if norm(x_d-x)<globalTol
    double globalTol;
    // Max velocity set for the joints
    double vMax;
    string referenceGen; // either "uniformParticle" - constant velocity with particleThread - or "minJerk"
    bool ipOptMemoryOn; // whether ipopt should account for the real motor model
    bool ipOptFilterOn; // whether ipopt should account for the real motor model
    bool boundSmoothnessFlag; //for ipopt - whether changes in velocity commands need to be smooth
    double boundSmoothnessValue; //actual allowed change in every joint velocity commands in deg/s from one time step to the next. Note: this is not adapted to the thread rate set by the rctCtrlRate param
    bool tactileCollisionPointsOn; //if on, will be reading collision points from /skinEventsAggregator/skin_events_aggreg:o
    bool visualCollisionPointsOn; //if on, will be reading predicted collision points from visuoTactileRF/pps_activations_aggreg:o
    bool visualizeTargetInSim;  // will use the yarp rpc /icubSim/world to visualize the target
    // will use the yarp rpc /icubSim/world to visualize the particle (trajectory - intermediate targets)
    bool visualizeParticleInSim; 
    // will use the yarp rpc /icubSim/world to visualize the potential collision points
    bool visualizeCollisionPointsInSim;
    //to enable/disable the smooth changes of joint velocities bounds in optimizer
    
    /***************************************************************************/
    // INTERNAL VARIABLES:
    double dT;  //period of the thread in seconds  =getRate()/1000.0;

    particleThread  *prtclThrd;     // Pointer to the particleThread in order to access its data - if referenceGen is "uniformParticle"
    iCub::ctrl::minJerkRefGen *refGenMinJerk; //If referenceGen is "minJerk"
    iCub::ctrl::Integrator *I; //if controlMode == positionDirect, we need to integrate the velocity control commands

    int        state;        // Flag to know in which state the thread is in
    
    // Driver for "classical" interfaces
    PolyDriver       ddA;
    PolyDriver       ddT;

    // "Classical" interfaces for the arm
    IEncoders            *iencsA;
    IVelocityControl2     *ivelA;
    IPositionDirect       *iposDirA;
    IControlMode2         *imodA;
    IControlLimits        *ilimA;
    yarp::sig::Vector     *encsA;
    iCub::iKin::iCubArm     *arm;
    int jntsA;

    // "Classical" interfaces for the torso
    IEncoders            *iencsT;
    IVelocityControl2     *ivelT;
    IPositionDirect       *iposDirT;
    IControlMode2         *imodT;
    IControlLimits        *ilimT;
    yarp::sig::Vector     *encsT;
    int jntsT;
    
    size_t chainActiveDOF;
    
    yarp::sig::Vector x_0;  // Initial end-effector position
    yarp::sig::Vector x_t;  // Current end-effector position
    yarp::sig::Vector x_n;  // Desired next end-effector position
    yarp::sig::Vector x_d;  // Vector that stores the new target

    bool movingTargetCircle;
    double radius;
    double frequency;
    yarp::sig::Vector circleCenter;

    //N.B. All angles in this thread are in degrees
    yarp::sig::Vector qA; //current values of arm joints (should be 7)
    yarp::sig::Vector qT; //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)
    yarp::sig::Vector q; //current joint angle values (10 if torso is on, 7 if off)
    yarp::sig::Vector qIntegrated; //joint angle values integrated from velocity commands by Integrator - controlMode positionDirect only

    yarp::sig::Vector q_dot;  // Computed joint velocities to reach the target
    
    yarp::sig::Matrix lim;  //matrix with joint position limits for the current chain
    yarp::sig::Matrix vLimNominal;     //matrix with min/max velocity limits for the current chain
    yarp::sig::Matrix vLimAdapted;  //matrix with min/max velocity limits after adptation by avoidanceHandler
      
    yarp::sig::Matrix H;      // End-effector pose
      
    // ports and files
    yarp::os::BufferedPort<yarp::os::Bottle> aggregSkinEventsInPort; //coming from /skinEventsAggregator/skin_events_aggreg:o
    yarp::os::BufferedPort<yarp::os::Bottle> aggregPPSeventsInPort; //coming from visuoTactileRF/pps_activations_aggreg:o 
    //expected format for both: (skinPart_s x y z o1 o2 o3 magnitude), with position x,y,z and normal o1 o2 o3 in link FoR
    yarp::os::Port outPort;
    yarp::os::Port outPortiCubGui;
    yarp::os::Port portToSimWorld;
    ofstream fout_param; //log parameters that stay constant during the simulation, but are important for analysis - e.g. joint limits 
    // Stamp for the setEnvelope for the ports
    yarp::os::Stamp ts;
 
    // IPOPT STUFF
    reactIpOpt    *slv;    // solver
    int ipoptExitCode;
    double timeToSolveProblem_s; //time taken by q_dot = solveIK(ipoptExitCode) ~ ipopt + avoidance handler
    std::deque<yarp::sig::Vector> memoryVelCommands_RAD; //buffer to store past velocities- this one is expceptionally in RAD - will be passed to ipopt
    double motorModel_kp;
    double motorModel_td;
    iCub::ctrl::Filter *filter;
    double filterTc;

    // Mutex for handling things correctly
    yarp::os::Mutex mutex;
    
    yarp::os::Bottle    cmd; 
    yarp::sig::Matrix T_world_root; //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR
    
    bool visualizeIniCubGui;
    bool visualizeParticleIniCubGui;
    bool visualizeTargetIniCubGui;
    
    // objects in simulator will be created only for first target - with new targets they will be moved
    bool firstTarget;
    std::vector<collisionPoint_t> collisionPoints; //list of "avoidance vectors" from peripersonal space / safety margin
    int collisionPointsVisualizedCount; //objects will be created in simulator and then their positions updated every iteration
    yarp::sig::Vector collisionPointsSimReservoirPos; //inactive collision points will be stored in the world
        
    /**
    * Solves the Inverse Kinematic task
    */
    yarp::sig::Vector solveIK(int &);

    /**** kinematic chain, control, ..... *****************************/
    
    /**
    * Aligns joint bounds according to the actual limits of the robot
    */
    bool alignJointsBounds();

    /**
    * Prints the joints bounds from the iCubArm
    */
    void printJointsBounds();

    /**
    * Updates the arm's kinematic chain with the encoders from the robot
    **/
    void updateArmChain();

    /**
    * Sends the computed velocities or positions to the robot, depending on controlMode
    */
    bool controlArm(const string controlMode,const yarp::sig::Vector &);

    /**
     * Check the state of each joint to be controlled
     * @param  jointsToSet vector of integers that defines the joints to be set
     * @param  _p part to set. It can be either "torso" or "arm"
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return             true/false if success/failure
     */
    bool areJointsHealthyAndSet(yarp::sig::VectorOf<int> &jointsToSet,
                                const string &_p, const string &_s);

    /**
     * Changes the control modes of the torso to either position or velocity
     * @param  _p part to set. It can be either "torso" or "arm"
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return    true/false if success/failure
     */
    bool setCtrlModes(const yarp::sig::VectorOf<int> &jointsToSet,
                      const string &_p, const string &_s);


    bool stopControlHelper();

    /***************** auxiliary computations  *******************************/

    /**
    * Computes the next target on a circular trajectory, using global vars frequency, radius, and circleCenter
    * @return new target position
    **/
    yarp::sig::Vector  getPosMovingTargetOnCircle();

    /**
    * Computes the spatial step that will be performed by the robot
    **/
    yarp::sig::Vector computeDeltaX();


    void convertPosFromRootToSimFoR(const yarp::sig::Vector &pos, yarp::sig::Vector &outPos);
    
    void convertPosFromLinkToRootFoR(const yarp::sig::Vector &pos,const iCub::skinDynLib::SkinPart skinPart, yarp::sig::Vector &outPos);
        

   /************************** communication through ports in/out ***********************************/

    bool getCollisionPointsFromPort(yarp::os::BufferedPort<yarp::os::Bottle> &inPort, double gain, string whichChain,std::vector<collisionPoint_t> &collPoints);

    /**
    * Sends useful data to a port in order to track it on matlab
    **/
    void sendData();


    /***************************** visualizations in icubGui  ****************************/
    //uses corresponding global variables for target pos (x_d) or particle pos (x_n) and creates bottles for the port to iCubGui
    void sendiCubGuiObject(const std::string object_type);
    
    void deleteiCubGuiObject(const std::string object_type);
    
    /****************** visualizations in icub simulator   *************************************/
    /**
    * Creates a sphere (not affected by gravity) in the iCub simulator through the /icubSim/world port
    * @param radius
    * @param pos  
    */
    void createStaticSphere(double radius, const yarp::sig::Vector &pos);
   
    void moveSphere(int index, const yarp::sig::Vector &pos);
    
    void createStaticBox(const yarp::sig::Vector &pos);
    
    void moveBox(int index, const yarp::sig::Vector &pos);
    
    void showCollisionPointsInSim();


    /**
    * Prints a message according to the verbosity level:
    * @param l will be checked against the global var verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;


};

#endif

