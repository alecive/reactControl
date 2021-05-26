/* 
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Matej Hoffmann <matej.hoffmann@iit.it>, Alessandro Roncone <alessandro.roncone@yale.edu>
 * website: www.robotcub.org
 * author websites: https://sites.google.com/site/matejhof, http://alecive.github.io
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
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
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
#include <cstdio>
#include <cstdarg>
#include <vector>
#include <deque>

#include "reactIpOpt.h"
#include "particleThread.h"
#include "avoidanceHandler.h"


using namespace yarp::dev;

using namespace std;


class reactCtrlThread: public yarp::os::PeriodicThread
{
    
public:
    // CONSTRUCTOR
    reactCtrlThread(int , string  , string  , string _ ,
                    int , bool , string , double , double , double , double , string , 
                    bool , bool , bool , bool , bool, bool , bool , bool , bool , bool ,
                    particleThread *, double);
    // INIT
    bool threadInit() override;
    // RUN
    void run() override;
    // RELEASE
    void threadRelease() override;

    // Enables the torso
    bool enableTorso();

    // Disables the torso
    bool disableTorso();

    // Sets the new target
    bool setNewTarget(const yarp::sig::Vector& _x_d, bool _movingCircle);

    // Sets the new target relative to the current position
    bool setNewRelativeTarget(const yarp::sig::Vector&);

    // Sets a moving target along a circular trajectory in the y and z axes, relative to the current end-effector position
    bool setNewCircularTarget(double _radius, double _frequency);

    //Will be reading reaching targets from a port
    bool setStreamingTarget();
    
    // Sets the tolerance
    bool setTol(double );

    // Gets the tolerance
    double getTol() const;

    // Sets the vMax
    bool setVMax(double );

    // Gets the vMax
    double getVMax() const;

    // Sets the trajectory speed
    bool setTrajSpeed(double );

    // Sets the verbosity
    bool setVerbosity(int );

    // gets the verbosity
    int getVerbosity() const { return verbosity; };

    // gets the state of the controller
    int getState() const { return state; };

    /**
    * Stops the control of the robot
    */
    bool stopControl();

    bool stopControlAndSwitchToPositionMode();
    
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
    // Weight of the reaching joint rest position task (disabled if 0.0)
    double restPosWeight;
    string referenceGen; // either "uniformParticle" - constant velocity with particleThread - or "minJerk"
    bool tactileCollisionPointsOn; //if on, will be reading collision points from /skinEventsAggregator/skin_events_aggreg:o
    bool visualCollisionPointsOn; //if on, will be reading predicted collision points from visuoTactileRF/pps_activations_aggreg:o
    
    bool gazeControl; //will follow target with gaze
    bool stiffInteraction; //stiff vs. compliant interaction mode
        
    bool hittingConstraints; //inequality constraints for safety of shoudler assembly and to prevent self-collisions torso-upper arm, upper-arm - forearm
    bool orientationControl; //if orientation should be minimized as well
    bool additionalControlPoints; //if there are additional control points - Cartesian targets for others parts of the robot body - e.g. elbow
    bool visualizeTargetInSim;  // will use the yarp rpc /icubSim/world to visualize the target
    // will use the yarp rpc /icubSim/world to visualize the particle (trajectory - intermediate targets)
    bool visualizeParticleInSim; 
    // will use the yarp rpc /icubSim/world to visualize the potential collision points
    bool visualizeCollisionPointsInSim;
    //to enable/disable the smooth changes of joint velocities bounds in optimizer

  /***************************************************************************/
    // INTERNAL VARIABLES:
    double dT;  //period of the thread in seconds  =getPeriod();

    particleThread  *prtclThrd;     // Pointer to the particleThread in order to access its data - if referenceGen is "uniformParticle"
    iCub::ctrl::minJerkTrajGen *minJerkTarget; //if referenceGen is "minJerk"
    iCub::ctrl::Integrator *I; //if controlMode == positionDirect, we need to integrate the velocity control commands

    int        state;        // Flag to know in which state the thread is in
    
    // Driver for "classical" interfaces
    PolyDriver       ddA;
    PolyDriver       ddT;
    PolyDriver       ddG; // gaze  controller  driver
    
    // "Classical" interfaces for the arm
    IEncoders             *iencsA;
    IVelocityControl      *ivelA;
    IPositionDirect       *iposDirA;
    IControlMode          *imodA;
    IInteractionMode      *iintmodeA;
    IImpedanceControl     *iimpA;
    IControlLimits        *ilimA;
    yarp::sig::Vector     *encsA;
    iCub::iKin::iCubArm   *arm;
    iCub::iKin::iKinChain *armChain;
    int jntsA;
    
    vector<InteractionModeEnum> interactionModesOrig;
    vector<InteractionModeEnum> interactionModesNew;
    vector<int> jointsToSetInteractionA;
    
    // "Classical" interfaces for the torso
    IEncoders         *iencsT;
    IVelocityControl  *ivelT;
    IPositionDirect   *iposDirT;
    IControlMode      *imodT;
    IControlLimits    *ilimT;
    yarp::sig::Vector *encsT;
    int jntsT;
    
    // Gaze interface
    IGazeControl    *igaze;
    int contextGaze;
    
    size_t chainActiveDOF;
    //parallel virtual arm and chain on which ipopt will be working in the positionDirect mode case
    iCub::iKin::iCubArm    *virtualArm;
    iCub::iKin::iKinChain *virtualArmChain; 
    
    yarp::sig::Vector x_0;  // Initial end-effector position
    yarp::sig::Vector x_t;  // Current end-effector position
    yarp::sig::Vector x_n;  // Desired next end-effector position
    yarp::sig::Vector x_d;  // Vector that stores the new target

    //All orientation in Euler angle format
    yarp::sig::Vector o_0;  // Initial end-effector orientation
    yarp::sig::Vector o_t;  // Current end-effector orientation
    yarp::sig::Vector o_n;  // Desired next end-effector orientation
    yarp::sig::Vector o_d;  // Vector that stores the new orientation

    bool movingTargetCircle;
    double radius;
    double frequency;
    yarp::sig::Vector circleCenter;

    bool streamingTarget;
    yarp::os::BufferedPort<yarp::os::Bottle> streamedTargets;
    std::vector<ControlPoint> additionalControlPointsVector; 
  
    
    //N.B. All angles in this thread are in degrees
    yarp::sig::Vector qA; //current values of arm joints (should be 7)
    yarp::sig::Vector qT; //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)
    yarp::sig::Vector q; //current joint angle values (10 if torso is on, 7 if off)
    yarp::sig::Vector qIntegrated; //joint angle values integrated from velocity commands by Integrator - controlMode positionDirect only

    yarp::sig::Vector q_dot;  // Computed joint velocities to reach the target
    
    yarp::sig::Matrix lim;  //matrix with joint position limits for the current chain
    yarp::sig::Matrix vLimNominal;     //matrix with min/max velocity limits for the current chain
    yarp::sig::Matrix vLimAdapted;  //matrix with min/max velocity limits after adptation by avoidanceHandler
      
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
    double t_0, t_1;
    int start_experiment, counter;
 
    // IPOPT STUFF
    int ipoptExitCode;
    double timeToSolveProblem_s; //time taken by q_dot = solveIK(ipoptExitCode) ~ ipopt + avoidance handler
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app; // pointer to instance of main application class for making calls to Ipopt
    Ipopt::SmartPtr<ControllerNLP> nlp; //pointer to IK solver instance

    // Mutex for handling things correctly
//    std::mutex mut;
    yarp::os::Bottle    cmd; 
    yarp::sig::Matrix T_world_root; //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR
    
    bool visualizeIniCubGui;
    bool visualizeParticleIniCubGui;
    bool visualizeTargetIniCubGui;

    bool firstSolve; //ipopt OptimizeTNLP only for first time to allocate memory, then use ReOptimizeTNLP
    // objects in simulator will be created only for first target - with new targets they will be moved
    bool firstTarget;
    std::vector<collisionPoint_t> collisionPoints; //list of "avoidance vectors" from peripersonal space / safety margin
    int collisionPointsVisualizedCount; //objects will be created in simulator and then their positions updated every iteration
    yarp::sig::Vector collisionPointsSimReservoirPos; //inactive collision points will be stored in the world
    std::unique_ptr<AvoidanceHandlerAbstract> avhdl;
        
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
    bool controlArm(const string& controlMode,const yarp::sig::Vector &);

    /**
     * Check the state of each joint to be controlled
     * @param  jointsToSet vector of integers that defines the joints to be set
     * @param  _p part to set. It can be either "torso" or "arm"
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return             true/false if success/failure
     */
    bool areJointsHealthyAndSet(std::vector<int> &jointsToSet,
                                const string &_p, const string &_s);

    /**
     * Changes the control modes of the torso to either position or velocity
     * @param  _p part to set. It can be either "torso" or "arm"
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return    true/false if success/failure
     */
    bool setCtrlModes(const std::vector<int> &jointsToSet,
                      const string &_p, const string &_s);


    bool stopControlHelper();

    bool stopControlAndSwitchToPositionModeHelper();

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
    
    void convertPosFromLinkToRootFoR(const yarp::sig::Vector &pos, iCub::skinDynLib::SkinPart skinPart, yarp::sig::Vector &outPos);
        

   /************************** communication through ports in/out ***********************************/

    bool getCollisionPointsFromPort(yarp::os::BufferedPort<yarp::os::Bottle> &inPort, double gain, const string& whichChain,std::vector<collisionPoint_t> &collPoints);

    /**
    * Sends useful data to a port in order to track it on matlab
    **/
    void sendData();

    /**
    * @brief Receive trajectories of control points from planner
    * @param x_desired standard vector of set of 3D points
    * @return true if received trajectories are valid, false otherwise
    */
    bool readMotionPlan(std::vector<yarp::sig::Vector> &x_desired);

    /***************************** visualizations in icubGui  ****************************/
    //uses corresponding global variables for target pos (x_d) or particle pos (x_n) and creates bottles for the port to iCubGui
    void sendiCubGuiObject(const std::string& object_type);
    
    void deleteiCubGuiObject(const std::string& object_type);
    
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
    int printMessage(int l, const char *f, ...) const;


};

#endif

