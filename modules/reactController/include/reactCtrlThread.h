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

#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <fstream>

#include "reactOSQP.h"
#include "particleThread.h"
#include "avoidanceHandler.h"
#include "visualisationHandler.h"


using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;

#define NR_ARM_JOINTS 7
#define NR_ARM_JOINTS_FOR_INTERACTION_MODE 5
#define NR_TORSO_JOINTS 3

struct ArmInterface
{
    iCub::ctrl::Integrator *I; //if controlMode == positionDirect, we need to integrate the velocity control commands

    std::string part_name;  // Which arm to use: either left_arm or right_arm
    std::string part_short; // Which arm to use (short version): either left or right

    // Driver for "classical" interfaces
    PolyDriver       ddA;

    // "Classical" interfaces for the arm
    IEncoders             *iencsA;
    IPositionDirect       *iposDirA;
    IControlMode          *imodA;
    IInteractionMode      *iintmodeA;
    IImpedanceControl     *iimpA;
    IControlLimits        *ilimA;
    yarp::sig::Vector     *encsA;
    iCub::iKin::iCubArm   *arm;
    int jntsA;

    std::vector<InteractionModeEnum> interactionModesOrig;
    std::vector<InteractionModeEnum> interactionModesNew;
    std::vector<int> jointsToSetInteractionA;

    size_t chainActiveDOF;
    //parallel virtual arm and chain on which QP will be working in the positionDirect mode case
    iCub::iKin::iCubArm    *virtualArm;

    yarp::sig::Vector x_0;  // Initial end-effector position
    yarp::sig::Vector x_t;  // Current end-effector position
    yarp::sig::Vector x_n;  // Desired next end-effector position
    yarp::sig::Vector x_d;  // Vector that stores the new target
    yarp::sig::Vector x_home;  // Home end-effector position

    //All orientation in Euler angle format
    yarp::sig::Vector o_0;  // Initial end-effector orientation
    yarp::sig::Vector o_t;  // Current end-effector orientation
    yarp::sig::Vector o_n;  // Desired next end-effector orientation
    yarp::sig::Vector o_d;  // Vector that stores the new orientation
    yarp::sig::Vector o_home;  // Home end-effector orientation

    //N.B. All angles in this thread are in degrees
    yarp::sig::Vector qA; //current values of arm joints (should be 7)
    yarp::sig::Vector q; //current joint angle values (10 if torso is on, 7 if off)
    yarp::sig::Vector qIntegrated; //joint angle values integrated from velocity commands by Integrator - controlMode positionDirect only
    yarp::sig::Vector q_dot;  // Computed joint velocities to reach the target

    yarp::sig::Matrix lim;  //matrix with joint position limits for the current chain
    yarp::sig::Matrix vLimNominal;     //matrix with min/max velocity limits for the current chain
    yarp::sig::Matrix vLimAdapted;  //matrix with min/max velocity limits after adptation by avoidanceHandler
    std::vector<collisionPoint_t> collisionPoints; //list of "avoidance vectors" from peripersonal space / safety margin
    std::unique_ptr<AvoidanceHandlerAbstract> avhdl;

    std::vector<double> fingerPos;
    yarp::sig::Vector   homePos;
    std::vector<yarp::sig::Vector> last_trajectory;
    bool avoidance;
    bool useSelfColPoints;

    explicit ArmInterface(std::string _part, bool _selfColPoints);
    void setVMax(bool useTorso, double vMax);
    void updateCollPoints(double dT);
    bool prepareDrivers(const std::string& robot, const std::string& name, bool stiffInteraction);
    void release();
    void updateArm(const Vector& qT);
    void initialization(double dT, iKinChain* chain, int verbosity);
    bool checkRecoveryPath(Vector& next_x);
    void updateRecoveryPath();
    /**
    * Aligns joint bounds according to the actual limits of the robot
     */
    bool alignJointsBound(IControlLimits* ilimT);

    /**
    * Prints the joints bounds from the iCubArm
     */
    void printJointsBounds() const;
};


class reactCtrlThread: public yarp::os::PeriodicThread
{

public:
    // CONSTRUCTOR
    reactCtrlThread(int , std::string   , std::string   , const std::string&  _ , const std::string& ,
                    int , bool , double , double , double , double , double , std::string  ,
                    bool , bool , bool, bool , bool , bool, bool , bool , bool , bool ,
                    particleThread *, double, bool);
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
    bool setStreamingTarget() { streamingTarget = true; return true; }

    // Sets the tolerance
    bool setTol(const double _tol)
    {
        if (_tol < 0) return false;
        tol = _tol; return true;
    }

    // Gets the tolerance
    double getTol() const { return tol; }

    // Sets the vMax
    bool setVMax(double );

    // Gets the vMax
    double getVMax() const { return vMax; }

    // Sets the trajectory speed
    bool setTrajSpeed(const double _traj_speed)
    {
        if (_traj_speed < 0) return false;
        trajSpeed = _traj_speed; return true;
    }

    // Sets the verbosity
    bool setVerbosity(int _verbosity) { verbosity = (_verbosity>=0) ? _verbosity : 0; return true;}

    // gets the verbosity
    int getVerbosity() const { return verbosity; };

    // gets the state of the controller
    int getState() const { return state; };

    // Stops the control of the robot
    bool stopControlAndSwitchToPositionMode();

    // Moves robot to home configuration
    bool goHome();

    // Robot holds its current position
    bool holdPosition();

protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    std::string name;
    // Name of the robot (to address the module toward icub or icubSim):
    std::string robot;
    // Flag to know if the torso shall be used or not
    bool useTorso;
    // Trajectory speed (default 0.1 m/s)
    double trajSpeed;
    // Tolerance of the QP task. The solver exits if norm2(x_d-x)<tol.
    double tol;
    // Global tolerance of the task. The controller exits if norm(x_d-x)<globalTol
    double globalTol;
    // Max velocity set for the joints
    double vMax;
    // Weight of the reaching joint rest position task (disabled if 0.0)
    double restPosWeight;
    double timeLimit;  // time limit to reach target
    std::string referenceGen; // either "uniformParticle" - constant velocity with particleThread - or "minJerk"
    bool tactileCollPointsOn; //if on, will be reading collision points from /skinEventsAggregator/skin_events_aggreg:o
    bool visualCollPointsOn; //if on, will be reading predicted collision points from visuoTactileRF/pps_activations_aggreg:o
    bool proximityCollPointsOn; //if on will be reading predicted collision points from proximity sensor
    bool gazeControl; //will follow target with gaze
    bool stiffInteraction; //stiff vs. compliant interaction mode

    bool hittingConstraints; //inequality constraints for safety of shoudler assembly and to prevent self-collisions torso-upper arm, upper-arm - forearm
    bool orientationControl; //if orientation should be minimized as well
    // will use the yarp rpc /icubSim/world to visualize the potential collision points
    bool visualizeCollisionPointsInSim;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    double dT;  //period of the thread in seconds  =getPeriod();

    particleThread  *prtclThrd;     // Pointer to the particleThread in order to access its data - if referenceGen is "uniformParticle"
    iCub::ctrl::minJerkTrajGen *minJerkTarget; //if referenceGen is "minJerk"
    std::unique_ptr<ArmInterface> main_arm;
    std::unique_ptr<ArmInterface> second_arm;
    int        state;        // Flag to know in which state the thread is in

    // Driver for "classical" interfaces
    PolyDriver       ddT;
    PolyDriver       ddG; // gaze  controller  driver

    std::vector<int> jointsToSetPosT{0,1,2};
    std::vector<int> jointsToSetPosA{0,1,2,3,4,5,6};

    // "Classical" interfaces for the torso
    IEncoders         *iencsT;
    IPositionDirect   *iposDirT;
    IControlMode      *imodT;
    IControlLimits    *ilimT;
    yarp::sig::Vector *encsT;
    int jntsT;
    yarp::sig::Vector qT; //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)

    // Gaze interface
    IGazeControl    *igaze;
    int contextGaze;

    bool movingTargetCircle;
    double radius;
    double frequency;
    yarp::sig::Vector circleCenter;

    bool streamingTarget;
    yarp::os::BufferedPort<yarp::os::Bottle> streamedTargets;

    bool holding_position;
    bool comingHome;

    // ports and files
    yarp::os::BufferedPort<yarp::os::Bottle> proximityEventsInPort; //coming from proximity sensor
    yarp::os::BufferedPort<yarp::os::Bottle> proximityEventsVisuPort; //sending out proximity data
    yarp::os::BufferedPort<yarp::os::Bottle> aggregSkinEventsInPort; //coming from /skinEventsAggregator/skin_events_aggreg:o
    yarp::os::BufferedPort<yarp::os::Bottle> aggregPPSeventsInPort; //coming from visuoTactileRF/pps_activations_aggreg:o
    //expected format for both: (skinPart_s x y z o1 o2 o3 magnitude), with position x,y,z and normal o1 o2 o3 in link FoR
    yarp::os::Port outPort;
    yarp::os::Port movementFinishedPort;
    std::ofstream fout_param; //log parameters that stay constant during the simulation, but are important for analysis - e.g. joint limits
    // Stamp for the setEnvelope for the ports
    yarp::os::Stamp ts;
    double t_0, t_1;
    int counter;

    // QPSolver STUFF
    int solverExitCode;
    double timeToSolveProblem_s; //time taken by q_dot = solveIK(solverExitCode)
    std::unique_ptr<QPSolver> solver;

    VisualisationHandler visuhdl;

    /**
    * Solves the Inverse Kinematic task
     */
    int solveIK();

    yarp::sig::Vector updateNextTarget(bool&);

    void nextMove(bool&);

    /**** kinematic chain, control, ..... *****************************/

    /**
    * Aligns joint bounds according to the actual limits of the robot
     */
    bool alignJointsBounds();

    /**
    * Updates the arm's kinematic chain with the encoders from the robot
    **/
    void updateArmChain();

    /**
    * Sends the computed velocities or positions to the robot, depending on controlMode
    */
    bool controlArm(const std::string& controlMode);

    /**
     * Check the state of each joint to be controlled
     * @param  jointsToSet vector of integers that defines the joints to be set
     * @param  _p part to set. It can be either "torso" or "arm"
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return             true/false if success/failure
     */
    bool areJointsHealthyAndSet(std::vector<int> &jointsToSet,
                                const std::string &_p, const std::string &_s);

    /**
     * Changes the control modes of the torso to either position or velocity
     * @param  _p part to set. It can be either "torso" or "arm"
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return    true/false if success/failure
     */
    bool setCtrlModes(const std::vector<int> &jointsToSet,
                      const std::string &_p, const std::string &_s);


    bool prepareDrivers();

    /***************** auxiliary computations  *******************************/

    /**
    * Computes the next target on a circular trajectory, using global vars frequency, radius, and circleCenter
    * @return new target position
    **/
    yarp::sig::Vector  getPosMovingTargetOnCircle();

    void insertTestingCollisions();

    void getCollisionsFromPorts();

    bool preprocCollisions();
    /************************** communication through ports in/out ***********************************/

    void getCollPointFromPort(Bottle* bot, double gain);

    void getCollisionPointsFromPort(BufferedPort<Bottle> &inPort, double gain);

    /**
     * writing to param file
     */
    void writeConfigData();

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


    /**
    * Prints a message according to the verbosity level:
    * @param l will be checked against the global var verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
     */
    int printMessage(int l, const char *f, ...) const;

};

#endif
