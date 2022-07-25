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
using namespace std;

#define NR_ARM_JOINTS 7
#define NR_ARM_JOINTS_FOR_INTERACTION_MODE 5
#define NR_TORSO_JOINTS 3

struct ArmInterface
{
    iCub::ctrl::Integrator *I; //if controlMode == positionDirect, we need to integrate the velocity control commands

    string part_name;  // Which arm to use: either left_arm or right_arm
    string part_short; // Which arm to use (short version): either left or right

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

    vector<InteractionModeEnum> interactionModesOrig;
    vector<InteractionModeEnum> interactionModesNew;
    vector<int> jointsToSetInteractionA;

    size_t chainActiveDOF;
    //parallel virtual arm and chain on which ipopt will be working in the positionDirect mode case
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

public:
    ArmInterface(std::string _part): I(nullptr), part_name(std::move(_part)), iencsA(nullptr), iposDirA(nullptr),
            imodA(nullptr), iintmodeA(nullptr), iimpA(nullptr), ilimA(nullptr), encsA(nullptr), arm(nullptr),
            jntsA(0), chainActiveDOF(0), virtualArm(nullptr), avhdl(nullptr), fingerPos({80,6,57,13,0,13,0,103}),
            homePos({0, 0, 0, -34, 30, 0, 50, 0,  0, 0})
    {
        if (part_name=="left_arm")
        {
            part_short="left";
        }
        else if (part_name=="right_arm")
        {
            part_short="right";
        }

        /******** iKin chain and variables, and transforms init *************************/
        arm = new iCub::iKin::iCubArm(part_short+"_v2");
        // Release / block torso links (blocked by default)
        for (int i = 0; i < NR_TORSO_JOINTS; i++)
        {
            arm->releaseLink(i);
        }
        //we set up the variables based on the current DOF - that is without torso joints if torso is blocked
        chainActiveDOF = arm->getDOF();

        //N.B. All angles in this thread are in degrees
        qA.resize(NR_ARM_JOINTS,0.0); //current values of arm joints (should be 7)
        q.resize(chainActiveDOF,0.0); //current joint angle values (10 if torso is on, 7 if off)
        qIntegrated.resize(chainActiveDOF,0.0); //joint angle pos predictions from integrator

        q_dot.resize(chainActiveDOF,0.0);
        vLimNominal.resize(chainActiveDOF,2);
        vLimAdapted.resize(chainActiveDOF,2);

        Time::delay(1);

        x_0.resize(3,0.0);
        x_t.resize(3,0.0);
        x_n.resize(3,0.0);
        x_d.resize(3,0.0);
        //store the home position
        // Vector pose = arm->EndEffPose();
        // x_home = pose.subVector(0,2);
        // o_home = pose.subVector(3,5)*pose(6);
        x_home = (part_short == "left")? Vector{-0.304, -0.202, 0.023}:Vector{-0.304, 0.202, 0.023};
        o_home = (part_short == "left")? Vector{-0.071, 1.710, -2.264}:Vector{-0.470, -2.440, 1.843};

        //palm facing inwards
        o_0.resize(3,0.0);  o_0(1)=-0.707*M_PI;     o_0(2)=+0.707*M_PI;
        o_t.resize(3,0.0);  o_t(1)=-0.707*M_PI;     o_t(2)=+0.707*M_PI;
        o_n.resize(3,0.0);  o_n(1)=-0.707*M_PI;     o_n(2)=+0.707*M_PI;
        o_d.resize(3,0.0);  o_d(1)=-0.707*M_PI;     o_d(2)=+0.707*M_PI;

        //  set grasping pose for fingers
        //    for (size_t j=0; j<fingerPos.size(); j++)
        //    {
        //        imodA->setControlMode(8+j, VOCAB_CM_POSITION_DIRECT);
        //        iposDirA->setPosition(8+j, fingerPos[j]);
        //
        //    }

        virtualArm = new iCubArm(*arm);  //Creates a new Limb from an already existing Limb object - but they will be too independent limbs from now on
    }

    void updateJointLimMatrix()
    {
        lim.resize(chainActiveDOF,2); //joint pos limits
        //filling joint pos limits Matrix
        iKinChain& armChain=*arm->asChain();
        for (size_t jointIndex=0; jointIndex<chainActiveDOF; jointIndex++)
        {
            lim(jointIndex,0)= CTRL_RAD2DEG*armChain(jointIndex).getMin();
            lim(jointIndex,1)= CTRL_RAD2DEG*armChain(jointIndex).getMax();
        }
    }

    void setVMax(bool useTorso, double vMax)
    {
        for (size_t r=0; r<chainActiveDOF; r++)
        {
            vLimNominal(r,0)=-vMax;
            vLimAdapted(r,0)=-vMax;
            vLimNominal(r,1)=vMax;
            vLimAdapted(r,1)=vMax;
        }
        if (useTorso){
            vLimNominal(1,0)=vLimNominal(1,1)=0.0;
            vLimAdapted(1,0)=vLimAdapted(1,1)=0.0;
        }
        else // TODO chainActiveDOF is still 10
        {
            vLimAdapted.setSubcol({0.0,0.0,0.0}, 0,0);
            vLimNominal.setSubcol({0.0,0.0,0.0}, 0,0);
            vLimAdapted.setSubcol({0.0,0.0,0.0}, 0,1);
            vLimNominal.setSubcol({0.0,0.0,0.0}, 0,1);
        }
    }

    void updateCollPoints(double dT)
    {
        for (auto& colP : collisionPoints)
        {
            colP.duration -= dT;
            colP.magnitude *= 0.8;
        }
        collisionPoints.erase(std::remove_if(collisionPoints.begin(), collisionPoints.end(),
                                             [](const collisionPoint_t & colP) { return colP.duration <= 0; }),
                              collisionPoints.end());
    }

    bool prepareDrivers(const std::string& robot, const std::string& name, bool stiffInteraction)
    {
        yarp::os::Property OptA;
        OptA.put("robot",  robot);
        OptA.put("part",   part_name);
        OptA.put("device", "remote_controlboard");
        OptA.put("remote", "/"+robot+"/"+part_name);
        OptA.put("local",  "/"+name +"/"+part_name);
        if (!ddA.open(OptA))
        {
            yError("[reactCtrlThread]Could not open %s PolyDriver!",part_name.c_str());
            return false;
        }

        bool okA = false;

        if (ddA.isValid())
        {
            okA = ddA.view(iencsA) && ddA.view(iposDirA) && ddA.view(imodA)
               && ddA.view(ilimA) && ddA.view(iintmodeA) && ddA.view(iimpA);
        }
        iencsA->getAxes(&jntsA);
        encsA = new yarp::sig::Vector(jntsA,0.0);

        if (!okA)
        {
            yError("[reactCtrlThread]Problems acquiring %s interfaces!!!!", part_name.c_str());
            return false;
        }
        interactionModesOrig.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_STIFF);
        jointsToSetInteractionA.clear();
        for (int i=0; i<NR_ARM_JOINTS_FOR_INTERACTION_MODE;i++)
        {
            jointsToSetInteractionA.push_back(i);
        }
        iintmodeA->getInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesOrig.data());
        if(stiffInteraction)
        {
            interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_STIFF);
            iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesNew.data());
        }
        else // not working -> joints in HW fault
        {
            interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_COMPLIANT);
            iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesNew.data());
            iimpA->setImpedance(0,0.4,0.03);
            iimpA->setImpedance(1,0.4,0.03);
            iimpA->setImpedance(2,0.4,0.03);
            iimpA->setImpedance(3,0.2,0.01);
            iimpA->setImpedance(4,0.05,0.0);
        }
        return true;
    }

    void release()
    {
        yInfo("Putting back original interaction modes.");
        iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesOrig.data());
        interactionModesNew.clear();
        interactionModesOrig.clear();
        jointsToSetInteractionA.clear();

        yInfo("threadRelease(): deleting arm and torso encoder arrays and arm object.");
        delete encsA; encsA = nullptr;
        delete   arm;   arm = nullptr;
        yInfo("Closing controllers..");
        ddA.close();
        collisionPoints.clear();

        if(virtualArm != nullptr)
        {
            yDebug("deleting virtualArm..");
            delete virtualArm;
            virtualArm = nullptr;
        }
        if(I != nullptr)
        {
            yDebug("deleting integrator I..");
            delete I;
            I = nullptr;
        }
    }

};


class reactCtrlThread: public yarp::os::PeriodicThread
{

public:
    // CONSTRUCTOR
    reactCtrlThread(int , string   , string   , string  _ , string ,
                    int , bool , double , double , double , double , double , string  ,
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
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;
    // Which arm to use: either left_arm or right_arm
    string part;
    // Which arm to use as second: left_arm or right_arm or None
    string second_part;
    // Flag to know if the torso shall be used or not
    bool useTorso;
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
    double timeLimit;  // time limit to reach target
    string referenceGen; // either "uniformParticle" - constant velocity with particleThread - or "minJerk"
    bool tactileCollPointsOn; //if on, will be reading collision points from /skinEventsAggregator/skin_events_aggreg:o
    bool visualCollPointsOn; //if on, will be reading predicted collision points from visuoTactileRF/pps_activations_aggreg:o
    bool proximityCollPointsOn; //if on will be reading predicted collision points from proximity sensor
    bool selfColPoints; // add robot body parts as the collision points to the avoidance handler
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
    unique_ptr<ArmInterface> main_arm;
    unique_ptr<ArmInterface> second_arm;
    int        state;        // Flag to know in which state the thread is in

    // Driver for "classical" interfaces
    PolyDriver       ddT;
    PolyDriver       ddG; // gaze  controller  driver

    vector<int> jointsToSetPosT{0,1,2};
    vector<int> jointsToSetPosA{0,1,2,3,4,5,6};

    // "Classical" interfaces for the torso
    IEncoders         *iencsT;
    IPositionDirect   *iposDirT;
    IControlMode      *imodT;
    IControlLimits    *ilimT;
    yarp::sig::Vector *encsT;
    int jntsT;

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
    bool tactileColAvoidance;

    yarp::sig::Vector qT; //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)
    yarp::sig::Vector weighted_normal; // weighted collision normal

    // ports and files
    yarp::os::BufferedPort<yarp::os::Bottle> proximityEventsInPort; //coming from proximity sensor
    yarp::os::BufferedPort<yarp::os::Bottle> aggregSkinEventsInPort; //coming from /skinEventsAggregator/skin_events_aggreg:o
    yarp::os::BufferedPort<yarp::os::Bottle> aggregPPSeventsInPort; //coming from visuoTactileRF/pps_activations_aggreg:o
    //expected format for both: (skinPart_s x y z o1 o2 o3 magnitude), with position x,y,z and normal o1 o2 o3 in link FoR
    yarp::os::Port outPort;
    yarp::os::Port movementFinishedPort;
    ofstream fout_param; //log parameters that stay constant during the simulation, but are important for analysis - e.g. joint limits
    // Stamp for the setEnvelope for the ports
    yarp::os::Stamp ts;
    double t_0, t_1;
    int counter;

    // QPSolver STUFF
    int solverExitCode;
    double timeToSolveProblem_s; //time taken by q_dot = solveIK(solverExitCode) ~ ipopt + avoidance handler
    std::unique_ptr<QPSolver> solver;

    VisualisationHandler visuhdl;
    std::vector<yarp::sig::Vector> last_trajectory;

    /**
    * Solves the Inverse Kinematic task
     */
    int solveIK();

    yarp::sig::Vector updateNextTarget();

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
    bool controlArm(const string& controlMode);

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


    bool stopControlAndSwitchToPositionModeHelper();

    bool prepareDrivers();

    /***************** auxiliary computations  *******************************/

    /**
    * Computes the next target on a circular trajectory, using global vars frequency, radius, and circleCenter
    * @return new target position
    **/
    yarp::sig::Vector  getPosMovingTargetOnCircle();

    bool insertTestingCollisions();

    bool getCollisionsFromPorts();

    bool preprocCollisions();
    /************************** communication through ports in/out ***********************************/

    void getCollPointFromPort(Bottle* bot, double gain);

    bool getCollisionPointsFromPort(BufferedPort<Bottle> &inPort, double gain);

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

