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
#include <yarp/os/Network.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>

#include <iCub/iKin/iKinFwd.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <vector>

#include "reactIpOpt.h"
#include "particleThread.h"

using namespace yarp::dev;

using namespace std;

class reactCtrlThread: public yarp::os::RateThread
{
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
    // [DEPRECATED] Trajectory time (default 3.0 s)
    double trajTime;
    // Tracjectory speed (default 0.1 m/s)
    double trajSpeed;
    // Tolerance of the ipopt task. The solver exits if norm2(x_d-x)<tol.
    double tol;
    // Global tolerance of the task. The controller exits if norm(x_d-x)<globalTol
    double globalTol;
    // Max velocity set for the joints
    double vMax;
    // will use the yarp rpc /icubSim/world to visualize the target
    bool visualizeTargetInSim;
    // will use the yarp rpc /icubSim/world to visualize the particle (trajectory - intermediate targets)
    bool visualizeParticleInSim; 
    // will use the yarp rpc /icubSim/world to visualize the potential collision points
    bool visualizeCollisionPointsInSim;
    
    /***************************************************************************/
    // INTERNAL VARIABLES:
    particleThread  *prtclThrd;     // Pointer to the particleThread in order to access its data

    int        state;        // Flag to know in which state the thread is in
    
    double      t_0;        // Time at which the trajectory starts - currently these params are not used 
    double      t_d;        // Time at which the trajectory should end - currently these params are not used
    yarp::sig::Vector x_0;  // Initial end-effector position
    yarp::sig::Vector x_t;  // Current end-effector position
    yarp::sig::Vector x_n;  // Desired next end-effector position
    yarp::sig::Vector x_d;  // Vector that stores the new target

    yarp::sig::Vector q_dot_0;    // Initial arm configuration
    yarp::sig::Vector q_dot;  // Computed arm configuration to reach the target
    yarp::sig::Matrix H;      // End-effector pose

    yarp::os::Port outPort;
    yarp::os::Port portToSimWorld;
    
    // Driver for "classical" interfaces
    PolyDriver       ddA;
    PolyDriver       ddT;

    // "Classical" interfaces for the arm
    IEncoders            *iencsA;
    IVelocityControl2     *ivelA;
    IControlMode2         *imodA;
    IControlLimits        *ilimA;
    yarp::sig::Vector     *encsA;
    iCub::iKin::iCubArm     *arm;
    int jntsA;

    // "Classical" interfaces for the torso
    IEncoders            *iencsT;
    IVelocityControl2     *ivelT;
    IControlMode2         *imodT;
    IControlLimits        *ilimT;
    yarp::sig::Vector     *encsT;
    int jntsT;

    // IPOPT STUFF
    reactIpOpt    *slv;    // solver
    int nDOF;

    // Mutex for handling things correctly
    yarp::os::Mutex mutex;
    
    yarp::os::Bottle    cmd; 
    yarp::sig::Matrix T; //from robot to simulator reference frame
    
    // objects in simulator will be created only for first target - with new targets they will be moved
    bool firstTarget;
    std::vector<collisionPoint_t> collisionPoints; //list of "avoidance vectors" from peripersonal space / safety margin
    int collisionPointsVisualizedCount; //objects will be created in simulator and then their positions updated every iteration
    yarp::sig::Vector collisionPointsSimReservoirPos; //inactive collision points will be stored in the world
    
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
    * Computes the spatial step that will be performed by the robot
    **/
    yarp::sig::Vector computeDeltaX();

    /**
    * Sends useful data to a port in order to track it on matlab
    **/
    void sendData();

    /**
    * Solves the Inverse Kinematic task
    */
    yarp::sig::Vector solveIK(int &);

    /**
    * Sends the computed velocities to the robot
    */
    bool controlArm(const yarp::sig::Vector &);

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

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    /**
    * Creates a sphere (not affected by gravity) in the iCub simulator through the /icubSim/world port
    * @param radius
    * @param pos  
    */
    void createStaticSphere(double radius, const yarp::sig::Vector &pos);
   
    void moveSphere(int index, const yarp::sig::Vector &pos);
    
    void createStaticBox(const yarp::sig::Vector &pos);
    
    void moveBox(int index, const yarp::sig::Vector &pos);
    
    void convertPosFromRootToSimFoR(const yarp::sig::Vector &pos, yarp::sig::Vector &outPos);
    
    void convertPosFromLinkToRootFoR(const yarp::sig::Vector &pos,const iCub::skinDynLib::SkinPart skinPart, yarp::sig::Vector &outPos);
        
    void showCollisionPointsInSim();
public:
    // CONSTRUCTOR
    reactCtrlThread(int , const string & , const string & , const string &_ ,
                    int , bool , double , double , double , double , bool , bool , bool , particleThread * );
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
    bool setNewTarget(const yarp::sig::Vector&);

    // Sets the new target relative to the current position
    bool setNewRelativeTarget(const yarp::sig::Vector&);

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
};

#endif

