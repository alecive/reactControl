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
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

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

    /***************************************************************************/
    // INTERNAL VARIABLES:
    particleThread  *prtclThrd;     // Pointer to the particleThread in order to access its data

    int        step;        // Flag to know in which step the thread is in
    
    double      t_0;        // Time at which the trajectory starts
    double      t_d;        // Time at which the trajectory should end
    yarp::sig::Vector x_d;  // Vector that stores the new target
    yarp::sig::Vector x_0;  // Initial end-effector position
    yarp::sig::Vector x_t;  // Current end-effector position

    yarp::sig::Vector vel;  // 3D velocity vector that describes the trajectory

    yarp::sig::Vector q_0;   // Initial arm configuration
    yarp::sig::Matrix H;    // End-effector pose

    double tol;         // Tolerance. The solver exits if norm(x_d-x)<tol.

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

    /**
    * Aligns joint bounds according to the actual limits of the robot
    */
    bool alignJointsBounds();

    /**
    * Updates the arm's kinematic chain with the encoders from the robot
    **/
    void updateArmChain();

    /**
    * Solves the Inverse Kinematic task
    */
    yarp::sig::Vector solveIK(int &);

    /**
    * Sends the computed velocities to the robot
    */
    bool controlArm(const yarp::sig::Vector &);

    /**
    * Sends the computed velocities to the robot
    */
    bool stopControl();

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
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR
    reactCtrlThread(int , const string & , const string & , const string &_ ,
                    int , bool , double , double , particleThread * );
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    // Sets the new target
    bool setNewTarget(const yarp::sig::Vector&);

    // Sets the new target relative to the current position
    bool setNewRelativeTarget(const yarp::sig::Vector&);

    // Sets the tolerance
    bool setTol(const double );

    // [DEPRECATED] Sets the trajectory time 
    bool setTrajTime(const double );

    // Sets the trajectory speed
    bool setTrajSpeed(const double );

    // Sets the verbosity
    bool setVerbosity(const int );

    // gets the verbosity
    int getVerbosity() { return verbosity; };
};

#endif

