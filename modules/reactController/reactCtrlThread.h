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
#include <yarp/dev/all.h>

#include <iCub/iKin/iKinFwd.h>

#include <gsl/gsl_math.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>

#include "reactIpOpt.h"

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
    // Flag used to know if the doubleTouch should automatically connect to the skinManager
    bool autoconnect;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    int    step;            // Flag to know in which step the thread is in
    bool isTask;            // Flag to know if there is a task to solve
    yarp::sig::Vector xD;   // Vector that stores the new target


    // Driver for "classical" interfaces
    PolyDriver       dd;

    // "Classical" interfaces - SLAVE ARM
    IEncoders            *iencs;
    IPositionControl2     *ipos;
    IInteractionMode     *imode;
    IImpedanceControl     *iimp;
    IControlLimits        *ilim;
    yarp::sig::Vector     *encs;
    iCub::iKin::iCubArm    *arm;
    int jnts;

    // IPOPT STUFF
    reactIpOpt    *slv;    // solver
    yarp::sig::Vector solution;
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
    void solveIK();

    /**
    * Toggles the internal state to the active state
    * @param  _task      boolean that is true/false if task is on/off
    * @return true/false if success/failure
    **/
    bool toggleTask(bool _task) { return isTask=_task; }

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR
    reactCtrlThread(int , const string & , const string & ,
                    const string &_ , int , bool  );
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    // Sets the new target
    bool setNewTarget(const yarp::sig::Vector&);
};

#endif

