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

#ifndef __REACTIPOPT_H__
#define __REACTIPOPT_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Log.h>

#include <yarp/math/SVD.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <iCub/skinDynLib/common.h>

/**
*
* Class for reactive control of kinematic chain with target and obstacles
*/
class reactIpOpt
{
private:
    // Default constructor: not implemented.
    reactIpOpt();
    // Copy constructor: not implemented.
    reactIpOpt(const reactIpOpt&);
    // Assignment operator: not implemented.
    reactIpOpt &operator=(const reactIpOpt&);    

protected:
    // The IpOpt application that supposedly will solve the task
    void *App;

    iCub::iKin::iKinChain chain;

    int verbosity;

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param tol exits if 0.5*norm(xd-x)^2<tol.
    * @param verbose is a integer number which progressively enables 
    *                different levels of warning messages or status
    *                dump. The larger this value the more detailed
    *                is the output (0=>off by default).
    */
    reactIpOpt(const iCub::iKin::iKinChain &c,
               const double tol, const unsigned int verbose=0);

    /**
    * Sets Tolerance.
    * @param tol exits if norm(xd-x)<tol.
    */
    void setTol(const double tol);

    /**
    * Retrieves Tolerance.
    * @return tolerance.
    */
    double getTol() const;

    /**
    * Sets Verbosity.
    * @param verbose is a integer number which progressively enables 
    *                different levels of warning messages or status
    *                dump. The larger this value the more detailed
    *                is the output.
    */
    void setVerbosity(const unsigned int verbose);

    /**
    * Executes the IpOpt algorithm trying to converge on target. 
    * @param xd        is the End-Effector target Pose to be attained. 
    * @param q_dot_0   are the initial joint velocities of the chain.
    * @param dt        is the time step to use in order to solve the task. 
    * @param v_lim     are the joint velocity limits for individual joints.
    * @param boundSmoothnessFlag whether the optimization should limit changes in joint vel control commmands between time steps
    * @param boundSmoothnessValue  the actual value of the max allowed change in joint vel
    * @param exit_code stores the exit code (NULL by default). It is one of these:
    *                   SUCCESS
    *                   MAXITER_EXCEEDED
    *                   CPUTIME_EXCEEDED
    *                   STOP_AT_TINY_STEP
    *                   STOP_AT_ACCEPTABLE_POINT
    *                   LOCAL_INFEASIBILITY
    *                   USER_REQUESTED_STOP
    *                   FEASIBLE_POINT_FOUND
    *                   DIVERGING_ITERATES
    *                   RESTORATION_FAILURE
    *                   ERROR_IN_STEP_COMPUTATION
    *                   INVALID_NUMBER_DETECTED
    *                   TOO_FEW_DEGREES_OF_FREEDOM
    *                   INTERNAL_ERROR
    * @return estimated joint velocities.
    */
    virtual yarp::sig::Vector solve(const yarp::sig::Vector &xd, const yarp::sig::Vector &q_dot_0,
                                    double dt, const yarp::sig::Matrix &v_lim, bool boundSmoothnessFlag, double boundSmoothnessValue, int *exit_code);

    /**
    * Default destructor.
    */
    virtual ~reactIpOpt();
};

#endif


