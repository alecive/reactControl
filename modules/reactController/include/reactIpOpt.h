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

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

/**
*
* Class for inverting chain's kinematics based on IpOpt lib
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

    iCub::iKin::iKinChain &chain;

    iCub::iKin::iKinLinIneqConstr  noLIC;
    iCub::iKin::iKinLinIneqConstr *pLIC;

    double obj_scaling;
    double x_scaling;
    double g_scaling;
    double lowerBoundInf;
    double upperBoundInf;

    int verbosity;

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param tol exits if 0.5*norm(xd-x)^2<tol.
    * @param max_iter exits if iter>=max_iter (max_iter<0 disables
    *                 this check, IKINCTRL_DISABLED(==-1) by
    *                 default).
    * @param verbose is a integer number which progressively enables 
    *                different levels of warning messages or status
    *                dump. The larger this value the more detailed
    *                is the output (0=>off by default).
    * @param useHessian relies on exact Hessian computation or  
    *                enable Quasi-Newton approximation (true by
    *                default).
    */
    reactIpOpt(iCub::iKin::iKinChain &c,
               const double tol, const int max_iter=IKINCTRL_DISABLED,
               const unsigned int verbose=0, bool useHessian=true);

    /**
    * Attach a iKinLinIneqConstr object in order to impose 
    * constraints of the form lB <= C*q <= uB.
    * @param lic is the iKinLinIneqConstr object to attach.
    * @see iKinLinIneqConstr
    */
    void attachLIC(iCub::iKin::iKinLinIneqConstr &lic) { pLIC=&lic; }

    /**
    * Returns a reference to the attached Linear Inequality 
    * Constraints object.
    * @return Linear Inequality Constraints pLIC. 
    * @see iKinLinIneqConstr
    */
    iCub::iKin::iKinLinIneqConstr &getLIC() { return *pLIC; }

    /**
    * Sets Maximum Iteration.
    * @param max_iter exits if iter>=max_iter (max_iter<0 
    *                 (IKINCTRL_DISABLED) disables this check).
    */ 
    void setMaxIter(const int max_iter);

    /**
    * Retrieves the current value of Maximum Iteration.
    * @return max_iter. 
    */ 
    int getMaxIter() const;

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
    * Enables/disables user scaling factors.
    * @param useUserScaling true if user scaling is enabled. 
    * @param obj_scaling user scaling factor for the objective 
    *                    function.
    * @param x_scaling user scaling factor for variables. 
    * @param g_scaling user scaling factor for constraints. 
    */
    void setUserScaling(const bool useUserScaling, const double _obj_scaling,
                        const double _x_scaling, const double _g_scaling);

    /**
    * Returns the lower and upper bounds to represent -inf and +inf.
    * @param lower is a reference to return the lower bound.
    * @param upper is a reference to return the upper bound. 
    */
    void getBoundsInf(double &lower, double &upper);

    /**
    * Sets the lower and upper bounds to represent -inf and +inf.
    * @param lower is the new lower bound. 
    * @param upper is the new upper bound. 
    */
    void setBoundsInf(const double lower, const double upper);

    /**
    * Executes the IpOpt algorithm trying to converge on target. 
    * @param xd        is the End-Effector target Pose to be attained. 
    * @param q_dot_0   are the initial joint velocities of the chain.
    * @param dt        is the time step to use in order to solve the task. 
    * @param cup_time  is the total time spent by IPOPT to solve the task.
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
    virtual yarp::sig::Vector solve(yarp::sig::Vector &xd, yarp::sig::Vector q_dot_0,
                                    double &dt, double *cpu_time, int *exit_code);

    /**
    * Default destructor.
    */
    virtual ~reactIpOpt();
};

#endif


