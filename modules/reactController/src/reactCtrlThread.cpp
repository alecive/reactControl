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

#include <yarp/os/Time.h>
#include <iCub/skinDynLib/common.h>

#include "reactCtrlThread.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#include <IpIpoptApplication.hpp>

using namespace yarp::sig;
using namespace yarp::math;

#define STATE_WAIT              0
#define STATE_REACH             1
#define STATE_IDLE              2

reactCtrlThread::reactCtrlThread(int _rate, const string &_name, const string &_robot,
                                 const string &_part, int _verbosity, bool _disableTorso,
                                 double _trajSpeed, double _globalTol, double _vMax, double _tol, particleThread *_pT) :
                                 RateThread(_rate), name(_name), robot(_robot), part(_part),
                                 verbosity(_verbosity), useTorso(!_disableTorso),
                                 trajSpeed(_trajSpeed), globalTol(_globalTol), vMax(_vMax), tol(_tol)
{
    prtclThrd=_pT;
    state=STATE_WAIT;

    if (part=="left_arm")
    {
        part_short="left";
    }
    else if (part=="right_arm")
    {
        part_short="right";
    }
    arm = new iCub::iKin::iCubArm(part_short.c_str());
    q_dot_0.resize(arm->getDOF(),0.0);

    // Block torso links
    for (int i = 0; i < 3; i++)
    {
        if (useTorso)
        {
            arm->releaseLink(i);
        }
        else
        {
            arm->blockLink(i,0.0);
        }
    }

    slv=NULL;
    x_0.resize(3,0.0);
    x_t.resize(3,0.0);
    x_n.resize(3,0.0);
    x_d.resize(3,0.0);
    
    H.resize(4,4);
}

bool reactCtrlThread::threadInit()
{
    yarp::os::Property OptA;
    OptA.put("robot",  robot.c_str());
    OptA.put("part",   part.c_str());
    OptA.put("device", "remote_controlboard");
    OptA.put("remote",("/"+robot+"/"+part).c_str());
    OptA.put("local", ("/"+name +"/"+part).c_str());
    if (!ddA.open(OptA))
    {
        yError("[reactCtrlThread]Could not open %s PolyDriver!",part.c_str());
        return false;
    }

    bool okA = 1;
    
    if (ddA.isValid())
    {
        okA = okA && ddA.view(iencsA);
        okA = okA && ddA.view(ivelA);
        okA = okA && ddA.view(imodA);
        okA = okA && ddA.view(ilimA);
    }
    iencsA->getAxes(&jntsA);
    encsA = new yarp::sig::Vector(jntsA,0.0);

    if (!okA)
    {
        yError("[reactCtrlThread]Problems acquiring %s interfaces!!!!",part.c_str());
        return false;
    }

    yarp::os::Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name +"/torso").c_str());
    if (!ddT.open(OptT))
    {
        yError("[reactCtrlThread]Could not open torso PolyDriver!");
        return false;
    }

    bool okT = 1;
    
    if (ddT.isValid())
    {
        okT = okT && ddT.view(iencsT);
        okT = okT && ddT.view(ivelT);
        okT = okT && ddT.view(imodT);
        okT = okT && ddT.view(ilimT);
    }
    iencsT->getAxes(&jntsT);
    encsT = new yarp::sig::Vector(jntsT,0.0);

    if (!okT)
    {
        yError("[reactCtrlThread]Problems acquiring torso interfaces!!!!");
        return false;
    }

    if (!alignJointsBounds())
    {
        yError("[reactCtrlThread]alignJointsBounds failed!!!\n");
        return false;
    }

    outPort.open("/"+name +"/data:o");

    yarp::os::Time::delay(0.2);

    return true;
}

void reactCtrlThread::run()
{
    updateArmChain();

    switch (state)
    {
        case STATE_WAIT:
        {
            // Vector x_d(3,0.0);
            // x_d    =x_t;
            // x_d[2]+=0.2;
            // setNewTarget(x_d);
            break;
        }
        case STATE_REACH:
        {
            // if (yarp::os::Time::now()>t_d)
            if (norm(x_t-x_d) < globalTol)
            {
                printf("\n");
                yDebug(0,"[reactCtrlThread] norm(x_t-x_d) %g\tglobalTol %g\n",norm(x_t-x_d),globalTol);
                if (!stopControl())
                {
                    yError("[reactCtrlThread] Unable to properly stop the control of the arm!");
                }
                
                break;
            }

            int exit_code;
            q_dot = solveIK(exit_code);
            // state++; // This is for testing purposes. To be removed!

            if (exit_code==Ipopt::Solve_Succeeded ||
                exit_code==Ipopt::Maximum_CpuTime_Exceeded)
            {
                if (exit_code==Ipopt::Maximum_CpuTime_Exceeded)
                {
                    yWarning("[reactCtrlThread] Ipopt cpu time was higher than the rate of the thread!");
                }
                
                if (!controlArm(q_dot))
                {
                    yError("I am not able to properly control the arm!");
                }
            }

            break;
        }
        case STATE_IDLE:
            yInfo("[reactCtrlThread] finished.");
            state=STATE_WAIT;
            break;
        default:
            yError("[reactCtrlThread] reactCtrlThread should never be here!!! Step: %d",state);
            yarp::os::Time::delay(2.0);
            break;
    }

    sendData();
}

Vector reactCtrlThread::solveIK(int &_exit_code)
{
    yarp::os::LockGuard lg(mutex);
    slv=new reactIpOpt(*arm->asChain(),tol,100,verbosity,false);
    // Next step will be provided iteratively.
    // The equation is x(t_next) = x_t + (x_d - x_t) * (t_next - t_now/T-t_now)
    //                              s.t. t_next = t_now + dT
    double dT=getRate()/1000.0;
    double t_t=yarp::os::Time::now();
    int    exit_code=-1;
    double cpu_time=0.0;

    // if (t_t>=t_d)
    // {
    //     return Vector(3,0.0);
    // }

    // First test: the next point will be given w.r.t. the current one
    // x_n = x_t + (x_d-x_t) * (dT/(t_d-t_t));
    // Second test: the next point will be agnostic of the current 
    // configuration
    // x_n = x_0 + (x_d-x_0) * ((t_t+dT-t_0)/(t_d-t_0));
    // Third solution: use the particleThread
    // If the particle reached the target, let's stop it
    if (norm(x_n-x_0) > norm(x_d-x_0))
    {
        prtclThrd->resetParticle(x_d);
    }

    x_n=prtclThrd->getParticle();

    // Remember: at this stage everything is kept in degrees because the robot is controlled in degrees.
    // At the ipopt level it comes handy to translate everything in radians because iKin works in radians.
    // So, q_dot_0 is in degrees, but I have to convert it in radians before sending it to ipopt
    Vector res=slv->solve(x_n,q_dot_0*CTRL_DEG2RAD,dT,vMax,&cpu_time,&exit_code)*CTRL_RAD2DEG;

    printf("\n");
    // printMessage(0,"t_d: %g\tt_t: %g\n",t_d-t_0, t_t-t_0);
    printMessage(0,"x_n: %s\tx_d: %s\tdT %g\n",x_n.toString(3,3).c_str(),x_d.toString(3,3).c_str(),dT);
    printMessage(0,"x_0: %s\tx_t: %s\n",       x_0.toString(3,3).c_str(),x_t.toString(3,3).c_str());
    printMessage(0,"norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g\n",
                    norm(x_n-x_t), norm(x_d-x_n), norm(x_d-x_t));
    printMessage(0,"Result: %s\n",res.toString(3,3).c_str());
    _exit_code=exit_code;
    q_dot_0=res;

    delete slv;
    return res;
}

bool reactCtrlThread::controlArm(const yarp::sig::Vector &_vels)
{   
    yarp::os::LockGuard lg(mutex);
    VectorOf<int> jointsToSetA;
    VectorOf<int> jointsToSetT;
    if (!areJointsHealthyAndSet(jointsToSetA,"arm","velocity"))
    {
        yWarning("[reactCtrlThread]Stopping control because arm joints are not healthy!");
        stopControl();
        return false;
    }

    if (useTorso)
    {
        if (!areJointsHealthyAndSet(jointsToSetT,"torso","velocity"))
        {
            yWarning("[reactCtrlThread]Stopping control because torso joints are not healthy!");
            stopControl();
            return false;
        }
    }

    if (!setCtrlModes(jointsToSetA,"arm","velocity"))
    {
        yError("[reactCtrlThread]I am not able to set the arm joints to velocity mode!");
        return false;
    }   

    if (useTorso)
    {
        if (!setCtrlModes(jointsToSetT,"torso","velocity"))
        {
            yError("[reactCtrlThread]I am not able to set the torso joints to velocity mode!");
            return false;
        }
    }

    printMessage(1,"Moving the robot with velocities: %s\n",_vels.toString(3,3).c_str());
    if (useTorso)
    {
        Vector velsT(3,0.0);
        velsT[0] = _vels[2];
        velsT[1] = _vels[1];
        velsT[2] = _vels[0];
        
        ivelT->velocityMove(velsT.data());
        ivelA->velocityMove(_vels.subVector(3,9).data());
    }
    else
    {
        ivelA->velocityMove(_vels.data());
    }

    return true;
}

Vector reactCtrlThread::computeDeltaX()
{
    iCub::iKin::iKinChain chain=*arm->asChain();
    yarp::sig::Matrix J1=chain.GeoJacobian();
    yarp::sig::Matrix J_cst;
    J_cst.resize(3,arm->getDOF());
    J_cst.zero();
    submatrix(J1,J_cst,0,2,0,arm->getDOF()-1);
    double dT=getRate()/1000.0;

    return dT*J_cst*q_dot;
}

void reactCtrlThread::sendData()
{
    if (outPort.getOutputCount()>0)
    {
        if (state==STATE_REACH)
        {
            yarp::os::Bottle out;
            out.clear();

            // 1: the sate of the robot
            out.addInt(state);

            // 2: the desired final target
            yarp::os::Bottle &b_x_d=out.addList();
            iCub::skinDynLib::vectorIntoBottle(x_d,b_x_d);

            // 3: the current desired target given by the particle
            yarp::os::Bottle &b_x_n=out.addList();
            iCub::skinDynLib::vectorIntoBottle(x_n,b_x_n);

            // 4: the end effector position in which the robot currently is
            yarp::os::Bottle &b_x_t=out.addList();
            iCub::skinDynLib::vectorIntoBottle(x_t,b_x_t);

            // 5: the delta_x, that is the 3D vector that ipopt commands to 
            //    the robot in order for x_t to reach x_n
            yarp::os::Bottle &b_delta_x=out.addList();
            iCub::skinDynLib::vectorIntoBottle(computeDeltaX(),b_delta_x);

            outPort.write(out);
        }
    }
}

bool reactCtrlThread::stopControl()
{
    yarp::os::LockGuard lg(mutex);

    state=STATE_IDLE;
    if (useTorso)
    {
        return ivelA->stop() && ivelT->stop();
    }

    return ivelA->stop();
}

bool reactCtrlThread::enableTorso()
{
    yarp::os::LockGuard lg(mutex);
    if (state==STATE_REACH)
    {
        return false;
    }

    useTorso=true;
    for (int i = 0; i < 3; i++)
    {
        arm->releaseLink(i);
    }
    alignJointsBounds();
    return true;
}

bool reactCtrlThread::disableTorso()
{
    yarp::os::LockGuard lg(mutex);
    if (state==STATE_REACH)
    {
        return false;
    }

    useTorso=false;
    for (int i = 0; i < 3; i++)
    {
        arm->blockLink(i,0.0);
    }
    return true;
}

bool reactCtrlThread::setTol(const double _tol)
{
    if (_tol>=0.0)
    {
        return tol=_tol;
    }
    return false;
}

double reactCtrlThread::getTol()
{
    return tol;
}

bool reactCtrlThread::setVMax(const double _vMax)
{
    if (_vMax>=0.0)
    {
        return vMax=_vMax;
    }
    return false;
}

double reactCtrlThread::getVMax()
{
    return vMax;
}

bool reactCtrlThread::setTrajTime(const double _traj_time)
{
    yWarning("[reactCtrlThread]trajTime is deprecated! Use trajSpeed instead.");
    return false;
    if (_traj_time>=0.0)
    {
        return trajTime=_traj_time;
    }
}

bool reactCtrlThread::setTrajSpeed(const double _traj_speed)
{
    if (_traj_speed>=0.0)
    {
        return trajSpeed=_traj_speed;
    }
}

bool reactCtrlThread::setVerbosity(const int _verbosity)
{
    return _verbosity>=0?verbosity=_verbosity:verbosity=0;
}

bool reactCtrlThread::setNewTarget(const Vector& _x_d)
{
    if (_x_d.size()==3)
    {
        q_dot_0.resize(arm->getDOF(),0.0);
        q_dot.resize(arm->getDOF(),0.0);
        x_0=x_t;
        x_n=x_0;
        x_d=_x_d;
        t_0=yarp::os::Time::now();
        yarp::sig::Vector vel(3,0.0);
        vel=trajSpeed * (x_d-x_0) / norm(x_d-x_0);
        t_d=t_0+trajTime;

        if (prtclThrd->setupNewParticle(x_0,vel))
        {
            yInfo("[reactCtrlThread] got new target: x_0: %s",x_0.toString(3,3).c_str());
            yInfo("[reactCtrlThread]                 x_d: %s",x_d.toString(3,3).c_str());
            yInfo("[reactCtrlThread]                 vel: %s",vel.toString(3,3).c_str());
            state=STATE_REACH;

            return true;
        }
    }
    return false;
}

bool reactCtrlThread::setNewRelativeTarget(const Vector& _rel_x_d)
{
    if(_rel_x_d == Vector(3,0.0)) return false;

    Vector x_d = x_t + _rel_x_d;
    return setNewTarget(x_d);
}

void reactCtrlThread::updateArmChain()
{
    yarp::os::LockGuard lg(mutex);
    iencsA->getEncoders(encsA->data());
    Vector qA=encsA->subVector(0,6);

    if (useTorso)
    {
        iencsT->getEncoders(encsT->data());
        Vector qT(3,0.0);
        qT[0]=(*encsT)[2];
        qT[1]=(*encsT)[1];
        qT[2]=(*encsT)[0];

        Vector q(10,0.0);
        q.setSubvector(0,qT);
        q.setSubvector(3,qA);
        arm->setAng(q*CTRL_DEG2RAD);
    }
    else
    {
        arm->setAng(qA*CTRL_DEG2RAD);
    }

    H=arm->getH();
    x_t=H.subcol(0,3,3);
}

bool reactCtrlThread::alignJointsBounds()
{
    yDebug("[reactCtrlThread][alignJointsBounds] pre alignment:");
    printJointsBounds();

    deque<IControlLimits*> lim;
    lim.push_back(ilimT);
    lim.push_back(ilimA);

    if (!arm->alignJointsBounds(lim)) return false;

    // iCub::iKin::iKinChain chain=*arm->asChain();
    // chain(0).setMin(-22.0*CTRL_DEG2RAD);    chain(0).setMin(-84.0*CTRL_DEG2RAD);
    // chain(1).setMin(-39.0*CTRL_DEG2RAD);    chain(0).setMin(-39.0*CTRL_DEG2RAD);
    // chain(2).setMin(-59.0*CTRL_DEG2RAD);    chain(0).setMin(-59.0*CTRL_DEG2RAD);

    yDebug("[reactCtrlThread][alignJointsBounds] post alignment:");
    printJointsBounds();

    return true;
}

void reactCtrlThread::printJointsBounds()
{
    double min, max;
    iCub::iKin::iKinChain chain=*arm->asChain();

    for (int i = 0; i < arm->getDOF(); i++)
    {
        min=chain(i).getMin()*CTRL_RAD2DEG;
        max=chain(i).getMax()*CTRL_RAD2DEG;
        yDebug("[jointsBounds] i: %i\tmin: %g\tmax %g",i,min,max);
    }
}

bool reactCtrlThread::areJointsHealthyAndSet(VectorOf<int> &jointsToSet,
                                             const string &_p, const string &_s)
{
    VectorOf<int> modes;
    if (_p=="arm")
    {
        modes=encsA->size();
        imodA->getControlModes(modes.getFirst());
    }
    else if (_p=="torso")
    {
        modes=encsT->size();
        imodT->getControlModes(modes.getFirst());
    }
    else
        return false;

    for (size_t i=0; i<modes.size(); i++)
    {
        if ((modes[i]==VOCAB_CM_HW_FAULT) || (modes[i]==VOCAB_CM_IDLE))
            return false;

        if (_s=="velocity")
        {
            if (modes[i]!=VOCAB_CM_MIXED || modes[i]!=VOCAB_CM_VELOCITY)
                jointsToSet.push_back(i);
        }
        else if (_s=="position")
        {
            if (modes[i]!=VOCAB_CM_MIXED || modes[i]!=VOCAB_CM_POSITION)
                jointsToSet.push_back(i);
        }

    }

    return true;
}

bool reactCtrlThread::setCtrlModes(const VectorOf<int> &jointsToSet,
                                   const string &_p, const string &_s)
{
    if (_s!="position" && _s!="velocity")
        return false;

    if (jointsToSet.size()==0)
        return true;

    VectorOf<int> modes;
    for (size_t i=0; i<jointsToSet.size(); i++)
    {
        if (_s=="position")
        {
            modes.push_back(VOCAB_CM_POSITION);
        }
        else if (_s=="velocity")
        {
            modes.push_back(VOCAB_CM_VELOCITY);
        }
    }

    if (_p=="arm")
    {
        imodA->setControlModes(jointsToSet.size(),
                               jointsToSet.getFirst(),
                               modes.getFirst());
    }
    else if (_p=="torso")
    {
        imodT->setControlModes(jointsToSet.size(),
                               jointsToSet.getFirst(),
                               modes.getFirst());
    }
    else
        return false;

    return true;
}

int reactCtrlThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    else
        return -1;
}

void reactCtrlThread::threadRelease()
{
    yInfo("Returning to position mode..");
        delete encsA; encsA = NULL;
        delete encsT; encsT = NULL;
        delete   arm;   arm = NULL;

    yInfo("Closing ports..");
        outPort.close();

    yInfo("Closing controllers..");
        stopControl();
        ddA.close();
        ddT.close();
}

// empty line to make gcc happy
