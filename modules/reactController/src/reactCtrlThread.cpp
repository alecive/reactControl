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
#include "reactCtrlThread.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#include <IpIpoptApplication.hpp>

using namespace yarp::sig;
using namespace yarp::math;

reactCtrlThread::reactCtrlThread(int _rate, const string &_name, const string &_robot,
                                 const string &_part, int _verbosity, bool _autoconnect,
                                 double _trajTime, double _tol) :
                                 RateThread(_rate), name(_name), robot(_robot), part(_part),
                                 verbosity(_verbosity), autoconnect(_autoconnect),
                                 trajTime(_trajTime), tol(_tol)
{
    step = 0;

    if (part=="left_arm")
    {
        part_short="left";
    }
    else if (part=="right_arm")
    {
        part_short="right";
    }
    arm = new iCub::iKin::iCubArm(part_short.c_str());
    q_0.resize(arm->getDOF(),0.0);

    // Block torso links
    for (int i = 0; i < 3; i++)
    {
        arm->blockLink(i,0.0);
    }

    slv=NULL;
    isTask=false;
    x_d.resize(3,0.0);
    x_t.resize(3,0.0);
    x_0.resize(3,0.0);
    H.resize(4,4);
}

bool reactCtrlThread::threadInit()
{
    yarp::os::Property Opt;
    Opt.put("robot",  robot.c_str());
    Opt.put("part",   part.c_str());
    Opt.put("device", "remote_controlboard");
    Opt.put("remote",("/"+robot+"/"+part).c_str());
    Opt.put("local", ("/"+name +"/"+part).c_str());
    if (!dd.open(Opt))
    {
        yError("[reactController] Could not open %s PolyDriver!",part.c_str());
        return false;
    }

    bool ok = 1;
    
    if (dd.isValid())
    {
        ok = ok && dd.view(iencs);
        ok = ok && dd.view(ivel);
        ok = ok && dd.view(imod);
        ok = ok && dd.view(ilim);
    }
    iencs->getAxes(&jnts);
    encs = new yarp::sig::Vector(jnts,0.0);

    if (!ok)
    {
        yError("[reactController] Problems acquiring %s interfaces!!!!",part.c_str());
        return false;
    }
    isTask=true;

    yarp::os::Time::delay(0.2);

    return true;
}

void reactCtrlThread::run()
{
    updateArmChain();
    
    switch (step)
    {
        case 0:
        {
            Vector x_d(3,0.0);
            x_d    =x_t;
            x_d[2]+=0.2;
            setNewTarget(x_d);
            break;
        }
        case 1:
        {
            int exit_code;
            Vector q_dot = solveIK(&exit_code);

            if (exit_code==Ipopt::Solve_Succeeded)
            {
                q_0 = q_dot;
                controlArm(q_dot);
            }

            if (yarp::os::Time::now()>t_d)
            {
                step++;
            }
            break;
        }
        case 2:
            break;
        default:
            yError("[reactCtrlThread] reactCtrlThread should never be here!!! Step: %d",step);
            yarp::os::Time::delay(2.0);
            break;
    }
}

Vector reactCtrlThread::solveIK(int *_exit_code)
{
    slv=new reactIpOpt(*arm->asChain(),tol,100,verbosity,false);
    // Next step will be provided iteratively:
    double dT=getRate()/1000.0;
    int exit_code;
    
    // Vector x_next = x_t + (x_d - x_t)/norm(x_d -x_t) * dT ;

    // The equation is x_next = x_0 + (x_d - x_0) * (t/T)
    // 
    Vector x_next(3,0.0);
    x_next = x_0 + (x_d-x_0) * (dT/t_d);
    
    Vector result = slv->solve(x_next,q_0,dT,&exit_code) * CTRL_RAD2DEG;

    printMessage(0,"x_t:    %s\tdT %g\n",x_t.toString().c_str(),dT);
    printMessage(0,"x_next: %s\n",x_next.toString().c_str());
    printMessage(0,"norm(x_next-x_t): %g\tnorm(x_d-x_next): %g\n",norm(x_next-x_t),
                                                                  norm(x_d-x_next));
    printMessage(0,"Result: %s\n",result.toString().c_str());
    delete slv;

    return result;
}

bool reactCtrlThread::controlArm(const yarp::sig::Vector &_vels)
{   
    printf("asdfjiafoiaj\n");
    VectorOf<int> jointsToSet;
    if (!areJointsHealthyAndSet(jointsToSet,"velocity"))
    {
        stopControl();
        return false;
    }
    else
    {
        printf("asdfjiafoiaj\n");
        setCtrlModes(jointsToSet,"velocity");
    }

    ivel->velocityMove(_vels.data());

    return true;
}

bool reactCtrlThread::stopControl()
{
    return ivel->stop();
}

bool reactCtrlThread::setTol(const double _tol)
{
    return tol=_tol;
}

bool reactCtrlThread::setTrajTime(const double _traj_time)
{
    if (_traj_time>=0.0)
    {
        return trajTime=_traj_time;
    }
}

bool reactCtrlThread::setNewTarget(const Vector& _x_d)
{
    toggleTask(true);
    if (_x_d.size()==3)
    {
        q_0.resize(arm->getDOF(),0.0);
        x_0=x_t;
        x_d=_x_d;
        t_0=yarp::os::Time::now();
        t_d=t_0+trajTime;
        yInfo("[reactCtrlThread] got new target. x_0: %s",x_0.toString().c_str());
        yInfo("[reactCtrlThread]                 x_d: %s",x_d.toString().c_str());
        yInfo("[reactCtrlThread]                 t_d: %g",t_d);
        step = 1;

        return true;
    }
    return false;
}

void reactCtrlThread::updateArmChain()
{
    iencs->getEncoders(encs->data());
    Vector q=encs->subVector(0,9);
    // printMessage(0,"[update_arm_chain] Q: %s\n",q.toString().c_str());
    arm->setAng(q*CTRL_DEG2RAD);
    Vector qq=arm->getAng();
    // printMessage(0,"[update_arm_chain]. Q: %s\n",(qq*CTRL_RAD2DEG).toString().c_str());
    H=arm->getH();
    x_t=H.subcol(0,3,3);
}

bool reactCtrlThread::alignJointsBounds()
{
    // deque<IControlLimits*> lim;
    // lim.push_back(ilimS);
    // lim.push_back(ilimM);

    // if (testLimb->       alignJointsBounds(lim) == 0) return false;
    // if (slv->probl->limb.alignJointsBounds(lim) == 0) return false;

    // lim.pop_front();
    // if (slv->probl->index.alignJointsBounds(lim) == 0) return false;

    return true;
}

bool reactCtrlThread::areJointsHealthyAndSet(VectorOf<int> &jointsToSet,const string &_s)
{
    VectorOf<int> modes(encs->size());
    imod->getControlModes(modes.getFirst());

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

bool reactCtrlThread::setCtrlModes(const VectorOf<int> &jointsToSet,const string &_s)
{
    if (_s!="position" || _s!="velocity")
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

    imod->setControlModes(jointsToSet.size(),
                           jointsToSet.getFirst(),
                           modes.getFirst());

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
        delete encs; encs = NULL;
        delete  arm;  arm = NULL;

    yInfo("Closing ports..");

    yInfo("Closing controllers..");
        dd.close();

    yInfo("Closing solver..");
}

// empty line to make gcc happy
