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

using namespace yarp::sig;
using namespace yarp::math;

reactCtrlThread::reactCtrlThread(int _rate, const string &_name, const string &_robot,
                                 const string &_part, int _verbosity, bool _autoconnect, double _trajTime) :
                                 RateThread(_rate), name(_name), robot(_robot), part(_part),
                                 verbosity(_verbosity), autoconnect(_autoconnect), trajTime(_trajTime)
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
        ok = ok && dd.view(ipos);
        ok = ok && dd.view(imode);
        ok = ok && dd.view(iimp);
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

    return true;
}

void reactCtrlThread::run()
{
    yarp::os::Time::delay(1.0);
    updateArmChain();
    
    if (isTask)
    {
        Vector x_d(3,0.0);
        x_d    =x_t;
        x_d[2]+=0.2;

        setNewTarget(x_d);

        solveIK();
        isTask=false;
    }
}

Vector reactCtrlThread::solveIK()
{
    slv=new reactIpOpt(*arm->asChain(),1e-4,100,verbosity,false);
    // Next step will be provided iteratively:
    double dT=getRate()/1000.0;
    int exit_code;
    
    // Vector x_next = x_t + (x_d - x_t)/norm(x_d -x_t) * dT ;

    // The equation is x_next = x_0 + (x_d - x_0) * (t/T)
    // 
    Vector x_next(3,0.0);
    x_next = x_0 + (x_d-x_0) * (dT/t_d);
    
    Vector result = slv->solve(x_next,dT,&exit_code) * CTRL_RAD2DEG;

    printMessage(0,"x_t:    %s\tdT %g\n",x_t.toString().c_str(),dT);
    printMessage(0,"x_next: %s\tnorm(x_next-x_t): %g\n",x_next.toString().c_str(),
                                                      norm(x_next-x_t));
    printMessage(0,"Result: %s\n",result.toString().c_str());
    delete slv;

    return result;
}

bool reactCtrlThread::setNewTarget(const Vector& _x_d)
{
    toggleTask(true);
    if (_x_d.size()==3)
    {
        x_0=x_t;
        x_d=_x_d;
        t_0=yarp::os::Time::now();
        t_d=trajTime;
        yInfo("[reactCtrlThread] got new target. x_d: %s",x_d.toString().c_str());
        yInfo("[reactCtrlThread]                 x_0: %s",x_0.toString().c_str());
        yInfo("[reactCtrlThread]                 t_d: %g",t_d);

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
