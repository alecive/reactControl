/* 
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone <alessandro.roncone@yale.edu>
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

#include "particleThread.h"

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

particleThread::particleThread(int _rate, const string &_name, int _verbosity) :
                               PeriodicThread((double)_rate/1000.0), name(_name), verbosity(_verbosity)
{
    // Create the integrator
    integrator=new Integrator(_rate/1000.0,Vector(3,0.0));

    isRunning=false;
    vel.resize(3,0.0);
}

void particleThread::run()
{
    if (isRunning)
    {
        std::lock_guard<std::mutex> lg(mut);
        integrator->integrate(vel);
    }
}

bool particleThread::setupNewParticle(const yarp::sig::Vector &_x_0, const yarp::sig::Vector &_vel)
{
    std::lock_guard<std::mutex> lg(mut);
    if (_x_0.size()>=3 && _vel.size()>=3)
    {
        isRunning=true;
        vel=_vel;
        integrator->reset(_x_0);
        return true;
    }
    
    return false;
}

bool particleThread::resetParticle(const yarp::sig::Vector &_x_0)
{
    std::lock_guard<std::mutex> lg(mut);
    integrator->reset(_x_0);
    isRunning=false;
    return true;
}

bool particleThread::stopParticle()
{
    std::lock_guard<std::mutex> lg(mut);
    isRunning=false;
    return true;
}

yarp::sig::Vector particleThread::getParticle()
{
    std::lock_guard<std::mutex> lg(mut);
    return integrator->get();
}

int particleThread::printMessage(const int l, const char *f, ...) const
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

void particleThread::threadRelease()
{
    delete integrator; integrator = nullptr;
}

// empty line to make gcc happy
