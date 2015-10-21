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

#include "particleThread.h"

using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

particleThread::particleThread(int _rate, const string &_name, int _verbosity) :
                               RateThread(_rate), name(_name), verbosity(_verbosity)
{
    // Create the integrator
    integrator=new Integrator(_rate/1000.0,Vector(3,0.0));

    isRunning=false;
}

bool particleThread::threadInit()
{
    return true;
}

void particleThread::run()
{

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
}

// empty line to make gcc happy
