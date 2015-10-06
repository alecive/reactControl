/* 
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
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

#include "reactCtrlThread.h"
#include <fstream>
#include <sstream>
#include <iomanip>

reactCtrlThread::reactCtrlThread(int _rate, const string &_name, const string &_robot,
                                 const string &_part, int _verbosity, bool _autoconnect) :
                                 RateThread(_rate), name(_name), robot(_robot), part(_part),
                                 verbosity(_verbosity), autoconnect(_autoconnect)
{
    step     = 0;

    // slv=NULL;
    // gue=NULL;
    // sol=NULL;
}

bool reactCtrlThread::threadInit()
{
    if (autoconnect)
    {
        yInfo("[reactController] Autoconnect flag set to ON");
        if (!Network::connect("/skinManager/skin_events:o",("/"+name+"/contacts:i").c_str()))
        {
            Network::connect("/virtualContactGeneration/virtualContacts:o",
                             ("/"+name+"/contacts:i").c_str());
        }
    }
    Network::connect(("/"+name+"/status:o").c_str(),"/visuoTactileRF/input:i");
    Network::connect(("/"+name+"/status:o").c_str(),"/visuoTactileWrapper/reactController:i");

    Property Opt;
    Opt.put("robot",  robot.c_str());
    Opt.put("part",   "right_arm");
    Opt.put("device", "remote_controlboard");
    Opt.put("remote",("/"+robot+"/right_arm").c_str());
    Opt.put("local", ("/"+name +"/right_arm").c_str());
    if (!dd.open(Opt))
    {
        yError("[reactController] Could not open right_arm PolyDriver!");
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
    encs = new Vector(jnts,0.0);

    if (!ok)
    {
        yError("[reactController] Problems acquiring either left_arm or right_arm interfaces!!!!");
        return false;
    }

    return true;
}

void reactCtrlThread::run()
{

}

void reactCtrlThread::solveIK()
{
    // slv->probl->limb.setH0(SE3inv(cntctH0));

    // slv->probl->limb.setAng(sol->joints);
    // slv->setInitialGuess(*sol);
    // slv->solve(*sol);
    // // sol->print();
    // solution = CTRL_RAD2DEG * sol->joints;

    // testLimb->setAng(sol->joints);
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
