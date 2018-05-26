/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
 * Public License for more details
*/

/** 
\defgroup icub_jointVelCtrlIdent jointVelCtrlIdent
 
Identification of the joint plant when controlled in velocity 
mode. 
 
\section intro_sec Description 
This module allows for system identification of the joint when 
controlled in velocity mode. Ideally, the plant should be 
represented by a pure integrator, but the behavior might deviate 
from the baseline due to the presence of poles and zero. To this 
end the joint is controlled with a \e chirp profile in velocity 
whose output is then sent to a yarp port for logging purpose. 
 
The joint is controlled in three different modalities that 
follow one another: 
- mode 0: the joint is subject to the chirp signal in velocity.
- mode 1: the joint is steered to reach for a target value 
 under the action of a minimum-jerk controller that does not
 account for plant dynamics.
- mode 2: the joint is steered to the target under the action 
 of a minimum-jerk compensator that accounts for the identified
 plant dynamics.
 
\section lib_sec Libraries 
- YARP.
- ctrlLib. 

\section parameters_sec Parameters
--name \e name
- specify the module stem-name. 
 
--robot \e robot
- select the robot to connect to.

--period \e Ts
- specify the controller period given in [ms].
 
--part \e part
- select the part to control. The special part "head_v2" is also
  available.
 
--joint \e joint
- select the joint to control.

The chirp signal is:

\f[
gain\cdot\sin\left(2\pi\left(f_{min}+\frac{f_{max}-f_{min}}{T}t\right)t\right),
\f]

therefore the following parameters can be assigned through
command-line options: <i>gain</i>, <i>fmin</i>, <i>fmax</i>, 
<i>T</i>. 
 
The model of the identified plant is:

\f[
P\left(s\right)=\frac{K_p}{s}\cdot\frac{1+T_zs}{1+2\zeta T_ws+\left(T_ws\right)^2},
\f]

therefore the following parameters can be assigned through 
command-line options: <i>Kp</i>, <i>Tz</i>, <i>Tw</i>, 
<i>Zeta</i>. 
 
Moreover, the reactivity of the minimum-jerk controllers can be 
tuned by changing their point-to-point trajectory time through 
the <i>Ttraj</i> command-line parameter. 
 
User might want also to tune the low-level PID controller, and 
to this purpose the following command-line options are 
available: <i>PID_Kp</i>, <i>PID_Ki</i>, <i>PID_Kd</i>. 
 
\section portsc_sec Ports Created 

\e /<modeName>:o streams out the following data: 
 
<cnt> <vel> <ref> <pwm_1> [... <pwm_n>] <fb> <dfb>
 
where <i>cnt</i> is a progress number accounting for the mode 
type such that mode=mod(cnt,3), <i>vel</i> is the commanded 
velocity in [deg/s], <i>ref</i> is the reference position as 
integrated by the board given in [deg], <i>pwm</i> is the pid 
output in [V] and <i>fb</i> represents the joint encoder 
feedback in [deg]. The last term <i>dfb</i> is the estimated 
derivative of the feedback. \n 
Importantly, the quantity <i>pwm</i> is retrieved only if the 
low level board broadcasts it out. 
 
\section tested_os_sec Tested OS
Windows, Linux
 
\author Ugo Pattacini
*/ 

#include <cstdio>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/***********************************************************/
class IdentThread : public RateThread
{
protected:
    string robot;
    string name;
    string part;
    int joint;

    int cnt;
    double gain;
    double fb0, fb1;
    double fmin, fmax;
    double t0, T, Ttraj;

    bool   pid_change;
    bool   pid_change_Kp;
    bool   pid_change_Ki;
    bool   pid_change_Kd;
    double pid_Kp;
    double pid_Ki;
    double pid_Kd;

    PolyDriver        drv;
    IControlMode2    *imod;
    IControlLimits   *ilim;
    IEncoders        *ienc;
    IPositionControl *ipos;
    IVelocityControl *ivel;
    IPidControl      *ipid;
    Pid               pid0;
    Pid               pid1;

    AWLinEstimator                  velEst;
    minJerkVelCtrlForIdealPlant    *ctrl1;
    minJerkVelCtrlForNonIdealPlant *ctrl2;
    Property plantParameters;

    BufferedPort<Bottle> port;

public:
    /***********************************************************/
    IdentThread() : RateThread(1000), velEst(16,1.0) { }

    /***********************************************************/
    void configure(ResourceFinder &rf)
    {
        name=rf.find("name").asString().c_str();
        setRate(rf.check("period",Value(10)).asInt());
        robot=rf.check("robot",Value("icub")).asString().c_str();
        part=rf.check("part",Value("right_arm")).asString().c_str();
        joint=rf.check("joint",Value(0)).asInt();
        T=rf.check("T",Value(10.0)).asDouble();
        gain=rf.check("gain",Value(40.0)).asDouble();
        fmin=rf.check("fmin",Value(0.01)).asDouble();
        fmax=rf.check("fmax",Value(1.0)).asDouble();

        Property props;
        Ttraj=rf.check("Ttraj",Value(1.0)).asDouble();
        props.put("Kp",rf.check("Kp",Value(1.0)).asDouble());
        props.put("Tz",rf.check("Tz",Value(0.0)).asDouble());
        props.put("Tw",rf.check("Tw",Value(0.0)).asDouble());
        props.put("Zeta",rf.check("Zeta",Value(0.0)).asDouble());
        string str;
        str="(dimension_0 (";
        str+=props.toString().c_str();
        str+="))";

        plantParameters.fromString(str.c_str());

        if (pid_change_Kp=rf.check("PID_Kp"))
            pid_Kp=rf.find("PID_Kp").asDouble();

        if (pid_change_Ki=rf.check("PID_Ki"))
            pid_Ki=rf.find("PID_Ki").asDouble();

        if (pid_change_Kd=rf.check("PID_Kd"))
            pid_Kd=rf.find("PID_Kd").asDouble();

        pid_change=pid_change_Kp||pid_change_Ki||pid_change_Kd;
    }

    /***********************************************************/
    bool threadInit()
    {
        string _part=part;
        if (_part=="head_v2")
            _part="head";

        Property options;
        options.put("device","remote_controlboard");
        options.put("remote",("/"+robot+"/"+_part).c_str());
        options.put("local",("/"+name+"/"+_part).c_str());

        if (!drv.open(options))
            return false;

        drv.view(imod);
        drv.view(ilim);
        drv.view(ienc);
        drv.view(ipos);
        drv.view(ivel);
        drv.view(ipid);

        if (pid_change)
        {
            ipid->getPid(VOCAB_PIDTYPE_VELOCITY,joint,&pid0);
            pid1=pid0;

            if (pid_change_Kp)
                pid1.setKp(pid_Kp);

            if (pid_change_Ki)
                pid1.setKi(pid_Ki);

            if (pid_change_Kd)
                pid1.setKd(pid_Kd);
            
            setPid(pid1);
        }

        ctrl1=new minJerkVelCtrlForIdealPlant(getRate()/1000.0,1);
        ctrl2=new minJerkVelCtrlForNonIdealPlant(getRate()/1000.0,1);
        ctrl2->setPlantParameters(plantParameters);

        Property props;
        ctrl2->getPlantParameters(props);
        printf("Plant Parameters = %s\n",props.toString().c_str());

        double min,max;
        ilim->getLimits(joint,&min,&max);
        double offs=0.75*gain;
        fb0=(min+max)/2.0-offs;
        fb1=(min+max)/2.0+offs;

        cnt=0;
        imod->setControlMode(joint,VOCAB_CM_VELOCITY);
        goToStartingPos();

        port.open(("/"+name+":o").c_str());
        Network::connect("/jointVelCtrlIdent:o","/data/jointVelCtrlIdent");
    
        t0=Time::now();

        return true;
    }

    /***********************************************************/
    void setPid(const Pid &pid)
    {
        if (((part=="head_v2") || (part=="right_arm") || (part=="left_arm")) &&
            ((joint==0) || (joint==1)))
        {
            ipid->setPid(VOCAB_PIDTYPE_VELOCITY,0,pid);
            ipid->setPid(VOCAB_PIDTYPE_VELOCITY,1,pid);
        }
        else if ((part=="torso") && ((joint==1) || (joint==2)))
        {
            ipid->setPid(VOCAB_PIDTYPE_VELOCITY,1,pid);
            ipid->setPid(VOCAB_PIDTYPE_VELOCITY,2,pid);
        }
        else
            ipid->setPid(VOCAB_PIDTYPE_VELOCITY,joint,pid1);
    }

    /***********************************************************/
    Vector getOutput()
    {
        Vector pwm(1);
        if (((part=="head_v2") || (part=="right_arm") || (part=="left_arm")) &&
            ((joint==0) || (joint==1)))
        {
            pwm.resize(2);
            ipid->getPidOutput(VOCAB_PIDTYPE_VELOCITY,0,&pwm[0]);
            ipid->getPidOutput(VOCAB_PIDTYPE_VELOCITY,1,&pwm[1]);
        }
        else if ((part=="torso") && ((joint==1) || (joint==2)))
        {
            pwm.resize(2);
            ipid->getPidOutput(VOCAB_PIDTYPE_VELOCITY,1,&pwm[0]);
            ipid->getPidOutput(VOCAB_PIDTYPE_VELOCITY,2,&pwm[1]);
        }
        else
            ipid->getPidOutput(VOCAB_PIDTYPE_VELOCITY,joint,&pwm[0]);

        return pwm;
    }

    /***********************************************************/
    void goToStartingPos()
    {
        ivel->stop(joint);
        imod->setControlMode(joint,VOCAB_CM_POSITION);
        ipos->setRefSpeed(joint,20.0);
        ipos->positionMove(joint,fb0);

        bool done=false;
        while (!done)
        {
            double fb;
            ienc->getEncoder(joint,&fb);
            done=fabs(fb0-fb)<0.5;
            Time::delay(0.1);
        }

        ipos->stop(joint);
        imod->setControlMode(joint,VOCAB_CM_VELOCITY);
        ivel->setRefAcceleration(joint,1e9);
        velEst.reset();
    }

    /***********************************************************/
    void run()
    {
        // Stamp for the setEnvelope for the ports
        yarp::os::Stamp ts;
        double t=Time::now()-t0;
        ts.update();
 
        if(t<=T){
            Bottle &out=port.prepare();
            out.clear();
           
            int mode= 0; //cnt%3;
            double fb;  ienc->getEncoder(joint,&fb);
            double ref; ipid->getPidReference(VOCAB_PIDTYPE_VELOCITY,joint,&ref);
            Vector pwm=getOutput();
            Vector vel(1), e(1); e[0]=fb1-fb;
        
            if (mode==0)
                vel[0]=gain*sin(2.0*M_PI*(fmin+((fmax-fmin)/T)*t)*t);
            else if (mode==1)
                vel=ctrl1->computeCmd(Ttraj,e);
            else
                vel=ctrl2->computeCmd(Ttraj,e);
    
            ivel->velocityMove(joint,vel[0]);

            AWPolyElement el;   
            el.data.resize(1,fb);
            el.time=Time::now();
            Vector dfb=velEst.estimate(el);

            out.addInt(cnt);
            out.addDouble(vel[0]);
            out.addDouble(ref);
            for (size_t i=0; i<pwm.length(); i++)
                out.addDouble(pwm[i]);
            out.addDouble(fb);
            out.addDouble(dfb[0]);
            
            port.setEnvelope(ts);
            port.write();
        }
        else
            this->threadRelease();
    }

    /***********************************************************/
    void threadRelease()
    {
        ivel->stop(joint);

        if (pid_change)
            setPid(pid0);

        delete ctrl1;
        delete ctrl2;
        drv.close();
        port.close();
    }
};


/***********************************************************/
class IdentModule: public RFModule
{
protected:
    IdentThread thr;

public:
    /***********************************************************/
    bool configure(ResourceFinder &rf)
    {
        thr.configure(rf);
        return thr.start();
    }

    /***********************************************************/
    bool close()
    {
        thr.stop();
        return true;
    }

    /***********************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /***********************************************************/
    bool updateModule()
    {
        return true;
    }
};


/***********************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server is unavailable!\n");
        return 1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","jointVelCtrlIdent");
    rf.configure(argc,argv);

    IdentModule mod;
    return mod.runModule(rf);
}


