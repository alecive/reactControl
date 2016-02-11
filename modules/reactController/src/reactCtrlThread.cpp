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

#include <fstream>
#include <sstream>
#include <iomanip>

#include <IpIpoptApplication.hpp>

#include <yarp/os/Time.h>
#include <iCub/ctrl/math.h>

#include "reactCtrlThread.h"


using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;

#define TACTILE_INPUT_GAIN 1.0
#define VISUAL_INPUT_GAIN 0.5

#define STATE_WAIT              0
#define STATE_REACH             1
#define STATE_IDLE              2

#define NR_ARM_JOINTS 7
#define NR_TORSO_JOINTS 3 

/*********** public methods ****************************************************************************/ 

reactCtrlThread::reactCtrlThread(int _rate, const string &_name, const string &_robot,  const string &_part,
                                 int _verbosity, bool _disableTorso,  double _trajSpeed, double _globalTol, 
                                 double _vMax, double _tol, bool _tactileCollisionPointsOn, bool _visualCollisionPointsOn,
                                 bool _visualizeTargetInSim, bool _visualizeParticleInSim,
                                 bool _visualizeCollisionPointsInSim,particleThread *_pT) :
                                 RateThread(_rate), name(_name), robot(_robot), part(_part),
                                 verbosity(_verbosity), useTorso(!_disableTorso),
                                 trajSpeed(_trajSpeed), globalTol(_globalTol), vMax(_vMax), tol(_tol),
                                 tactileCollisionPointsOn(_tactileCollisionPointsOn), visualCollisionPointsOn(_visualCollisionPointsOn),
                                 visualizeTargetInSim(_visualizeTargetInSim), visualizeParticleInSim(_visualizeParticleInSim),
                                 visualizeCollisionPointsInSim(_visualizeCollisionPointsInSim)
{
   prtclThrd=_pT;
}

bool reactCtrlThread::threadInit()
{
   
    printMessage(5,"[reactCtrlThread] threadInit()\n");
    state=STATE_WAIT;

    if (part=="left_arm")
    {
        part_short="left";
    }
    else if (part=="right_arm")
    {
        part_short="right";
    }
    
   /******** iKin chain and variables, and transforms init *************************/
   
    arm = new iCub::iKin::iCubArm(part_short.c_str());
    // Release / block torso links (blocked by default)
    for (int i = 0; i < NR_TORSO_JOINTS; i++)
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

    //we set up the variables based on the current DOF - that is without torso joints if torso is blocked
    chainActiveDOF = arm->getDOF();
 
    //N.B. All angles in this thread are in degrees
    qA.resize(NR_ARM_JOINTS,0.0); //current values of arm joints (should be 7)
    if (useTorso)
        qT.resize(NR_TORSO_JOINTS,0.0); //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)
    q.resize(chainActiveDOF,0.0); //current joint angle values (10 if torso is on, 7 if off)
   
    q_dot_0.resize(chainActiveDOF,0.0);
    vLimNominal.resize(chainActiveDOF,2);
    vLimAdapted.resize(chainActiveDOF,2);
    for (size_t r=0; r<chainActiveDOF; r++)
    {
        vLimNominal(r,0)=-vMax;
        vLimAdapted(r,0)=-vMax;
        vLimNominal(r,1)=vMax;
        vLimAdapted(r,1)=vMax;
    }
    //optionally: if (useTorso) vLimNominal(1,0)=vLimNominal(1,1)=0.0;  // disable torso roll
         
    H.resize(4,4);

    T_world_root = zeros(4,4); 
    T_world_root(0,1)=-1;
    T_world_root(1,2)=1; T_world_root(1,3)=0.5976;
    T_world_root(2,0)=-1; T_world_root(2,3)=-0.026;
    T_world_root(3,3)=1;
    //iT_world_root=SE3inv(T_world_root);
    
    /*****  Drivers, interfaces, control boards etc. ***********************************************************/
    
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
   
    /************ variables related to the optimization problem for ipopt *******/
   
    slv=NULL;
    ipoptBoundSmoothnessOn = true;
    x_0.resize(3,0.0);
    x_t.resize(3,0.0);
    x_n.resize(3,0.0);
    x_d.resize(3,0.0);
  
    /***************** ports and files*************************************************************************************/
    
    aggregPPSeventsInPort.open("/"+name+"/pps_events_aggreg:i");
    aggregSkinEventsInPort.open("/"+name+"/skin_events_aggreg:i");
    
    outPort.open("/"+name +"/data:o"); //for dumping
    
    fout_param.open("param.log");
    
    /***** writing to param file ******************************************************/
    iCub::iKin::iKinChain chainTemp = *(arm->asChain());
    fout_param<<chainActiveDOF<<" ";
    for (size_t i=0; i<chainActiveDOF; i++)
    {
        fout_param<<CTRL_RAD2DEG*(chainTemp(i).getMin())<<" ";
        fout_param<<CTRL_RAD2DEG*(chainTemp(i).getMax())<<" ";
    }
    for (size_t j=0; j<chainActiveDOF; j++)
    {
        fout_param<<vLimNominal(j,0)<<" ";
        fout_param<<vLimNominal(j,1)<<" ";
    }
    yInfo("Written to param file and closing..");    
    fout_param.close();
    
    /**** visualizing targets and collision points in simulator ***************************/
    
    if((robot == "icubSim") && (visualizeTargetInSim || visualizeParticleInSim || visualizeCollisionPointsInSim) ){ 
        string port2icubsim = "/" + name + "/sim:o";
        if (!portToSimWorld.open(port2icubsim.c_str())) {
            yError("[reactCtrlThread] Unable to open port << port2icubsim << endl");
        }    
        std::string port2world = "/icubSim/world";
        yarp::os::Network::connect(port2icubsim, port2world.c_str());
    
        cmd.clear();
        cmd.addString("world");
        cmd.addString("del");
        cmd.addString("all");
        portToSimWorld.write(cmd);
        
        collisionPointsVisualizedCount = 0;
        collisionPointsSimReservoirPos.zero();
        collisionPointsSimReservoirPos.resize(3);
        collisionPointsSimReservoirPos(0)=0.3;
        collisionPointsSimReservoirPos(1)=0.03;
        collisionPointsSimReservoirPos(2)=0.0;
    
    }    
    firstTarget = true;
    printMessage(5,"[reactCtrlThread] threadInit() finished.\n");
    yarp::os::Time::delay(0.2);

    return true;
}

void reactCtrlThread::run()
{
    printMessage(5,"[reactCtrlThread::run()] started, state: %d.\n",state);
    yarp::os::LockGuard lg(mutex);
    updateArmChain();
    printMessage(10,"[reactCtrlThread::run()] updated arm chain.\n");
    //debug - see Jacobian
    //iCub::iKin::iKinChain &chain_temp=*arm->asChain();
    //yarp::sig::Matrix J1_temp=chain_temp.GeoJacobian();
    //yDebug("GeoJacobian: \n %s \n",J1_temp.toString(3,3).c_str());    
        
    collisionPoints.clear();
    
    /* For now, let's experiment with some fixed points on the forearm skin, emulating the vectors coming from margin of safety, 
     to test the performance of the algorithm
    Let's try 3 triangle midpoints on the upper patch on the forearm, taking positions from CAD, 
    normals from /home/matej/programming/icub-main/app/skinGui/conf/positions/left_forearm_mesh.txt
    All in wrist FoR (nr. 8), see  /media/Data/my_matlab/skin/left_forearm/selected_taxels_upper_patch_for_testing.txt

    Upper patch, left forearm
    1) central, distal triangle - ID 291, row 4 in triangle_centers_CAD_upperPatch_wristFoR8
    ID  x   y   z   n1  n2  n3
    291 -0.0002 -0.0131 -0.0258434  -0.005  0.238   -0.971
    2) left, outermost, proximal triangle - ID 255, row 7 in triangle_centers_CAD_upperPatch_wristFoR8
    ID  x   y   z   n1  n2  n3
    255 0.026828    -0.054786   -0.0191051  0.883   0.15    -0.385
    3) right, outermost, proximal triangle - ID 207, row 1 in triangle_centers_CAD_upperPatch_wristFoR8
    ID  x   y   z   n1  n2  n3
    207 -0.027228   -0.054786   -0.0191051  -0.886  0.14    -0.431 */
    
    /*collisionPoint_t collisionPointStruct;
    collisionPointStruct.skin_part = SKIN_LEFT_FOREARM;
    collisionPointStruct.x.resize(3,0.0);
    collisionPointStruct.n.resize(3,0.0);
    collisionPointStruct.x(0) = -0.0002;  collisionPointStruct.x(1) = -0.0131; collisionPointStruct.x(2) = -0.0258434;
    collisionPointStruct.n(0) = -0.005; collisionPointStruct.n(1) = 0.238; collisionPointStruct.n(2) = -0.971;
    collisionPointStruct.magnitude = 0.1; //~ "probability of collision" */
    
    if (tactileCollisionPointsOn){
        printMessage(9,"[reactCtrlThread::run()] Getting tactile collisions from port.\n");
        getCollisionPointsFromPort(aggregSkinEventsInPort, TACTILE_INPUT_GAIN, part_short,collisionPoints);
    }
    if (visualCollisionPointsOn){ //note, these are not mutually exclusive - they can co-exist
        printMessage(9,"[reactCtrlThread::run()] Getting visual collisions from port.\n");
        getCollisionPointsFromPort(aggregPPSeventsInPort, VISUAL_INPUT_GAIN, part_short,collisionPoints);
    }
    //after this point, we don't care where did the collision points come from - our relative confidence in the two modalities is expressed in the gains
    
    if (visualizeCollisionPointsInSim){
        printMessage(5,"[reactCtrlThread::run()] will visualize collision points in simulator.\n");
        showCollisionPointsInSim();
    }
    
    switch (state)
    {
        case STATE_WAIT:
            break;
        case STATE_REACH:
        {
            if (norm(x_t-x_d) < globalTol) //we keep solving until we reach the desired target
            {
                yDebug(0,"[reactCtrlThread] norm(x_t-x_d) %g\tglobalTol %g\n",norm(x_t-x_d),globalTol);
                if (!stopControlHelper())
                    yError("[reactCtrlThread] Unable to properly stop the control of the arm!");
                break;
            }

            int exit_code;
            q_dot = solveIK(exit_code);
            
            if (exit_code==Ipopt::Solve_Succeeded || exit_code==Ipopt::Maximum_CpuTime_Exceeded)
            {
                if (exit_code==Ipopt::Maximum_CpuTime_Exceeded)
                    yWarning("[reactCtrlThread] Ipopt cpu time was higher than the rate of the thread!");
                
                if (!controlArm(q_dot))
                    yError("I am not able to properly control the arm!");
            }

            break;
        }
        case STATE_IDLE:
        {
            yInfo("[reactCtrlThread] finished.");
            state=STATE_WAIT;
            break;
        }
        default:
            yFatal("[reactCtrlThread] reactCtrlThread should never be here!!! Step: %d",state);
    }

    sendData();
}

void reactCtrlThread::threadRelease()
{
    yInfo("Returning to position mode..");
        delete encsA; encsA = NULL;
        delete encsT; encsT = NULL;
        delete   arm;   arm = NULL;

    collisionPoints.clear();    
    
    if((robot == "icubSim") && (visualizeTargetInSim || visualizeParticleInSim || visualizeCollisionPointsInSim) ){ 
        yInfo("Deleting objects from simulator world.");
        cmd.clear();
        cmd.addString("world");
        cmd.addString("del");
        cmd.addString("all");
        portToSimWorld.write(cmd);
    }  
    
    yInfo("Closing ports..");
        aggregPPSeventsInPort.interrupt();
        aggregPPSeventsInPort.close();
        aggregSkinEventsInPort.interrupt();
        aggregSkinEventsInPort.close();
        outPort.interrupt();
        outPort.close();
        if (portToSimWorld.isOpen()){
            portToSimWorld.interrupt();
            portToSimWorld.close();
        }
   // yInfo("Closing files..");    
     //   fout_param.close();
    yInfo("Closing controllers..");
        stopControl();
        ddA.close();
        ddT.close();
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
        tol=_tol;
        return true;
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
        vMax=_vMax;
        return true;
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
}

bool reactCtrlThread::setTrajSpeed(const double _traj_speed)
{
    if (_traj_speed>=0.0)
    {
        trajSpeed=_traj_speed;
        return true;
    }
    return false;
}

bool reactCtrlThread::setVerbosity(const int _verbosity)
{
    if (_verbosity>=0)
        verbosity=_verbosity;
    else
        verbosity=0;
    return true;
}

bool reactCtrlThread::setNewTarget(const Vector& _x_d)
{
    if (_x_d.size()==3)
    {
        q_dot_0.resize(chainActiveDOF,0.0);
        q_dot.resize(chainActiveDOF,0.0);
        x_0=x_t;
        x_n=x_0;
        x_d=_x_d;
        
        if(visualizeTargetInSim){
            Vector x_d_sim(3,0.0);
            convertPosFromRootToSimFoR(x_d,x_d_sim);
            if (firstTarget){
                createStaticSphere(0.03,x_d_sim);
            }
            else{
                moveSphere(1,x_d_sim);
            }
        }
        
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

            if(visualizeParticleInSim){
                    Vector x_0_sim(3,0.0);
                    convertPosFromRootToSimFoR(x_0,x_0_sim);
                    if (firstTarget){
                        createStaticSphere(0.02,x_0_sim);
                    }
                    else{
                       moveSphere(2,x_0_sim); //sphere created as second will keep the index 2  
                    }
            }
            
           
        }
        else{
            yWarning("prtclThrd->setupNewParticle(x_0,vel) returned false.\n");
            return false;
        }
        
        if (firstTarget){
            firstTarget = false;
        }
    }
    return true;
}

bool reactCtrlThread::setNewRelativeTarget(const Vector& _rel_x_d)
{
    if(_rel_x_d == Vector(3,0.0)) return false;

    Vector x_d = x_t + _rel_x_d;
    return setNewTarget(x_d);
}


bool reactCtrlThread::stopControl()
{
    yarp::os::LockGuard lg(mutex);
    return stopControlHelper();
}


//************** protected methods *******************************/


Vector reactCtrlThread::solveIK(int &_exit_code)
{
    slv=new reactIpOpt(*arm->asChain(),tol,verbosity);
    // Next step will be provided iteratively.
    // The equation is x(t_next) = x_t + (x_d - x_t) * (t_next - t_now/T-t_now)
    //                              s.t. t_next = t_now + dT
    double dT=getRate()/1000.0;
    double t_t=yarp::os::Time::now();
    int    exit_code=-1;

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
    if (norm(x_n-x_0) > norm(x_d-x_0)) //if the particle is farther than the final target, we reset the particle - it will stay with the target
    {
        prtclThrd->resetParticle(x_d);
    }

    x_n=prtclThrd->getParticle(); //to get next target 
 
    if(visualizeParticleInSim){
        Vector x_n_sim(3,0.0);
        convertPosFromRootToSimFoR(x_n,x_n_sim);
        moveSphere(2,x_n_sim); //sphere created as second (particle) will keep the index 2  
    }
    
    AvoidanceHandlerAbstract *avhdl; 
    avhdl = new AvoidanceHandlerTactile(*arm->asChain(),collisionPoints,verbosity);
    vLimAdapted=avhdl->getVLIM(vLimNominal);
    printMessage(2,"calling ipopt with the following joint velocity limits (deg): \n %s \n",vLimAdapted.toString(3,3).c_str());
    //printf("calling ipopt with the following joint velocity limits (rad): \n %s \n",(vLimAdapted*CTRL_DEG2RAD).toString(3,3).c_str());
    // Remember: at this stage everything is kept in degrees because the robot is controlled in degrees.
    // At the ipopt level it comes handy to translate everything in radians because iKin works in radians.
    // So, q_dot_0 is in degrees, but I have to convert it in radians before sending it to ipopt
    Vector res=slv->solve(x_n,q_dot_0*CTRL_DEG2RAD,dT,vLimAdapted*CTRL_DEG2RAD,ipoptBoundSmoothnessOn,&exit_code)*CTRL_RAD2DEG;

    // printMessage(0,"t_d: %g\tt_t: %g\n",t_d-t_0, t_t-t_0);
    printMessage(0,"x_n: %s\tx_d: %s\tdT %g\n",x_n.toString(3,3).c_str(),x_d.toString(3,3).c_str(),dT);
    printMessage(0,"x_0: %s\tx_t: %s\n",       x_0.toString(3,3).c_str(),x_t.toString(3,3).c_str());
    printMessage(0,"norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g\n",
                    norm(x_n-x_t), norm(x_d-x_n), norm(x_d-x_t));
    printMessage(0,"Result (solved velocities (deg/s)): %s\n",res.toString(3,3).c_str());
    _exit_code=exit_code;
    q_dot_0=res;  //result at this step will be prepared as q_dot_0 for the next iteration of the solver

    delete slv;
    return res;
}

bool reactCtrlThread::controlArm(const yarp::sig::Vector &_vels)
{   
    VectorOf<int> jointsToSetA;
    VectorOf<int> jointsToSetT;
    if (!areJointsHealthyAndSet(jointsToSetA,"arm","velocity"))
    {
        yWarning("[reactCtrlThread::controlArm] Stopping control because arm joints are not healthy!");
        stopControlHelper();
        return false;
    }
 
    if (useTorso)
    {
        if (!areJointsHealthyAndSet(jointsToSetT,"torso","velocity"))
        {
            yWarning("[reactCtrlThread::controlArm] Stopping control because torso joints are not healthy!");
            stopControlHelper();
            return false;
        }
    }
    
    if (!setCtrlModes(jointsToSetA,"arm","velocity"))
    {
        yError("[reactCtrlThread::controlArm] I am not able to set the arm joints to velocity mode!");
        return false;
    }   

    if (useTorso)
    {
        if (!setCtrlModes(jointsToSetT,"torso","velocity"))
        {
            yError("[reactCtrlThread::controlArm] I am not able to set the torso joints to velocity mode!");
            return false;
        }
    }
    if(verbosity>=10){
        printf("[reactCtrlThread::controlArm] setting following arm joints to %s: ",Vocab::decode(VOCAB_CM_VELOCITY).c_str());
        for (size_t k=0; k<jointsToSetA.size(); k++){
                printf("%d ",jointsToSetA[k]);
        }
        printf("\n");
        if(useTorso){
            printf("[reactCtrlThread::controlArm] setting following torso joints to %s: ",Vocab::decode(VOCAB_CM_VELOCITY).c_str());
            for (size_t l=0; l<jointsToSetT.size(); l++){
                printf("%d ",jointsToSetT[l]);
            }
            printf("\n");       
        }
    }
    printMessage(1,"[reactCtrlThread::controlArm] Joint velocities (iKin order, deg/s): %s\n",_vels.toString(3,3).c_str());
    if (useTorso)
    {
        Vector velsT(3,0.0);
        velsT[0] = _vels[2]; //swapping pitch and yaw as per iKin vs. motor interface convention
        velsT[1] = _vels[1];
        velsT[2] = _vels[0]; //swapping pitch and yaw as per iKin vs. motor interface convention
       
        printMessage(2,"    velocityMove(): torso (swap pitch & yaw): %s\n",velsT.toString(3,3).c_str());
        ivelT->velocityMove(velsT.data());
        ivelA->velocityMove(_vels.subVector(3,9).data()); //indexes 3 to 9 are the arm joints velocities
        
    }
    else
    {
        ivelA->velocityMove(_vels.data()); //if there is not torso, _vels has only the 7 arm joints
    }

    return true;
}

Vector reactCtrlThread::computeDeltaX()
{
    iCub::iKin::iKinChain &chain=*arm->asChain();
    yarp::sig::Matrix J1=chain.GeoJacobian();
    yarp::sig::Matrix J_cst;
    J_cst.resize(3,chainActiveDOF);
    J_cst.zero();
    submatrix(J1,J_cst,0,2,0,chainActiveDOF-1);
    double dT=getRate()/1000.0;

    return dT*J_cst*q_dot;
}

void reactCtrlThread::sendData()
{
    ts.update();
    printMessage(5,"[reactCtrlThread::sendData()]\n");
    if (outPort.getOutputCount()>0)
    {
        if (state==STATE_REACH)
        {
            yarp::os::Bottle b;
            b.clear();

            //col 1
            b.addInt(chainActiveDOF);
            //cols 2-4: the desired final target (for end-effector)
            vectorIntoBottle(x_d,b);
            // 5:7 the end effector position in which the robot currently is
            vectorIntoBottle(x_t,b);
            // 8:10 the current desired target given by the particle (for end-effector)
            vectorIntoBottle(x_n,b);
            //variable - if torso on: 11:20: joint velocities as solution to control and sent to robot 
            vectorIntoBottle(q_dot,b); 
            //variable - if torso on: 21:30: joint positions as solution from ipopt and sent to robot 
            vectorIntoBottle(q,b); 
            //variable - if torso on: 31:50; assuming it is row by row, so min_1, max_1, min_2, max_2 etc.
            matrixIntoBottle(vLimAdapted,b);
            
            // the delta_x, that is the 3D vector that ipopt commands to 
            //    the robot in order for x_t to reach x_n
            //yarp::os::Bottle &b_delta_x=out.addList();
            //iCub::skinDynLib::vectorIntoBottle(computeDeltaX(),b_delta_x);
                         
            outPort.setEnvelope(ts);
            outPort.write(b);
        }
    }
}

bool reactCtrlThread::stopControlHelper()
{
    state=STATE_IDLE;
    if (useTorso)
    {
        return ivelA->stop() && ivelT->stop();
    }

    return ivelA->stop();
}

void reactCtrlThread::updateArmChain()
{    
    iencsA->getEncoders(encsA->data());
    qA=encsA->subVector(0,NR_ARM_JOINTS-1);

    if (useTorso)
    {
        iencsT->getEncoders(encsT->data());
        qT[0]=(*encsT)[2];
        qT[1]=(*encsT)[1];
        qT[2]=(*encsT)[0];

        q.setSubvector(0,qT);
        q.setSubvector(NR_TORSO_JOINTS,qA);
    }
    else
    {
        q = qA;        
    }
    arm->setAng(q*CTRL_DEG2RAD);
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

    // iCub::iKin::iKinChain &chain=*arm->asChain();
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
    iCub::iKin::iKinChain &chain=*arm->asChain();

    for (size_t i = 0; i < chainActiveDOF; i++)
    {
        min=chain(i).getMin()*CTRL_RAD2DEG;
        max=chain(i).getMax()*CTRL_RAD2DEG;
        yDebug("[jointsBounds (deg)] i: %i\tmin: %g\tmax %g",i,min,max);
    }
}

bool reactCtrlThread::areJointsHealthyAndSet(VectorOf<int> &jointsToSet,
                                             const string &_p, const string &_s)
{
    jointsToSet.clear();
    VectorOf<int> modes;
    if (_p=="arm")
    {
        modes.resize(NR_ARM_JOINTS,VOCAB_CM_IDLE);
        imodA->getControlModes(modes.getFirst());
    }
    else if (_p=="torso")
    {
        modes.resize(NR_TORSO_JOINTS,VOCAB_CM_IDLE);
        imodT->getControlModes(modes.getFirst());
    }
    else
        return false;
    
    for (size_t i=0; i<modes.size(); i++) //TODO in addition, one might check if some joints are blocked like here:  ServerCartesianController::areJointsHealthyAndSet
    {
        if ((modes[i]==VOCAB_CM_HW_FAULT) || (modes[i]==VOCAB_CM_IDLE))
            return false;

        if (_s=="velocity")
        {
            if ((modes[i]!=VOCAB_CM_MIXED) && (modes[i]!=VOCAB_CM_VELOCITY)){ // we will set only those that are not in correct modes already
                //printMessage(3,"    joint %d in %s mode, pushing to jointsToSet \n",i,Vocab::decode(modes[i]).c_str());
                jointsToSet.push_back(i);
            }
        }
        else if (_s=="position")
        {
            if ((modes[i]!=VOCAB_CM_MIXED) && (modes[i]!=VOCAB_CM_POSITION))
                jointsToSet.push_back(i);
        }

    }
    if(verbosity >= 10){
        printf("[reactCtrlThread::areJointsHealthyAndSet] %s: ctrl Modes retreived: ",_p.c_str());
        for (size_t j=0; j<modes.size(); j++){
                printf("%s ",Vocab::decode(modes[j]).c_str());
        }
        printf("\n");
        printf("Indexes of joints to set: ");
        for (size_t k=0; k<jointsToSet.size(); k++){
                printf("%d ",jointsToSet[k]);
        }
        printf("\n");
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


void reactCtrlThread::convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos)
{
    Vector pos_temp = pos;
    pos_temp.resize(4); 
    pos_temp(3) = 1.0;
     
    //printf("convertPosFromRootToSimFoR: need to convert %s in icub root FoR to simulator FoR.\n",pos.toString().c_str());
    //printf("convertPosFromRootToSimFoR: pos in icub root resized to 4, with last value set to 1:%s\n",pos_temp.toString().c_str());
    
    outPos.resize(4,0.0); 
    outPos = T_world_root * pos_temp;
    //printf("convertPosFromRootToSimFoR: outPos in simulator FoR:%s\n",outPos.toString().c_str());
    outPos.resize(3); 
    //printf("convertPosFromRootToSimFoR: outPos after resizing back to 3 values:%s\n",outPos.toString().c_str());
    return;
}

void reactCtrlThread::convertPosFromLinkToRootFoR(const Vector &pos,const SkinPart skinPart, Vector &outPos)
{
    Matrix T_root_to_link = yarp::math::zeros(4,4);
    int torsoDOF = 0;
    if (useTorso){
        torsoDOF = 3;
    }

     T_root_to_link = arm->getH(SkinPart_2_LinkNum[skinPart].linkNum + torsoDOF);
     //e.g. skinPart LEFT_UPPER_ARM gives link number 2, which means we ask iKin for getH(2+3), which gives us  FoR 6 - at the first elbow joint, which is the FoR for the upper arm 
     
    Vector pos_temp = pos;
    pos_temp.resize(4); 
    pos_temp(3) = 1.0;
    //printf("convertPosFromLinkToRootFoR: need to convert %s in the %dth link FoR, skin part %s into iCub root FoR.\n",pos.toString().c_str(),SkinPart_2_LinkNum[skinPart].linkNum,SkinPart_s[skinPart].c_str());
    //printf("convertPosFromRootToSimFoR: pos in icub root resized to 4, with last value set to 1:%s\n",pos_temp.toString().c_str());
    
    outPos.resize(4,0.0);
    outPos = T_root_to_link * pos_temp;
    outPos.resize(3);
    //printf("convertPosFromLinkToRootFoR: outPos after resizing back to 3 values:%s\n",outPos.toString().c_str());
    
    return;   
 
}



bool reactCtrlThread::getCollisionPointsFromPort(BufferedPort<Bottle> &inPort, double gain, string which_chain,std::vector<collisionPoint_t> &collPoints)
{
    printMessage(9,"[reactCtrlThread::getCollisionPointsFromPort].\n");
    collisionPoint_t collPoint;    
    SkinPart sp = SKIN_PART_UNKNOWN;
    
    collPoint.skin_part = SKIN_PART_UNKNOWN;
    collPoint.x.resize(3,0.0);
    collPoint.n.resize(3,0.0);
    collPoint.magnitude=0.0;
    
    Bottle* collPointsMultiBottle = inPort.read(false);
    if(collPointsMultiBottle != NULL){
         printMessage(5,"[reactCtrlThread::getCollisionPointsFromPort]: There were %d bottles on the port.\n",collPointsMultiBottle->size());
         for(int i=0; i< collPointsMultiBottle->size();i++){
             Bottle* collPointBottle = collPointsMultiBottle->get(i).asList();
             printMessage(5,"Bottle %d contains %s", i,collPointBottle->toString().c_str());
             sp =  (SkinPart)(collPointBottle->get(0).asInt());
             //we take only those collision points that are relevant for the chain we are controlling
             if( ((which_chain == "left") && ( (sp==SKIN_LEFT_HAND) || (sp==SKIN_LEFT_FOREARM) || (sp==SKIN_LEFT_UPPER_ARM) ) )
              || ((which_chain == "right") && ( (sp==SKIN_RIGHT_HAND) || (sp==SKIN_RIGHT_FOREARM) || (sp==SKIN_RIGHT_UPPER_ARM) ) ) ){ 
                collPoint.skin_part = sp;
                collPoint.x(0) = collPointBottle->get(1).asDouble();
                collPoint.x(1) = collPointBottle->get(2).asDouble();
                collPoint.x(2) = collPointBottle->get(3).asDouble();
                collPoint.n(0) = collPointBottle->get(4).asDouble();
                collPoint.n(1) = collPointBottle->get(5).asDouble();
                collPoint.n(2) = collPointBottle->get(6).asDouble();
                collPoint.magnitude = collPointBottle->get(7).asDouble() * gain;
                collPoints.push_back(collPoint);
             }
         }
        return true;
    }
    else{
       printMessage(9,"[reactCtrlThread::getCollisionPointsFromPort]: no avoidance vectors on the port.\n") ;
       return false;
    };   
}
    


void reactCtrlThread::createStaticSphere(double radius, const Vector &pos)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("ssph");
    cmd.addDouble(radius);
    
    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    // color
    cmd.addInt(1);cmd.addInt(0);cmd.addInt(0);
    cmd.addString("false"); //no collisions
    printMessage(5,"createSphere(): sending %s \n",cmd.toString().c_str());
    portToSimWorld.write(cmd);
}

void reactCtrlThread::moveSphere(int index, const Vector &pos)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("ssph");
    cmd.addInt(index);
    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    portToSimWorld.write(cmd);
}

void reactCtrlThread::createStaticBox(const Vector &pos)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addDouble(0.02); cmd.addDouble(0.02); cmd.addDouble(0.02); //fixed size
    
    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    // color 
    cmd.addInt(0);cmd.addInt(0);cmd.addInt(1); //blue
    cmd.addString("false"); //no collisions
    printMessage(5,"createBox(): sending %s \n",cmd.toString().c_str());
    portToSimWorld.write(cmd);
}

void reactCtrlThread::moveBox(int index, const Vector &pos)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("sbox");
    cmd.addInt(index);
    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    portToSimWorld.write(cmd);
}

void reactCtrlThread::showCollisionPointsInSim()
{
    int nrCollisionPoints = collisionPoints.size();
    Vector pos(3,0.0);  
    if (nrCollisionPoints > collisionPointsVisualizedCount){
        for(int i=1; i<= (nrCollisionPoints - collisionPointsVisualizedCount);i++){
            pos = collisionPointsSimReservoirPos; 
            pos(2)=pos(2)+0.03*i;
            printMessage(5,"There are more collision points, %d, than available boxes in sim, %d, adding one at %s\n",nrCollisionPoints,collisionPointsVisualizedCount,pos.toString(3,3).c_str());
            createStaticBox(pos);   
            collisionPointsVisualizedCount++;
        }
        
    }
    
    int j=1;
    Vector posRoot(3,0.0);
    Vector posSim(3,0.0);
    for(std::vector<collisionPoint_t>::const_iterator it = collisionPoints.begin(); it != collisionPoints.end(); ++it) {
        convertPosFromLinkToRootFoR(it->x,it->skin_part,posRoot);
        convertPosFromRootToSimFoR(posRoot,posSim);
        moveBox(j,posSim); //just move a box from the sim world
        j++;
        posRoot.zero(); posSim.zero();
    }
    
    //if there have been more boxes allocated, just move them to the reservoir in the world
    //(icubSim does not support deleting individual objects)
        
    if (nrCollisionPoints < collisionPointsVisualizedCount){
        for(int k=collisionPointsVisualizedCount; k> nrCollisionPoints;k--){
            pos = collisionPointsSimReservoirPos;
            pos(2) = pos(2) + +0.03*k;
            printMessage(5,"There are fewer collision points, %d, than available boxes in sim, %d, moving the rest to the reservoir in the sim world -  this one to: %s \n",nrCollisionPoints,collisionPointsVisualizedCount,pos.toString(3,3).c_str());
            moveBox(k,pos);    
        }
        
    }    
                 
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


// empty line to make gcc happy
