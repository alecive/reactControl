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
//using namespace iCub::motionPlan;

#define TACTILE_INPUT_GAIN 1.5
#define VISUAL_INPUT_GAIN 0.5

#define STATE_WAIT              0
#define STATE_REACH             1
#define STATE_IDLE              2

#define NR_ARM_JOINTS 7
#define NR_ARM_JOINTS_FOR_INTERACTION_MODE 5
#define NR_TORSO_JOINTS 3 

/*********** public methods ****************************************************************************/ 

reactCtrlThread::reactCtrlThread(int _rate, const string &_name, const string &_robot,  const string &_part,
                                 int _verbosity, bool _disableTorso,  string _controlMode, 
                                 double _trajSpeed, double _globalTol, double _vMax, double _tol,
                                 string _referenceGen, 
                                 bool _tactileCollisionPointsOn, bool _visualCollisionPointsOn,
                                 bool _gazeControl, bool _stiffInteraction,
                                 bool _hittingConstraints, bool _orientationControl,
                                 bool _additionaControlPoints, 
                                 bool _visualizeTargetInSim, bool _visualizeParticleInSim, bool _visualizeCollisionPointsInSim,
                                 particleThread *_pT) :
                                 RateThread(_rate), name(_name), robot(_robot), part(_part),
                                 verbosity(_verbosity), useTorso(!_disableTorso), controlMode(_controlMode),
                                 trajSpeed(_trajSpeed), globalTol(_globalTol), vMax(_vMax), tol(_tol),
                                 referenceGen(_referenceGen), 
                                 tactileCollisionPointsOn(_tactileCollisionPointsOn), visualCollisionPointsOn(_visualCollisionPointsOn),
                                 gazeControl(_gazeControl), stiffInteraction(_stiffInteraction),
                                 hittingConstraints(_hittingConstraints),orientationControl(_orientationControl),
                                 additionalControlPoints(_additionaControlPoints),
                                 visualizeTargetInSim(_visualizeTargetInSim), visualizeParticleInSim(_visualizeParticleInSim),
                                 visualizeCollisionPointsInSim(_visualizeCollisionPointsInSim)
{
    dT=getRate()/1000.0;
    prtclThrd=_pT;  //in case of referenceGen != uniformParticle, NULL will be received
}

bool reactCtrlThread::threadInit()
{
   
    printMessage(2,"[reactCtrlThread] threadInit()\n");
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
    qIntegrated.resize(chainActiveDOF,0.0); //joint angle pos predictions from integrator
    lim.resize(chainActiveDOF,2); //joint pos limits
    
    q_dot.resize(chainActiveDOF,0.0); 
    vLimNominal.resize(chainActiveDOF,2);
    vLimAdapted.resize(chainActiveDOF,2);
    for (size_t r=0; r<chainActiveDOF; r++)
    {
        vLimNominal(r,0)=-vMax;
        vLimAdapted(r,0)=-vMax;
        vLimNominal(r,1)=vMax;
        vLimAdapted(r,1)=vMax;
    }
    if (useTorso){ 
        ;
        // disable torso pitch
        //vLimNominal(0,0)=vLimNominal(0,1)=0.0;  
        //vLimAdapted(0,0)=vLimAdapted(0,1)=0.0;  
        // disable torso roll
        //vLimNominal(1,0)=vLimNominal(1,1)=0.0;  
        //vLimAdapted(1,0)=vLimAdapted(1,1)=0.0;  
    }     
         
    //H.resize(4,4);

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
        okA = okA && ddA.view(iposDirA);
        okA = okA && ddA.view(imodA);
        okA = okA && ddA.view(ilimA);
        okA = okA && ddA.view(iintmodeA);
        okA = okA && ddA.view(iimpA);
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
        okT = okT && ddT.view(iposDirT);
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
    
    interactionModesOrig.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_STIFF);
    jointsToSetInteractionA.clear();
    for (int i=0; i<NR_ARM_JOINTS_FOR_INTERACTION_MODE;i++)
        jointsToSetInteractionA.push_back(i);
    iintmodeA->getInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.getFirst(),interactionModesOrig.getFirst());
    if(stiffInteraction){
        interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_STIFF);
        iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.getFirst(),interactionModesNew.getFirst());
    }
    else{
        interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_COMPLIANT);
        iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.getFirst(),interactionModesNew.getFirst());
        iimpA->setImpedance(0,0.4,0.03); 
        iimpA->setImpedance(1,0.4,0.03);
        iimpA->setImpedance(2,0.4,0.03);
        iimpA->setImpedance(3,0.2,0.01);
        iimpA->setImpedance(4,0.2,0.0);
    }
    
    if (!alignJointsBounds())
    {
        yError("[reactCtrlThread]alignJointsBounds failed!!!\n");
        return false;
    }
   
    if(gazeControl){ 
        Property OptGaze;
        OptGaze.put("device","gazecontrollerclient");
        OptGaze.put("remote","/iKinGazeCtrl");
        OptGaze.put("local",("/"+name+"/gaze").c_str());

        if ((!ddG.open(OptGaze)) || (!ddG.view(igaze))){
        yError(" could not open the Gaze Controller!");
        return false;
        }

        igaze -> storeContext(&contextGaze);
        igaze -> setSaccadesMode(false);
        igaze -> setNeckTrajTime(0.75);
        igaze -> setEyesTrajTime(0.5);
    }
   
    //filling joint pos limits Matrix
    armChain=arm->asChain();
    for (size_t jointIndex=0; jointIndex<chainActiveDOF; jointIndex++)
    {
        lim(jointIndex,0)= CTRL_RAD2DEG*(*armChain)(jointIndex).getMin();
        lim(jointIndex,1)= CTRL_RAD2DEG*(*armChain)(jointIndex).getMax();
    }
   
    /************ variables related to target and the optimization problem for ipopt *******/
    if(referenceGen == "minJerk") 
        minJerkTarget = new minJerkTrajGen(3,dT,1.0); //dim 3, dT, trajTime 1s - will be overwritten later
    else
        minJerkTarget = NULL;
  
    streamingTarget = false;
    nextStreamedTargets = new motionPlan();
    additionalControlPointsVector.clear();
       
    movingTargetCircle = false;
    radius = 0.0; frequency = 0.0;
    circleCenter.resize(3,0.0);
    circleCenter(0) = -0.3; //for safety, we assign the x-coordinate on in it within iCub's reachable space
  
    updateArmChain(); 
   
    x_0.resize(3,0.0);
    x_t.resize(3,0.0);
    x_n.resize(3,0.0);
    x_d.resize(3,0.0);
    
    //set initial orientation to palm pointing away from body - using compact axis-angle representation
    //the one below works for both palms - well, maybe not exactly
    //o_0.resize(3,0.0); o_0(2)=M_PI;
    //o_t.resize(3,0.0); o_t(2)=M_PI;
    //o_n.resize(3,0.0); o_n(2)=M_PI;
    //o_d.resize(3,0.0); o_d(2)=M_PI;

   //palm facing inwards
    o_0.resize(3,0.0);  o_0(1)=-0.707*M_PI;     o_0(2)=+0.707*M_PI;
    o_t.resize(3,0.0);  o_t(1)=-0.707*M_PI;     o_t(2)=+0.707*M_PI;
    o_n.resize(3,0.0);  o_n(1)=-0.707*M_PI;     o_n(2)=+0.707*M_PI;
    o_d.resize(3,0.0);  o_d(1)=-0.707*M_PI;     o_d(2)=+0.707*M_PI;

    if(controlMode == "positionDirect"){
        virtualArm = new iCubArm(*arm);  //Creates a new Limb from an already existing Limb object - but they will be too independent limbs from now on
        virtualArmChain = virtualArm->asChain();
        I = new Integrator(dT,q,lim);  
    }
    else{
        virtualArm = NULL;
        virtualArmChain = NULL;
        I = NULL;
    }      
    /*** visualize in iCubGui  ***************/
    visualizeIniCubGui = true;
    visualizeParticleIniCubGui = false;
    visualizeTargetIniCubGui = true;
    
    /***************** ports and files*************************************************************************************/
    
    aggregPPSeventsInPort.open("/"+name+"/pps_events_aggreg:i");
    aggregSkinEventsInPort.open("/"+name+"/skin_events_aggreg:i");
    
    nextStreamedTargets->useCallback();
    nextStreamedTargets->open("/"+name+"/streamedWholeBodyTargets:i");
       
    outPort.open("/"+name +"/data:o"); //for dumping
    if (visualizeIniCubGui)
         outPortiCubGui.open(("/"+name+"/gui:o").c_str());
    
    fout_param.open("param.log");
    
    /***** writing to param file ******************************************************/
    fout_param<<chainActiveDOF<<" ";
    for (size_t i=0; i<chainActiveDOF; i++)
    {
        fout_param<<lim(i,0)<<" ";
        fout_param<<lim(i,1)<<" ";
    }
    for (size_t j=0; j<chainActiveDOF; j++)
    {
        fout_param<<vLimNominal(j,0)<<" ";
        fout_param<<vLimNominal(j,1)<<" ";
    }
    fout_param<<-1<<" "<<trajSpeed<<" "<<tol<<" "<<globalTol<<" "<<dT<<" "<<0<<" "<<0<<" ";
    // the -1 used to be trajTime, keep it for compatibility with matlab scripts 
    //the 0s used to be boundSmoothnessFlag and boundSmoothnessValue
    if(controlMode == "velocity")
        fout_param<<"1 ";
    else if(controlMode == "positionDirect")
        fout_param<<"2 ";
    fout_param<<"0 0 0 "; //used to be ipOptMemoryOn, ipOptFilterOn, filterTc  
    if(stiffInteraction)
        fout_param<<"1 ";
    else 
        fout_param<<"0 ";
    fout_param<<endl;
    
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
    t_0=Time::now();

    return true;
}

void reactCtrlThread::run()
{
    bool controlSuccess =false;
    //printMessage(2,"[reactCtrlThread::run()] started, state: %d.\n",state);
    //yarp::os::LockGuard lg(mutex);
    updateArmChain();
    //printMessage(10,"[reactCtrlThread::run()] updated arm chain.\n");
    //debug - see Jacobian
    //iCub::iKin::iKinChain &chain_temp=*arm->asChain();
    //yarp::sig::Matrix J1_temp=chain_temp.GeoJacobian();
    //yDebug("GeoJacobian: \n %s \n",J1_temp.toString(3,3).c_str());    
    
    //printMessage(0,"[reactCtrlThread::run()] streamingTarget: %d \n",streamingTarget);
    //for (std::vector<ControlPoint>::iterator it = additionalControlPointsVector.begin() ; it != additionalControlPointsVector.end(); ++it)
    //{
      //  printMessage(0,(*it).toString().c_str());
    //}
    if (streamingTarget)    //read "trajectory" - in this special case, it is only set of next target positions for possibly multiple control points
    {
        
        if (nextStreamedTargets->gotNewMsg())
        {
            printf("[reactCtrlThread::run()] Streaming target - new message arrived.\n");
            nextStreamedTargets->setNewMsg(false);
            deque<waypointTrajectory> waypointTraj = nextStreamedTargets->getListTrajectory();
            additionalControlPointsVector.clear();
            for (std::deque<waypointTrajectory>::iterator it = waypointTraj.begin() ; it != waypointTraj.end(); ++it)
            {
                yInfo("[reactCtrlThread::run()] (*it).getCtrlPointName() = %s\n", (*it).getCtrlPointName().c_str()); //for debugging
                if ( ((*it).getCtrlPointName() == "End-Effector") || ((*it).getCtrlPointName() == "Elbow"))
                {
                    yInfo("[reactCtrlThread::run()] check getCtrlPointName()"); //for debugging
                    if ((*it).getDimension() >= 3)
                    {
                        yInfo("[reactCtrlThread::run()] check getDimension()"); //for debugging
                        if ((*it).getDimension() > 3)
                            yWarning("[reactCtrlThread::run()] %d dimensions specified for control point %s - only first three (pos) will be used.",(*it).getDimension(),(*it).getCtrlPointName().c_str());
                        if ((*it).getNbWaypoint() >= 1)
                        {
                            yInfo("[reactCtrlThread::run()] check getNbWaypoint()");    //for debugging
                            if ((*it).getNbWaypoint() > 1)
                                yWarning("[reactCtrlThread::run()] %d waypoints specified for control point %s - only first one will be used.",(*it).getNbWaypoint(),(*it).getCtrlPointName().c_str());
                            if ((*it).getCtrlPointName() == "End-Effector")
                            {
                                yInfo("[reactCtrlThread::run()] check getCtrlPointName()== End-Effector");  //for debugging
                                Vector x_temp = (*it).getWaypoints().front();
                                printf("\tReceived x_temp = %s\n",x_temp.toString(3,3).c_str());
                                if (x_temp.size() == (*it).getDimension())
                                    setNewTarget((*it).getWaypoints().front(),false);
                                else
                                    yWarning("[reactCtrlThread::run()] problem with obtaining waypoint\n");


                                //vector<Vector> waypoints = (*it).getWaypoints(); //temporary
                                //setNewTarget(*(++(waypoints.begin())),false); //temporary
                                printMessage(0,"[reactCtrlThread::run()] setting end-eff position from streaming: (%s).\n",x_d.toString(3,3).c_str());
                            }
                            else if ((*it).getCtrlPointName() == "Elbow")
                            { //elbow
                                ControlPoint *controlPoint = new ControlPoint();
                                controlPoint->type = "Elbow";
                                controlPoint->x_desired = (*it).getWaypoints().front();
                                //vector<Vector> waypoints2 = (*it).getWaypoints(); //temporary
                                //controlPoint->x_desired = *(++(waypoints2.begin())); //temporary
                                 //temporary
                                //Vector elbow_des_pos(3,0.0);
                                //elbow_des_pos(0)=-0.1; elbow_des_pos(1)= -0.1; elbow_des_pos(2)= 0.0;
                                //elbow_des_pos(0)=-0.027; elbow_des_pos(1)= -0.243; elbow_des_pos(2)= 0.195;
                                //controlPoint->x_desired = elbow_des_pos; //temporary
                                //printf("testing: setting fixed elbow target to (%s) \n",controlPoint->x_desired.toString(3,3).c_str());
                                printMessage(0,"[reactCtrlThread::run()] Pushing desired elbow position from streaming: (%s).\n",
                                             controlPoint->x_desired.toString(3,3).c_str());
                                additionalControlPointsVector.push_back(*controlPoint);
                            }
                        }
                        else
                            yWarning("[reactCtrlThread::run()] %d waypoints for control point %s < 1 - ignoring.",(*it).getNbWaypoint(),(*it).getCtrlPointName().c_str());
                    
                    }
                    else //((*it).numberDimension < 3)
                        yWarning("[reactCtrlThread::run()] %d dimensions specified for control point %s < 3 - ignoring.",(*it).getDimension(),(*it).getCtrlPointName().c_str());
                }
                else
                    yWarning("[reactCtrlThread::run()] Don't know how to handle control point of type %s.",(*it).getCtrlPointName().c_str());
                
            }
            printf("Filled up additionalControlPointsVector: \n");
            for (std::vector<ControlPoint>::iterator it = additionalControlPointsVector.begin() ; it != additionalControlPointsVector.end(); ++it)
            {
               printMessage(0,(*it).toString().c_str());
            }
   
        }
                         
          
    }

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
            if ((norm(x_t-x_d) < globalTol)) //we keep solving until we reach the desired target
            {
                yDebug(0,"[reactCtrlThread] norm(x_t-x_d) %g\tglobalTol %g\n",norm(x_t-x_d),globalTol);
                state=STATE_IDLE;
                if (!stopControlHelper())
                    yError("[reactCtrlThread] Unable to properly stop the control of the arm!");
                break;
            }
            
            if (movingTargetCircle){
                x_d = getPosMovingTargetOnCircle();
                if(visualizeTargetInSim){
                    Vector x_d_sim(3,0.0);
                    convertPosFromRootToSimFoR(x_d,x_d_sim);
                    moveSphere(1,x_d_sim);
                }
            }
            if (visualizeTargetIniCubGui){
                sendiCubGuiObject("target");
            }
            if (referenceGen == "uniformParticle"){
                if ( (norm(x_n-x_0) > norm(x_d-x_0)) || movingTargetCircle) //if the particle is farther than the final target, we reset the particle - it will stay with the target; or if target is moving
                {
                    prtclThrd->resetParticle(x_d);
                }
                x_n=prtclThrd->getParticle(); //to get next target
            }
            else if(referenceGen == "minJerk"){
                minJerkTarget->computeNextValues(x_d);    
                //refGenMinJerk->computeNextValues(x_t,x_d); 
                x_n = minJerkTarget->getPos();
            }

            else if(referenceGen == "none")
            {
                //yInfo("no reference gen mode\n");
                x_n = x_d;
            }
 
            if(visualizeParticleIniCubGui){
                sendiCubGuiObject("particle");
            }
    
            if (referenceGen != "none")
            {
                if(visualizeParticleInSim){
                    Vector x_n_sim(3,0.0);
                    convertPosFromRootToSimFoR(x_n,x_n_sim);
                    moveSphere(2,x_n_sim); //sphere created as second (particle) will keep the index 2
                }
            }
            
            if(gazeControl)
                igaze -> lookAtFixationPoint(x_d); //for now looking at final target (x_d), not at intermediate/next target x_n
            
             if (tactileCollisionPointsOn || visualCollisionPointsOn){
                AvoidanceHandlerAbstract *avhdl; 
                avhdl = new AvoidanceHandlerTactile(*arm->asChain(),collisionPoints,verbosity); //the "tactile" handler will currently be applied to visual inputs (from PPS) as well
                vLimAdapted=avhdl->getVLIM(vLimNominal);
                delete avhdl; avhdl = NULL; //TODO this is not efficient, in the future find a way to reset the handler, not recreate
            }
            
         
            //printMessage(2,"[reactCtrlThread::run()]: Will call solveIK.\n");
            double t_1=yarp::os::Time::now();
            q_dot = solveIK(ipoptExitCode);
            timeToSolveProblem_s  = yarp::os::Time::now()-t_1;
                                
           if (ipoptExitCode==Ipopt::Solve_Succeeded || ipoptExitCode==Ipopt::Maximum_CpuTime_Exceeded)
            {
                if (ipoptExitCode==Ipopt::Maximum_CpuTime_Exceeded)
                    yWarning("[reactCtrlThread] Ipopt cpu time was higher than the rate of the thread!");
            }
            else
                  yWarning("[reactCtrlThread] Ipopt solve did not succeed!");

            if(controlMode == "positionDirect"){
                //yInfo()<<"   t after opt, before control [s] = "<<yarp::os::Time::now() -t_0;
                //yInfo()<<"   q_dot [deg/s] = ("<<q_dot.toString(3,3)<<")";
                //yInfo()<<"   integrate joint pos with time step: "<<dT;
                //yInfo("E.g., %f + %f*%f = %f",qIntegrated(0),q_dot(0),dT,qIntegrated(0)+q_dot(0)*dT);
                //yInfo()<<"   qIntegrated before integration [deg] = ("<<qIntegrated.toString(3,3)<<")";
                qIntegrated = I->integrate(q_dot);    
                //yInfo()<<"   qIntegrated after integration [deg] = ("<<qIntegrated.toString(3,3)<<")";
                //yInfo()<<"   joint positions real before control [deg] ("<<q.toString(3,3)<<")";
                //yInfo()<<"   xee_pos_real before                    control [m] = "<<x_t.toString(3,3);         
                if (!controlArm(controlMode,qIntegrated)){
                    yError("I am not able to properly control the arm in positionDirect!");
                    controlSuccess = false;
                }
                else{
                    controlSuccess = true; 
                }
               //Vector xee_pos_virtual_before=virtualArmChain->EndEffPosition();
               //yInfo()<<"   xee_pos_virtual before updating virtual chain [m] = "<<xee_pos_virtual_before.toString(3,3);         
               //Vector qVirtualChainReturn = CTRL_RAD2DEG*
               virtualArmChain->setAng(qIntegrated*CTRL_DEG2RAD);
               //yInfo()<<"   virtualChain setAng return [deg] = ("<<qVirtualChainReturn.toString(3,3)<<")";
                
               
            }
        
            else if (controlMode == "velocity"){
                if (!controlArm(controlMode,q_dot)){
                    yError("I am not able to properly control the arm in velocity!");
                    controlSuccess = false;   
                }
                else
                    controlSuccess = true;
            }
            
            updateArmChain();
            //Vector xee_pos_virtual_after=virtualArmChain->EndEffPosition();
            //yInfo()<<"   t after opt and control [s] = "<<yarp::os::Time::now() -t_0;
            //yInfo()<<"   xee_pos_virtual after updating virtual chain [m] = "<<xee_pos_virtual_after.toString(3,3);         
            //yInfo()<<"   xee_pos_real after                    control [m] = "<<x_t.toString(3,3);         
            //yInfo("  virtualChain.getAng() [deg] (%s)",(virtualArmChain->getAng()*CTRL_RAD2DEG).toString().c_str());
            //yInfo("  arm->getAng()         [deg] (%s)",(arm->getAng()*CTRL_RAD2DEG).toString().c_str());
            //yInfo()<<"   joint positions real after control [deg] ("<<q.toString(3,3)<<")";
            //yInfo()<<"   e_pos_real after opt step and control [m] = "<<norm(x_n-x_t);
            //yInfo()<<"   e_pos_virtual after opt step and control [m] = "<<norm(x_n-xee_pos_virtual_after);
            //yInfo()<<"";
    
            
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
    if (tactileCollisionPointsOn || visualCollisionPointsOn)
        vLimAdapted = vLimNominal; //if it was changed by the avoidanceHandler, we reset it
    printMessage(2,"[reactCtrlThread::run()] finished, state: %d.\n\n\n",state);
    
}

void reactCtrlThread::threadRelease()
{
    
    yInfo("Putting back original interaction modes."); 
    iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.getFirst(),interactionModesOrig.getFirst());
    jointsToSetInteractionA.clear();
    interactionModesNew.clear();
    interactionModesOrig.clear();
    
    yInfo("threadRelease(): deleting arm and torso encoder arrays and arm object.");
    delete encsA; encsA = NULL;
    delete encsT; encsT = NULL;
    delete   arm;   arm = NULL;
    bool stoppedOk = stopControlAndSwitchToPositionMode();
    if (stoppedOk)
        yInfo("Sucessfully stopped arm and torso controllers");
    else
        yWarning("Controllers not stopped sucessfully");
    yInfo("Closing controllers..");
    ddA.close();
    ddT.close();
    
    if(gazeControl){
        yInfo("Closing gaze controller..");
        Vector ang(3,0.0);
        igaze -> lookAtAbsAngles(ang);
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
        ddG.close();
    }
    
    collisionPoints.clear();    
    
    if(nextStreamedTargets != NULL){
        yDebug("deleting nextStreamedTargets..");
        delete nextStreamedTargets;
        nextStreamedTargets = NULL;    
    }
    
    additionalControlPointsVector.clear();
    if(minJerkTarget != NULL){
        yDebug("deleting minJerkTarget..");
        delete minJerkTarget;
        minJerkTarget = NULL;    
    }
  
    if(virtualArm != NULL){
        yDebug("deleting virtualArm..");
        delete virtualArm;
        virtualArm = NULL;   
        virtualArmChain = NULL;    
    }
         
    if(I != NULL){
        yDebug("deleting integrator I..");
        delete I;
        I = NULL;    
    }
       
    if (visualizeIniCubGui)
        yInfo("Resetting objects in iCubGui..");
        if (outPortiCubGui.getOutputCount()>0)
        {
            Bottle b;
            b.addString("reset");
            outPortiCubGui.write(b);
        }
    
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
        if (outPortiCubGui.isOpen()){
            outPortiCubGui.interrupt();
            outPortiCubGui.close();
        }
        if (portToSimWorld.isOpen()){
            portToSimWorld.interrupt();
            portToSimWorld.close();
        }
   // yInfo("Closing files..");    
     //   fout_param.close();

}


bool reactCtrlThread::enableTorso()
{
   // yarp::os::LockGuard lg(mutex);
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
    //yarp::os::LockGuard lg(mutex);
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

bool reactCtrlThread::setNewTarget(const Vector& _x_d, bool _movingCircle)
{
    if (_x_d.size()==3)
    {
        movingTargetCircle = _movingCircle;
        q_dot.zero();
        updateArmChain(); //updates chain, q and x_t
        virtualArmChain->setAng(q*CTRL_DEG2RAD); //with new target, we make the two chains identical at the start
        if(controlMode == "positionDirect")
           I->reset(q);   
                
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
                
        if (referenceGen == "uniformParticle"){
            yarp::sig::Vector vel(3,0.0);
            vel=trajSpeed * (x_d-x_0) / norm(x_d-x_0);
            if (!prtclThrd->setupNewParticle(x_0,vel)){
                yWarning("prtclThrd->setupNewParticle(x_0,vel) returned false.\n");
                return false;
            }
        }
        else if(referenceGen == "minJerk"){
            minJerkTarget->init(x_0); //initial pos
            minJerkTarget->setTs(dT); //time step
            double T = 1.0; // 1 s 
            //calculate the time to reach from the distance to target and desired velocity - this was wrong somehow
            //double T = sqrt( (x_d(0)-x_0(0))*(x_d(0)-x_0(0)) + (x_d(1)-x_0(1))*(x_d(1)-x_0(1)) + (x_d(2)-x_0(2))*(x_d(2)-x_0(2)) )  / trajSpeed; 
            minJerkTarget->setT(T);
        }
        /*else if (referenceGen == "none")
        {
            ;
        } */

        
        yInfo("[reactCtrlThread] got new target: x_0: %s",x_0.toString(3,3).c_str());
        yInfo("[reactCtrlThread]                 x_d: %s",x_d.toString(3,3).c_str());
        //yInfo("[reactCtrlThread]                 vel: %s",vel.toString(3,3).c_str());
       
        if(visualizeTargetIniCubGui)
            sendiCubGuiObject("target");
               
        if(visualizeParticleInSim){
            Vector x_0_sim(3,0.0);
            convertPosFromRootToSimFoR(x_0,x_0_sim);
            if (firstTarget)
               createStaticSphere(0.02,x_0_sim);
            else
               moveSphere(2,x_0_sim); //sphere created as second will keep the index 2  
        }
              
        if (firstTarget)
            firstTarget = false;
       
        state=STATE_REACH;
             
        return true;
    }
    else
        return false;
}

bool reactCtrlThread::setNewRelativeTarget(const Vector& _rel_x_d)
{
    streamingTarget = false;
    if(_rel_x_d == Vector(3,0.0)) return false;
    updateArmChain(); //updates chain, q and x_t
    Vector _x_d = x_t + _rel_x_d;
    return setNewTarget(_x_d,false);
}

bool reactCtrlThread::setNewCircularTarget(const double _radius,const double _frequency)
{
    streamingTarget = false;
    radius = _radius;
    frequency = _frequency;
    updateArmChain(); //updates chain, q and x_t
    circleCenter = x_t; // set it to end-eff position at this point 
    
    setNewTarget(getPosMovingTargetOnCircle(),true);
    return true;
}

bool reactCtrlThread::setStreamingTarget()
{
    streamingTarget = true;
    return true;
}

bool reactCtrlThread::stopControl()
{
    //yarp::os::LockGuard lg(mutex);
    bool stoppedOk = stopControlHelper();
    if (stoppedOk)
        yInfo("reactCtrlThread::stopControl(): Sucessfully stopped controllers");
    else
        yWarning("reactCtrlThread::stopControl(): Controllers not stopped sucessfully"); 
    return stoppedOk;
}

bool reactCtrlThread::stopControlAndSwitchToPositionMode()
{
    //yarp::os::LockGuard lg(mutex);
    bool stoppedOk = stopControlAndSwitchToPositionModeHelper();
    if (stoppedOk)
        yInfo("reactCtrlThread::stopControlAndSwitchToPositionMode(): Sucessfully stopped controllers");
    else
        yWarning("reactCtrlThread::stopControlAndSwitchToPositionMode(): Controllers not stopped sucessfully"); 
    return stoppedOk;
}



//************** protected methods *******************************/


Vector reactCtrlThread::solveIK(int &_exit_code)
{
      
    printMessage(3,"calling ipopt with the following joint velocity limits (deg): \n %s \n",vLimAdapted.toString(3,3).c_str());
    //printf("calling ipopt with the following joint velocity limits (rad): \n %s \n",(vLimAdapted*CTRL_DEG2RAD).toString(3,3).c_str());
    // Remember: at this stage everything is kept in degrees because the robot is controlled in degrees.
    // At the ipopt level it comes handy to translate everything in radians because iKin works in radians.
   
   //Vector xee_pos_virtual=virtualArmChain->EndEffPosition();
   //Vector xee_pos_real= arm->asChain()->EndEffPosition();
   //yInfo()<<"   t [s] = "<<yarp::os::Time::now() -t_0;
   //yInfo()<<"   e_pos_real before opt step [m] = "<<norm(x_n-xee_pos_real);
   //yInfo()<<"   e_pos real using x_t       [m] = "<<norm(x_n-x_t); 
   //yInfo()<<"   e_pos_virtual before opt step [m] = "<<norm(x_n-xee_pos_virtual);
        
   Vector res(chainActiveDOF,0.0); 
    
   Vector xr(6,0.0);
   xr.setSubvector(0,x_n);
   xr.setSubvector(3,o_n);
    
   Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
   app->Options()->SetNumericValue("tol",tol);
   app->Options()->SetNumericValue("constr_viol_tol",1e-6);
   app->Options()->SetIntegerValue("acceptable_iter",0);
   app->Options()->SetStringValue("mu_strategy","adaptive");
   app->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
   app->Options()->SetNumericValue("max_cpu_time",0.75*dT);
   app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
   app->Options()->SetStringValue("hessian_approximation","limited-memory");
   app->Options()->SetStringValue("derivative_test",verbosity?"first-order":"none");
   app->Options()->SetIntegerValue("print_level",verbosity?5:0);
   app->Initialize();

   /*Matrix H5real=(*(arm->asChain())).getH(5);
   Vector elbow_pos_real = H5real.getCol(3).subVector(0,2);
   //printf("[reactCtrlThread::solveIK]: real chain getH(%d) - elbow: \n %s \n",*(arm->asChain()).getDOF()-4-1,H5real.toString(3,3).c_str());
   printf("[reactCtrlThread::solveIK]: real p0 - elbow: (%s)\n",elbow_pos_real.toString(3,3).c_str());
   Matrix H5virtual=(*virtualArmChain).getH(5);
   Vector elbow_pos_virtual = H5virtual.getCol(3).subVector(0,2);
   //printf("[reactCtrlThread::solveIK]: real chain getH(%d) - elbow: \n %s \n",*(arm->asChain()).getDOF()-4-1,H5real.toString(3,3).c_str());
   printf("[reactCtrlThread::solveIK]: virtual p0 - elbow: (%s)\n",elbow_pos_virtual.toString(3,3).c_str());    */                 
   
   Ipopt::SmartPtr<ControllerNLP> nlp;
   if (controlMode == "positionDirect") //in this mode, ipopt will use the qIntegrated values to update its copy of chain
        nlp=new ControllerNLP(*virtualArmChain,additionalControlPointsVector);
   else
        nlp=new ControllerNLP(*(arm->asChain()),additionalControlPointsVector);
   nlp->set_hitting_constraints(hittingConstraints);
   nlp->set_orientation_control(orientationControl);
   nlp->set_additional_control_points(additionalControlPoints);
   nlp->set_dt(dT);
   nlp->set_xr(xr);
   nlp->set_v_limInDegPerSecond(vLimAdapted);
   nlp->set_v0InDegPerSecond(q_dot);
   nlp->init();

   _exit_code=app->OptimizeTNLP(GetRawPtr(nlp));
  
   res=nlp->get_resultInDegPerSecond();
   
    // printMessage(0,"t_d: %g\tt_t: %g\n",t_d-t_0, t_t-t_0);
    if(verbosity >= 1){ 
        printf("x_n: %s\tx_d: %s\tdT %g\n",x_n.toString(3,3).c_str(),x_d.toString(3,3).c_str(),dT);
        printf("x_0: %s\tx_t: %s\n",       x_0.toString(3,3).c_str(),x_t.toString(3,3).c_str());
        printf("norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g\n",
                    norm(x_n-x_t), norm(x_d-x_n), norm(x_d-x_t));
        printf("Result (solved velocities (deg/s)): %s\n",res.toString(3,3).c_str());
    }
     
   
    return res;
}


 /**** kinematic chain, control, ..... *****************************/

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
    //H=arm->getH();
    //x_t=H.subcol(0,3,3);
    x_t = arm->EndEffPosition();
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
        yDebug("[jointsBounds (deg)] i: %lu\tmin: %g\tmax %g",i,min,max);
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
        else if (_s=="positionDirect")
        {
            if (modes[i]!=VOCAB_CM_POSITION_DIRECT)
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
    if (_s!="position" && _s!="velocity" && _s!="positionDirect")
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
        else if (_s=="positionDirect")
        {
            modes.push_back(VOCAB_CM_POSITION_DIRECT);
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

//N.B. the targetValues can be either positions or velocities, depending on the control mode!
bool reactCtrlThread::controlArm(const string _controlMode, const yarp::sig::Vector &_targetValues)
{   
    VectorOf<int> jointsToSetA;
    VectorOf<int> jointsToSetT;
    if (!areJointsHealthyAndSet(jointsToSetA,"arm",_controlMode))
    {
        yWarning("[reactCtrlThread::controlArm] Stopping control because arm joints are not healthy!");
        stopControlHelper();
        return false;
    }
 
    if (useTorso)
    {
        if (!areJointsHealthyAndSet(jointsToSetT,"torso",_controlMode))
        {
            yWarning("[reactCtrlThread::controlArm] Stopping control because torso joints are not healthy!");
            stopControlHelper();
            return false;
        }
    }
    
    if (!setCtrlModes(jointsToSetA,"arm",_controlMode))
    {
        yError("[reactCtrlThread::controlArm] I am not able to set the arm joints to %s mode!",_controlMode.c_str());
        return false;
    }   

    if (useTorso)
    {
        if (!setCtrlModes(jointsToSetT,"torso",_controlMode))
        {
            yError("[reactCtrlThread::controlArm] I am not able to set the torso joints to %s mode!",_controlMode.c_str());
            return false;
        }
    }
    /*if(verbosity>=10){
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
    }*/
    if (_controlMode == "velocity"){
        printMessage(1,"[reactCtrlThread::controlArm] Joint velocities (iKin order, deg/s): %s\n",_targetValues.toString(3,3).c_str());
        if (useTorso)
        {
            Vector velsT(TORSO_DOF,0.0);
            velsT[0] = _targetValues[2]; //swapping pitch and yaw as per iKin vs. motor interface convention
            velsT[1] = _targetValues[1];
            velsT[2] = _targetValues[0]; //swapping pitch and yaw as per iKin vs. motor interface convention
        
            printMessage(2,"    velocityMove(): torso (swap pitch & yaw): %s\n",velsT.toString(3,3).c_str());
            ivelT->velocityMove(velsT.data());
            ivelA->velocityMove(_targetValues.subVector(3,9).data()); //indexes 3 to 9 are the arm joints velocities
        }
        else
        {
            ivelA->velocityMove(_targetValues.data()); //if there is not torso, _targetValues has only the 7 arm joints
        }
    }
    else if(_controlMode == "positionDirect"){ 
         printMessage(1,"[reactCtrlThread::controlArm] Target joint positions (iKin order, deg): %s\n",_targetValues.toString(3,3).c_str());
        if (useTorso)
        {
            Vector posT(3,0.0);
            posT[0] = _targetValues[2]; //swapping pitch and yaw as per iKin vs. motor interface convention
            posT[1] = _targetValues[1];
            posT[2] = _targetValues[0]; //swapping pitch and yaw as per iKin vs. motor interface convention
        
            printMessage(2,"    positionDirect: torso (swap pitch & yaw): %s\n",posT.toString(3,3).c_str());
            iposDirT->setPositions(posT.data());
            iposDirA->setPositions(_targetValues.subVector(3,9).data()); //indexes 3 to 9 are the arm joints 
        }
        else
        {
            iposDirA->setPositions(_targetValues.data()); //if there is not torso, _targetValues has only the 7 arm joints
        }
        
    }
        
    return true;
}


bool reactCtrlThread::stopControlHelper()
{
    if (useTorso)
    {
        return ivelA->stop() && ivelT->stop();
    }

    return ivelA->stop();
}


bool reactCtrlThread::stopControlAndSwitchToPositionModeHelper()
{
    state=STATE_IDLE;
    
    VectorOf<int> jointsToSetA;
    jointsToSetA.push_back(0);jointsToSetA.push_back(1);jointsToSetA.push_back(2);jointsToSetA.push_back(3);jointsToSetA.push_back(4);
    jointsToSetA.push_back(5);jointsToSetA.push_back(6);
    if (useTorso)
    {
        ivelA->stop();
        ivelT->stop();
        VectorOf<int> jointsToSetT;
        jointsToSetT.push_back(0);jointsToSetT.push_back(1);jointsToSetT.push_back(2);
        return  setCtrlModes(jointsToSetA,"arm","position") && setCtrlModes(jointsToSetT,"torso","position");
    }
    else{
        ivelA->stop();
        return  setCtrlModes(jointsToSetA,"arm","position"); 
        
    }
   
}



/***************** auxiliary computations  *******************************/
 
Vector reactCtrlThread::getPosMovingTargetOnCircle()
{
      Vector _x_d=circleCenter; 
      //x-coordinate will stay constant; we set y, and z
      _x_d[1]+=radius*cos(2.0*M_PI*frequency*yarp::os::Time::now());
      _x_d[2]+=radius*sin(2.0*M_PI*frequency*yarp::os::Time::now());

      return _x_d;
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

 /**** communication through ports in/out ****************/


bool reactCtrlThread::getCollisionPointsFromPort(BufferedPort<Bottle> &inPort, double gain, string which_chain,std::vector<collisionPoint_t> &collPoints)
{
    //printMessage(9,"[reactCtrlThread::getCollisionPointsFromPort].\n");
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
             printMessage(5,"Bottle %d contains %s \n", i,collPointBottle->toString().c_str());
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
                collPoint.magnitude = collPointBottle->get(13).asDouble() * gain;
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
            //variable - if torso on: 21:30: actual joint positions 
            vectorIntoBottle(q,b); 
            //variable - if torso on: 31:50; joint vel limits as input to ipopt, after avoidanceHandler,
            matrixIntoBottle(vLimAdapted,b); // assuming it is row by row, so min_1, max_1, min_2, max_2 etc.
            b.addInt(ipoptExitCode);
            b.addDouble(timeToSolveProblem_s);
            if (controlMode == "positionDirect")
                vectorIntoBottle(qIntegrated,b);
            
            // the delta_x, that is the 3D vector that ipopt commands to 
            //    the robot in order for x_t to reach x_n
            //yarp::os::Bottle &b_delta_x=out.addList();
            //iCub::skinDynLib::vectorIntoBottle(computeDeltaX(),b_delta_x);
                         
            outPort.setEnvelope(ts);
            outPort.write(b);
        }
    }
}


/**** visualizations using iCubGui **************************************/


void reactCtrlThread::sendiCubGuiObject(const string object_type)
{
    if (outPortiCubGui.getOutputCount()>0)
    {
        Bottle obj;
        if (object_type == "particle"){
            obj.addString("object");
            obj.addString("particle");
     
            // size 
            obj.addDouble(20.0);
            obj.addDouble(20.0);
            obj.addDouble(20.0);
        
            // positions - iCubGui works in mm
            obj.addDouble(1000*x_n(0));
            obj.addDouble(1000*x_n(1));
            obj.addDouble(1000*x_n(2));
        
            // orientation
            obj.addDouble(0.0);
            obj.addDouble(0.0);
            obj.addDouble(0.0);
        
            // color
            obj.addInt(125);
            obj.addInt(255);
            obj.addInt(125);
        
            // transparency
            obj.addDouble(0.9);
        }
        else if(object_type == "target"){
            obj.addString("object");
            obj.addString("target");
     
            // size 
            obj.addDouble(40.0);
            obj.addDouble(40.0);
            obj.addDouble(40.0);
        
            // positions - iCubGui works in mm
            obj.addDouble(1000*x_d(0));
            obj.addDouble(1000*x_d(1));
            obj.addDouble(1000*x_d(2));
        
            // orientation
            obj.addDouble(0.0);
            obj.addDouble(0.0);
            obj.addDouble(0.0);
        
            // color
            obj.addInt(0);
            obj.addInt(255);
            obj.addInt(0);
        
            // transparency
            obj.addDouble(0.7);
            
        }
       
        outPortiCubGui.write(obj);
        
    }
}

void reactCtrlThread::deleteiCubGuiObject(const string object_type)
{
    if (outPortiCubGui.getOutputCount()>0)
    {
        Bottle obj;
        obj.addString("delete");
        obj.addString(object_type);
        outPortiCubGui.write(obj);
    }
}
 
 
  
 /***** visualizations in iCub simulator ********************************/

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
