/* 
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Matej Hoffmann <matej.hoffmann@iit.it>, Alessandro Roncone <alessandro.roncone@yale.edu>
 * website: www.robotcub.org
 * author websites: https://sites.google.com/site/matejhof, http://alecive.github.io
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

#include <utility>
#include <vector>
#include <sstream>

#include <IpIpoptApplication.hpp>

#include <yarp/os/Time.h>
#include <iCub/ctrl/math.h>

#include "reactCtrlThread.h"

using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;

#define TACTILE_INPUT_GAIN 1.5
#define VISUAL_INPUT_GAIN 0.5

#define STATE_WAIT              0
#define STATE_REACH             1
#define STATE_IDLE              2

#define NR_ARM_JOINTS 7
#define NR_ARM_JOINTS_FOR_INTERACTION_MODE 5
#define NR_TORSO_JOINTS 3 

/*********** public methods ****************************************************************************/ 

reactCtrlThread::reactCtrlThread(int _rate, string _name, string _robot,  string _part,
                                 int _verbosity, bool _disableTorso,  string _controlMode, 
                                 double _trajSpeed, double _globalTol, double _vMax, double _tol,
                                 string _referenceGen,
                                 bool _tactileCollisionPointsOn, bool _visualCollisionPointsOn,
                                 bool _gazeControl, bool _stiffInteraction,
                                 bool _hittingConstraints, bool _orientationControl,
                                 bool _additionaControlPoints,
                                 bool _visualizeTargetInSim, bool _visualizeParticleInSim, bool _visualizeCollisionPointsInSim,
                                 particleThread *_pT, double _restPosWeight, bool _selfColPoints) :
                                 PeriodicThread((double)_rate/1000.0), name(std::move(_name)), robot(std::move(_robot)), part(std::move(_part)),
                                 verbosity(_verbosity), useTorso(!_disableTorso), controlMode(std::move(_controlMode)),
                                 trajSpeed(_trajSpeed), globalTol(_globalTol), vMax(_vMax), tol(_tol),
                                 referenceGen(std::move(_referenceGen)),
                                 tactileCollisionPointsOn(_tactileCollisionPointsOn), visualCollisionPointsOn(_visualCollisionPointsOn),
                                 gazeControl(_gazeControl), stiffInteraction(_stiffInteraction),
                                 hittingConstraints(_hittingConstraints),orientationControl(_orientationControl),
                                 additionalControlPoints(_additionaControlPoints),
                                 visualizeTargetInSim(_visualizeTargetInSim), visualizeParticleInSim(_visualizeParticleInSim),
                                 visualizeCollisionPointsInSim(_visualizeCollisionPointsInSim), start_experiment(0),
                                 counter(0), t_1(0), restPosWeight(_restPosWeight), selfColPoints(_selfColPoints)
{
    dT=getPeriod();
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
   
    arm = new iCub::iKin::iCubArm(part_short);
    // Release / block torso links (blocked by default)
    for (int i = 0; i < NR_TORSO_JOINTS; i++)
    {
        arm->releaseLink(i);
    }

    //we set up the variables based on the current DOF - that is without torso joints if torso is blocked
    chainActiveDOF = arm->getDOF();
 
    //N.B. All angles in this thread are in degrees
    qA.resize(NR_ARM_JOINTS,0.0); //current values of arm joints (should be 7)
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
        // disable torso pitch
        //vLimNominal(0,0)=vLimNominal(0,1)=0.0;
        //vLimAdapted(0,0)=vLimAdapted(0,1)=0.0;
        // disable torso roll
        vLimNominal(1,0)=vLimNominal(1,1)=0.0;
        vLimAdapted(1,0)=vLimAdapted(1,1)=0.0;
    }
    else
    {
        // disable torso pitch
        vLimNominal(0,0)=vLimNominal(0,1)=0.0;
        vLimAdapted(0,0)=vLimAdapted(0,1)=0.0;
        // disable torso roll
        vLimNominal(1,0)=vLimNominal(1,1)=0.0;
        vLimAdapted(1,0)=vLimAdapted(1,1)=0.0;
        // disable torso yaw
        vLimNominal(2,0)=vLimNominal(2,1)=0.0;
        vLimAdapted(2,0)=vLimAdapted(2,1)=0.0;
    }

    T_world_root = zeros(4,4); 
    T_world_root(0,1)=-1;
    T_world_root(1,2)=1; T_world_root(1,3)=0.5976;
    T_world_root(2,0)=-1; T_world_root(2,3)=-0.026;
    T_world_root(3,3)=1;
    //iT_world_root=SE3inv(T_world_root);
    
    /*****  Drivers, interfaces, control boards etc. ***********************************************************/
    
    yarp::os::Property OptA;
    OptA.put("robot",  robot);
    OptA.put("part",   part);
    OptA.put("device", "remote_controlboard");
    OptA.put("remote", "/"+robot+"/"+part);
    OptA.put("local",  "/"+name +"/"+part);
    if (!ddA.open(OptA))
    {
        yError("[reactCtrlThread]Could not open %s PolyDriver!",part.c_str());
        return false;
    }

    bool okA = true;
    
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
    OptT.put("robot",  robot);
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote", "/"+robot+"/torso");
    OptT.put("local",  "/"+name +"/torso");
    if (!ddT.open(OptT))
    {
        yError("[reactCtrlThread]Could not open torso PolyDriver!");
        return false;
    }

    bool okT = true;
    
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
    iintmodeA->getInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesOrig.data());
    if(stiffInteraction){
        interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_STIFF);
        iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesNew.data());
    }
    else{
        interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_COMPLIANT);
        iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesNew.data());
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
        OptGaze.put("local","/"+name+"/gaze");

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
        minJerkTarget = nullptr;
  
    streamingTarget = true;
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
        virtualArm = nullptr;
        virtualArmChain = nullptr;
        I = nullptr;
    }      
    /*** visualize in iCubGui  ***************/
    visualizeIniCubGui = true;
    visualizeParticleIniCubGui = false;
    visualizeTargetIniCubGui = true;
    
    /***************** ports and files*************************************************************************************/
    
    aggregPPSeventsInPort.open("/"+name+"/pps_events_aggreg:i");
    aggregSkinEventsInPort.open("/"+name+"/skin_events_aggreg:i");

    streamedTargets.open("/"+name+"/streamedWholeBodyTargets:i");
       
    outPort.open("/"+name +"/data:o"); //for dumping
    if (visualizeIniCubGui)
         outPortiCubGui.open("/"+name+"/gui:o");
    
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
    if(additionalControlPoints)
        fout_param<<"1 ";
    else
        fout_param<<"0 ";
    fout_param<<endl;
    
    yInfo("Written to param file and closing..");    
    fout_param.close();
    /**** visualizing targets and collision points in simulator ***************************/
    
    if((robot == "icubSim") && (visualizeTargetInSim || visualizeParticleInSim || visualizeCollisionPointsInSim) ){ 
        string port2icubsim = "/" + name + "/sim:o";
        if (!portToSimWorld.open(port2icubsim)) {
            yError("[reactCtrlThread] Unable to open port << port2icubsim << endl");
        }    
        std::string port2world = "/icubSim/world";
        yarp::os::Network::connect(port2icubsim, port2world);
    
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

    app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",tol);
    app->Options()->SetNumericValue("constr_viol_tol",1e-6);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
    app->Options()->SetNumericValue("max_cpu_time",0.7*dT);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test",verbosity?"first-order":"none");
//    app->Options()->SetStringValue("derivative_test_print_all", "yes");
    app->Options()->SetIntegerValue("print_level", verbosity?5:0);
    app->Options()->SetNumericValue("derivative_test_tol", 1e-7);
//    app->Options()->SetStringValue("print_timing_statistics", "yes");
    app->Initialize();

    //in positionDirect mode, ipopt will use the qIntegrated values to update its copy of chain
    nlp=new ControllerNLP((controlMode == "positionDirect")? *virtualArmChain:*(arm->asChain()),
                          additionalControlPointsVector, hittingConstraints, orientationControl,
                          additionalControlPoints, dT, restPosWeight);
    //the "tactile" handler will currently be applied to visual inputs (from PPS) as well
    avhdl = std::make_unique<AvoidanceHandlerTactile>(*arm->asChain(),collisionPoints,selfColPoints,verbosity);
    firstSolve = true;
    printMessage(5,"[reactCtrlThread] threadInit() finished.\n");
    yarp::os::Time::delay(0.2);
    t_0=Time::now();

    return true;
}

void reactCtrlThread::run()
{
    double t2 = yarp::os::Time::now();
    if (state == STATE_REACH) {
        std::cout << t2-t_0 << " ";
    }
    //printMessage(2,"[reactCtrlThread::run()] started, state: %d.\n",state);
    //std::lock_guard<std::mutex> lg(mut);
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
        std::vector<Vector> x_planned;
        if (readMotionPlan(x_planned))
        {
            additionalControlPointsVector.clear();
            if (!x_planned.empty())
            {
                setNewTarget(x_planned[0],false);
                if (x_planned.size()==2 && additionalControlPoints)
                {
                    ControlPoint ctrlPt;
                    ctrlPt.type = "Elbow";
                    ctrlPt.x_desired = x_planned[1];
                    additionalControlPointsVector.push_back(ctrlPt);
                }
            }
        }
    }
    else {
       //temporary - we set the desired elbow pos manually in order to test without the planner module
//        additionalControlPointsVector.clear();
//        ControlPoint *controlPoint = new ControlPoint();
//        controlPoint->type = "Elbow";
//        Vector elbow_des_pos(3,0.0);
//        elbow_des_pos(0)=-0.114; elbow_des_pos(1)= -0.176; elbow_des_pos(2)= 0.08;
//        //elbow_des_pos(0)=-0.027; elbow_des_pos(1)= -0.243; elbow_des_pos(2)= 0.195;
//        controlPoint->x_desired = elbow_des_pos; 
//        //printf("testing: setting fixed elbow target to (%s) \n",controlPoint->x_desired.toString(3,3).c_str());
//        additionalControlPointsVector.push_back(*controlPoint);
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

    if (t_1 > 0)
    {
        collisionPoint_t collisionPointStruct;
        collisionPointStruct.skin_part = SKIN_LEFT_FOREARM;
        collisionPointStruct.x.resize(3,0.0);
        collisionPointStruct.n.resize(3,0.0);
        collisionPointStruct.magnitude = 1.5; //~ "probability of collision"
        if (yarp::os::Time::now() - t_1 > 10 && counter < 150) {
            collisionPointStruct.x = {-0.0002, -0.0131,-0.0258434}; // {-0.031, -0.079, 0.005};
            collisionPointStruct.n = {-0.005, 0.238, -0.971}; // {-0.739, 0.078, 0.105};
            counter++;
            collisionPoints.push_back(collisionPointStruct);
        } else if (yarp::os::Time::now() - t_1 > 20 && counter < 450) {
            collisionPointStruct.x = {0.026828, -0.054786, -0.0191051}; // {0.014, 0.081, 0.029};
            collisionPointStruct.n = {0.883, 0.15, -0.385}; // {0.612, 0.066, 0.630};
            counter++;
            collisionPoints.push_back(collisionPointStruct);
        } else if (yarp::os::Time::now() - t_1 > 30 && counter < 500) {
            collisionPointStruct.x = {-0.027228, -0.054786, -0.0191051}; // {0.018, 0.095, 0.024};
            collisionPointStruct.n = {-0.886, 0.14, -0.431 }; // {0.568, -0.18, 0.406};
            counter++;
            collisionPoints.push_back(collisionPointStruct);
        }
    }
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
        {
            Vector v0(3,0.0);
            switch(start_experiment) {
                case 1: { v0[2] = 0.1; setNewRelativeTarget(v0); break; }
                case 2: { v0[2] = -0.2; setNewRelativeTarget(v0); break; }
                case 3: { v0[2] = 0.1; setNewRelativeTarget(v0); break; }
                case 4: setNewCircularTarget(0.1,0.1); break;
                default: break;
            }
            break;
        }
        case STATE_REACH:
        {
//            yInfo("[reactCtrlThread] norm(x_t-x_d) = %g",norm(x_t-x_d));
            if ((norm(x_t-x_d) < globalTol) || yarp::os::Time::now() > 60+t_0) //we keep solving until we reach the desired target
            {
                yDebug("[reactCtrlThread] norm(x_t-x_d) %g\tglobalTol %g",norm(x_t-x_d),globalTol);
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
                if(additionalControlPoints)
                  sendiCubGuiObject("additionalTargets");
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
                x_n = x_d;
            }
//            yDebug("[reactCtrlThread] x_n: %s,  x_n_next: %s",x_n.toString(3,3).c_str(), x_n_next.toString(3,3).c_str());
 
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
                vLimAdapted=avhdl->getVLIM(CTRL_DEG2RAD * vLimNominal) * CTRL_RAD2DEG;
            }
//            yDebug("vLimAdapted = %s",vLimAdapted.toString(3,3).c_str());
            //printMessage(2,"[reactCtrlThread::run()]: Will call solveIK.\n");
            double t_3=yarp::os::Time::now();
            q_dot = solveIK(ipoptExitCode); //this is the key function call where the reaching opt problem is solved 
            timeToSolveProblem_s  = yarp::os::Time::now()-t_3;

            if (ipoptExitCode==Ipopt::Maximum_CpuTime_Exceeded)
                yWarning("[reactCtrlThread] Ipopt cpu time was higher than the rate of the thread!");
            else if (ipoptExitCode!=Ipopt::Solve_Succeeded)
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
                }
            }
            
            updateArmChain(); //N.B. This is the second call within run(); may give more precise data for the logging; may also cost time
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

    if (state == STATE_REACH) {
        std::cout << yarp::os::Time::now()-t_0 << " ";
    }
    sendData();
    if (tactileCollisionPointsOn || visualCollisionPointsOn)
        vLimAdapted = vLimNominal; //if it was changed by the avoidanceHandler, we reset it
    printMessage(2,"[reactCtrlThread::run()] finished, state: %d.\n\n\n",state);
    if (state == STATE_REACH) {
        double t3 = yarp::os::Time::now();

        std::cout << t3-t_0 << " " << timeToSolveProblem_s << " " << t3-t2;
        if (t3-t2 > 0.02)
            std::cout <<" Alert!";
        std::cout << "\n";
    }
}

void reactCtrlThread::threadRelease()
{
    
    yInfo("Putting back original interaction modes."); 
    iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesOrig.data());
    jointsToSetInteractionA.clear();
    interactionModesNew.clear();
    interactionModesOrig.clear();
    
    yInfo("threadRelease(): deleting arm and torso encoder arrays and arm object.");
    delete encsA; encsA = nullptr;
    delete encsT; encsT = nullptr;
    delete   arm;   arm = nullptr;
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

    additionalControlPointsVector.clear();
    if(minJerkTarget != nullptr){
        yDebug("deleting minJerkTarget..");
        delete minJerkTarget;
        minJerkTarget = nullptr;
    }
  
    if(virtualArm != nullptr){
        yDebug("deleting virtualArm..");
        delete virtualArm;
        virtualArm = nullptr;
        virtualArmChain = nullptr;
    }
         
    if(I != nullptr){
        yDebug("deleting integrator I..");
        delete I;
        I = nullptr;
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
}


bool reactCtrlThread::enableTorso()
{
   // std::lock_guard<std::mutex> lg(mut);
    if (state==STATE_REACH)
    {
        return false;
    }

    useTorso=true;
    for (int i = 0; i < 3; i++)
    {
        arm->releaseLink(i);
    }
    vLimNominal(0,0)=vLimNominal(2,0)=-vMax;
    vLimAdapted(0,0)=vLimAdapted(2,0)=-vMax;

    vLimNominal(0,1)=vLimNominal(2,1)=vMax;
    vLimAdapted(0,1)=vLimAdapted(2,1)=vMax;
    alignJointsBounds();
    return true;
}

bool reactCtrlThread::disableTorso()
{
    //std::lock_guard<std::mutex> lg(mut);
    if (state==STATE_REACH)
    {
        return false;
    }

    useTorso=false;

    // disable torso pitch
    vLimNominal(0,0)=vLimNominal(0,1)=0.0;
    vLimAdapted(0,0)=vLimAdapted(0,1)=0.0;
    // disable torso roll
    vLimNominal(1,0)=vLimNominal(1,1)=0.0;
    vLimAdapted(1,0)=vLimAdapted(1,1)=0.0;
    // disable torso yaw
    vLimNominal(2,0)=vLimNominal(2,1)=0.0;
    vLimAdapted(2,0)=vLimAdapted(2,1)=0.0;
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

double reactCtrlThread::getTol() const
{
    return tol;
}

bool reactCtrlThread::setVMax(const double _vMax)
{
    if (_vMax>=0.0)
    {
        vMax=_vMax;
        for (size_t r=0; r<chainActiveDOF; r++)
        {
            vLimNominal(r,0)=-vMax;
            vLimAdapted(r,0)=-vMax;
            vLimNominal(r,1)=vMax;
            vLimAdapted(r,1)=vMax;
        }
        if (useTorso){
            vLimNominal(1,0)=vLimNominal(1,1)=0.0;
            vLimAdapted(1,0)=vLimAdapted(1,1)=0.0;
        }
        else
        {
            // disable torso pitch
            vLimNominal(0,0)=vLimNominal(0,1)=0.0;
            vLimAdapted(0,0)=vLimAdapted(0,1)=0.0;
            // disable torso roll
            vLimNominal(1,0)=vLimNominal(1,1)=0.0;
            vLimAdapted(1,0)=vLimAdapted(1,1)=0.0;
            // disable torso yaw
            vLimNominal(2,0)=vLimNominal(2,1)=0.0;
            vLimAdapted(2,0)=vLimAdapted(2,1)=0.0;
        }
        return true;
    }
    return false;
}

double reactCtrlThread::getVMax() const
{
    return vMax;
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
        if (start_experiment == 0) {
            t_0=Time::now();
        }
        start_experiment++;
        movingTargetCircle = _movingCircle;
        q_dot.zero();
        updateArmChain(); //updates chain, q and x_t
        virtualArmChain->setAng(q*CTRL_DEG2RAD); //with new target, we make the two chains identical at the start
        if(controlMode == "positionDirect")
           I->reset(q);   
                
        x_0=x_t;
        x_n=x_0;
        x_d=_x_d;
        if (start_experiment == 1) {
            x_d[0] = -0.289; x_d[1] = -0.164; x_d[2] = 0.1;
        }
        
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
    t_1 = yarp::os::Time::now();
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
    //std::lock_guard<std::mutex> lg(mut);
    bool stoppedOk = stopControlHelper();
    if (stoppedOk)
        yInfo("reactCtrlThread::stopControl(): Sucessfully stopped controllers");
    else
        yWarning("reactCtrlThread::stopControl(): Controllers not stopped sucessfully"); 
    return stoppedOk;
}

bool reactCtrlThread::stopControlAndSwitchToPositionMode()
{
    //std::lock_guard<std::mutex> lg(mut);
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


    Vector xr(6,0.0);
    xr.setSubvector(0,x_n);
    xr.setSubvector(3,o_n);

    /*Matrix H5real=(*(arm->asChain())).getH(5);
    Vector elbow_pos_real = H5real.getCol(3).subVector(0,2);
    //printf("[reactCtrlThread::solveIK]: real chain getH(%d) - elbow: \n %s \n",*(arm->asChain()).getDOF()-4-1,H5real.toString(3,3).c_str());
    printf("[reactCtrlThread::solveIK]: real p0 - elbow: (%s)\n",elbow_pos_real.toString(3,3).c_str());
    Matrix H5virtual=(*virtualArmChain).getH(5);
    Vector elbow_pos_virtual = H5virtual.getCol(3).subVector(0,2);
    //printf("[reactCtrlThread::solveIK]: real chain getH(%d) - elbow: \n %s \n",*(arm->asChain()).getDOF()-4-1,H5real.toString(3,3).c_str());
    printf("[reactCtrlThread::solveIK]: virtual p0 - elbow: (%s)\n",elbow_pos_virtual.toString(3,3).c_str());    */

    nlp->init(xr, q_dot, vLimAdapted);
    if (firstSolve) {
        _exit_code = app->OptimizeTNLP(GetRawPtr(nlp));
        firstSolve = false;
    } else {
        _exit_code = app->ReOptimizeTNLP(GetRawPtr(nlp));
    }
    Vector res=nlp->get_resultInDegPerSecond();
   
    // printMessage(0,"t_d: %g\tt_t: %g\n",t_d-t_0, t_t-t_0);
    if(verbosity >= 1){ 
        printf("x_n: %s\tx_d: %s\tdT %g\n",x_n.toString(3,3).c_str(),x_d.toString(3,3).c_str(),dT);
        printf("x_0: %s\tx_t: %s\n",       x_0.toString(3,3).c_str(),x_t.toString(3,3).c_str());
        printf("norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g\n",
                    norm(x_n-x_t), norm(x_d-x_n), norm(x_d-x_t));
        if(additionalControlPoints)
        {
            //additionalControlPointsVector.front(); //let's print the first one - assume it's the elbow for now
            printf("elbow_x_n == elbow_x_d: %s\telbow_x_t: %s\n", additionalControlPointsVector.front().x_desired.toString(3,3).c_str(), additionalControlPointsVector.front().p0.toString(3,3).c_str());
            printf("norm elbow pos error: %g\n",norm(additionalControlPointsVector.front().x_desired - additionalControlPointsVector.front().p0));
        }
        printf("Result (solved velocities (deg/s)): %s\n",res.toString(3,3).c_str());
    }
   
    return res;
}


 /**** kinematic chain, control, ..... *****************************/

void reactCtrlThread::updateArmChain()
{    
    iencsA->getEncoders(encsA->data());
    qA=encsA->subVector(0,NR_ARM_JOINTS-1);

    iencsT->getEncoders(encsT->data());
    qT[0]=(*encsT)[2];
    qT[1]=(*encsT)[1];
    qT[2]=(*encsT)[0];

    q.setSubvector(0,qT);
    q.setSubvector(NR_TORSO_JOINTS,qA);

    arm->setAng(q*CTRL_DEG2RAD);
    //H=arm->getH();
    //x_t=H.subcol(0,3,3);
    x_t = arm->EndEffPosition();
    o_t = arm->EndEffPose().subVector(3,5)*arm->EndEffPose()[6];
}

bool reactCtrlThread::alignJointsBounds()
{
    yDebug("[reactCtrlThread][alignJointsBounds] pre alignment:");
    printJointsBounds();

    deque<IControlLimits*> limits;
    limits.push_back(ilimT);
    limits.push_back(ilimA);

    if (!arm->alignJointsBounds(limits)) return false;

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

bool reactCtrlThread::areJointsHealthyAndSet(vector<int> &jointsToSet,
                                             const string &_p, const string &_s)
{
    jointsToSet.clear();
    vector<int> modes(jntsA);
    if (_p=="arm")
    {
        modes.resize(NR_ARM_JOINTS,VOCAB_CM_IDLE);
        imodA->getControlModes(modes.data());
    }
    else if (_p=="torso")
    {
        modes.resize(NR_TORSO_JOINTS,VOCAB_CM_IDLE);
        imodT->getControlModes(modes.data());
    }
    else
        return false;
    
    for (int i=0; i<modes.size(); i++)
    {
        if (arm->isLinkBlocked(i))  //TODO in addition, one might check if some joints are blocked like here:  ServerCartesianController::areJointsHealthyAndSet
            continue;
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
        for (int mode : modes){
                printf("%s ",Vocab::decode(mode).c_str());
        }
        printf("\n");
        printf("Indexes of joints to set: ");
        for (int joint : jointsToSet){
                printf("%d ",joint);
        }
        printf("\n");
    }
    
    return true;
}

bool reactCtrlThread::setCtrlModes(const vector<int> &jointsToSet,
                                   const string &_p, const string &_s)
{
    if (_s!="position" && _s!="velocity" && _s!="positionDirect")
        return false;

    if (jointsToSet.empty())
        return true;

    vector<int> modes;
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
        imodA->setControlModes((int)jointsToSet.size(),
                               jointsToSet.data(),
                               modes.data());
    }
    else if (_p=="torso")
    {
        imodT->setControlModes((int)jointsToSet.size(),
                               jointsToSet.data(),
                               modes.data());
    }
    else
        return false;

    return true;
}

//N.B. the targetValues can be either positions or velocities, depending on the control mode!
bool reactCtrlThread::controlArm(const string& _controlMode, const yarp::sig::Vector &_targetValues)
{   
    vector<int> jointsToSetA;
    vector<int> jointsToSetT;
    if (!areJointsHealthyAndSet(jointsToSetA,"arm",_controlMode))
    {
        yWarning("[reactCtrlThread::controlArm] Stopping control because arm joints are not healthy!");
        stopControlHelper();
        return false;
    }
 
    if (!areJointsHealthyAndSet(jointsToSetT,"torso",_controlMode))
    {
        yWarning("[reactCtrlThread::controlArm] Stopping control because torso joints are not healthy!");
        stopControlHelper();
        return false;
    }
    
    if (!setCtrlModes(jointsToSetA,"arm",_controlMode))
    {
        yError("[reactCtrlThread::controlArm] I am not able to set the arm joints to %s mode!",_controlMode.c_str());
        return false;
    }   

    if (!setCtrlModes(jointsToSetT,"torso",_controlMode))
    {
        yError("[reactCtrlThread::controlArm] I am not able to set the torso joints to %s mode!",_controlMode.c_str());
        return false;
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
    if (_controlMode == "velocity")
    {
        printMessage(1,"[reactCtrlThread::controlArm] Joint velocities (iKin order, deg/s): %s\n",_targetValues.toString(3,3).c_str());

        Vector velsT(TORSO_DOF,0.0);
        velsT[0] = _targetValues[2]; //swapping pitch and yaw as per iKin vs. motor interface convention
        velsT[1] = _targetValues[1];
        velsT[2] = _targetValues[0]; //swapping pitch and yaw as per iKin vs. motor interface convention

        printMessage(2,"    velocityMove(): torso (swap pitch & yaw): %s\n",velsT.toString(3,3).c_str());
        ivelT->velocityMove(velsT.data());
        ivelA->velocityMove(_targetValues.subVector(3,9).data()); //indexes 3 to 9 are the arm joints velocities
    }
    else if(_controlMode == "positionDirect")
    {
        printMessage(1,"[reactCtrlThread::controlArm] Target joint positions (iKin order, deg): %s\n",_targetValues.toString(3,3).c_str());
        Vector posT(3,0.0);
        posT[0] = _targetValues[2]; //swapping pitch and yaw as per iKin vs. motor interface convention
        posT[1] = _targetValues[1];
        posT[2] = _targetValues[0]; //swapping pitch and yaw as per iKin vs. motor interface convention

        printMessage(2,"    positionDirect: torso (swap pitch & yaw): %s\n",posT.toString(3,3).c_str());
        iposDirT->setPositions(posT.data());
        iposDirA->setPositions(_targetValues.subVector(3,9).data()); //indexes 3 to 9 are the arm joints
    }
        
    return true;
}


bool reactCtrlThread::stopControlHelper()
{
    return ivelA->stop() && ivelT->stop();
}


bool reactCtrlThread::stopControlAndSwitchToPositionModeHelper()
{
    state=STATE_IDLE;
    
    vector<int> jointsToSetA;
    jointsToSetA.push_back(0);jointsToSetA.push_back(1);jointsToSetA.push_back(2);jointsToSetA.push_back(3);jointsToSetA.push_back(4);
    jointsToSetA.push_back(5);jointsToSetA.push_back(6);

    ivelA->stop();
    ivelT->stop();
    vector<int> jointsToSetT;
    jointsToSetT.push_back(0);jointsToSetT.push_back(1);jointsToSetT.push_back(2);
    return  setCtrlModes(jointsToSetA,"arm","position") && setCtrlModes(jointsToSetT,"torso","position");
}



/***************** auxiliary computations  *******************************/
 
Vector reactCtrlThread::getPosMovingTargetOnCircle()
{
      Vector _x_d=circleCenter; 
      //x-coordinate will stay constant; we set y, and z
      _x_d[1]+=radius*cos(2.0*M_PI*frequency*(yarp::os::Time::now()-t_1));
      _x_d[2]+=radius*sin(2.0*M_PI*frequency*(yarp::os::Time::now()-t_1));

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
    return getPeriod()*J_cst*q_dot;
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
}

void reactCtrlThread::convertPosFromLinkToRootFoR(const Vector &pos,const SkinPart skinPart, Vector &outPos)
{
    Matrix T_root_to_link = yarp::math::zeros(4,4);
    int torsoDOF = 3;
    T_root_to_link = arm->getH(SkinPart_2_LinkNum[skinPart].linkNum + (skinPart != SKIN_FRONT_TORSO)*torsoDOF);
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
}

 /**** communication through ports in/out ****************/


bool reactCtrlThread::getCollisionPointsFromPort(BufferedPort<Bottle> &inPort, double gain, const string& which_chain,std::vector<collisionPoint_t> &collPoints)
{
    //printMessage(9,"[reactCtrlThread::getCollisionPointsFromPort].\n");
    collisionPoint_t collPoint;    
    SkinPart sp;
    
    collPoint.skin_part = SKIN_PART_UNKNOWN;
    collPoint.x.resize(3,0.0);
    collPoint.n.resize(3,0.0);
    collPoint.magnitude=0.0;
    
    Bottle* collPointsMultiBottle = inPort.read(false);
    if(collPointsMultiBottle != nullptr){
         printMessage(5,"[reactCtrlThread::getCollisionPointsFromPort]: There were %d bottles on the port.\n",collPointsMultiBottle->size());
         for(int i=0; i< collPointsMultiBottle->size();i++){
             Bottle* collPointBottle = collPointsMultiBottle->get(i).asList();
             printMessage(5,"Bottle %d contains %s \n", i,collPointBottle->toString().c_str());
             sp =  (SkinPart)(collPointBottle->get(0).asInt());
             //we take only those collision points that are relevant for the chain we are controlling
             if( ((which_chain == "left") && ( (sp==SKIN_LEFT_HAND) || (sp==SKIN_LEFT_FOREARM) || (sp==SKIN_LEFT_UPPER_ARM)) ) || (sp==SKIN_FRONT_TORSO)
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
    }
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
            b.addInt(static_cast<int>(chainActiveDOF));
            //position
            //cols 2-4: the desired final target (for end-effector)
            vectorIntoBottle(x_d,b);
            // 5:7 the end effector position in which the robot currently is
            vectorIntoBottle(x_t,b);
            // 8:10 the current desired target given by the reference generation (particle / minJerk) (for end-effector)
            vectorIntoBottle(x_n,b);
            
            //orientation
            //cols 11-13: the desired final orientation (for end-effector)
            vectorIntoBottle(o_d,b);
            // 14:16 the end effector orientation in which the robot currently is
            vectorIntoBottle(o_t,b);
            // 17:19 the current desired orientation given by referenceGen (currently not supported - equal to o_d)
            vectorIntoBottle(o_n,b);
                        
            //variable - if torso on: 20:29: joint velocities as solution to control and sent to robot 
            vectorIntoBottle(q_dot,b); 
            //variable - if torso on: 30:39: actual joint positions 
            vectorIntoBottle(q,b); 
            //variable - if torso on: 40:59; joint vel limits as input to ipopt, after avoidanceHandler,
            matrixIntoBottle(vLimAdapted,b); // assuming it is row by row, so min_1, max_1, min_2, max_2 etc.
            b.addInt(ipoptExitCode);
            b.addDouble(timeToSolveProblem_s);
            if (controlMode == "positionDirect")
                //joint pos from virtual chain IPopt is operating onl variable - if torso on: 60:69
                vectorIntoBottle(qIntegrated,b);
            if(additionalControlPoints && (!additionalControlPointsVector.empty()))
            {
               //we assume there will be only one - elbow - now
               //desired elbow position; variable - if torso on and positionDirect: 70:72 ;
               vectorIntoBottle( (*additionalControlPointsVector.begin()).x_desired , b);
            }
            else
            {
               Vector elbow_d(3,0.0);
               vectorIntoBottle( elbow_d , b);
            }

            //actual elbow position on real chain; variable - if torso on and positionDirect: 73:75 ;
            vectorIntoBottle( (*(arm->asChain())).getH((*(arm->asChain())).getDOF()-4-1).getCol(3).subVector(0,2) , b);

                                      
            outPort.setEnvelope(ts);
            outPort.write(b);
        }
    }
}

bool reactCtrlThread::readMotionPlan(std::vector<Vector> &x_desired)
{
    Bottle *inPlan = streamedTargets.read(false);

    bool hasPlan = false;
    if(inPlan!=nullptr) {
        yInfo() << "received motionPlan";
        size_t nbCtrlPts = inPlan->size();

        for (size_t i=0; i<nbCtrlPts; i++)
        {
            if (Bottle* inListTraj = inPlan->get(i).asList())
            {
                string ctrlPtName = inListTraj->find("control-point").asString();
                if (inListTraj->find("number-waypoints").asInt()>0)
                {
                    int nDim = inListTraj->find("number-dimension").asInt();
                    if (nDim>=3)
                    {
                        Vector xCtrlPt(nDim, 0.0);
                        if (Bottle* coordinate = inListTraj->find("waypoint_0").asList())
                        {
                            if (coordinate->size()==nDim)
                            {
                                for (size_t k=0; k<nDim; k++)
                                    xCtrlPt[k]=coordinate->get(k).asDouble();
                                yInfo("\tControl point of %s\t: %s\n",ctrlPtName.c_str(),xCtrlPt.toString(3,3).c_str());
                                x_desired.push_back(xCtrlPt);
                                hasPlan = true;
                            }
                        }
                    }
                }
            }
        }
    }
    return hasPlan;
}

/**** visualizations using iCubGui **************************************/


void reactCtrlThread::sendiCubGuiObject(const string& object_type)
{
    if (outPortiCubGui.getOutputCount()>0)
    {
        Bottle obj;
        if (object_type == "particle")
        {
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
        else if(object_type == "target")
        {
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
        else if(object_type == "additionalTargets")
        {        
            for (const auto & controlPoint : additionalControlPointsVector)
            {
                obj.addString("object");
                obj.addString("extra-target");
     
                // size 
                obj.addDouble(30.0);
                obj.addDouble(30.0);
                obj.addDouble(30.0);
        
                // positions - iCubGui works in mm
                obj.addDouble(1000 * controlPoint.x_desired(0));
                obj.addDouble(1000 * controlPoint.x_desired(1));
                obj.addDouble(1000 * controlPoint.x_desired(2));
        
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
       }
       outPortiCubGui.write(obj);
        
    }
}

void reactCtrlThread::deleteiCubGuiObject(const string& object_type)
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

void reactCtrlThread::createStaticSphere(double _radius, const Vector &pos)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("ssph");
    cmd.addDouble(_radius);
    
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
    cmd.addDouble(0.01); cmd.addDouble(0.01); cmd.addDouble(0.01); //fixed size
    
    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    // color
    cmd.addInt(1);cmd.addInt(1);cmd.addInt(0); //blue
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
    size_t nrCollisionPoints = collisionPoints.size(); //+avhdl->selfColPoints.size();
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
    for(const auto & collisionPoint : collisionPoints) {
        convertPosFromLinkToRootFoR(collisionPoint.x,collisionPoint.skin_part,posRoot);
        convertPosFromRootToSimFoR(posRoot,posSim);
        moveBox(j,posSim); //just move a box from the sim world
        j++;
        posRoot.zero(); posSim.zero();
    }


//    for(const auto & collisionPoint : avhdl->selfColPoints) {
//        convertPosFromLinkToRootFoR(collisionPoint.x,SKIN_FRONT_TORSO,posRoot);
//        convertPosFromRootToSimFoR(posRoot,posSim);
//        moveBox(j,posSim); //just move a box from the sim world
//        j++;
//        posRoot.zero(); posSim.zero();
//    }
    
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
