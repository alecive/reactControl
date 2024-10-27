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

#include "reactCtrlThread.h"

#define TACTILE_INPUT_GAIN 0.6
#define VISUAL_INPUT_GAIN 0.6 // changed from 0.8 in sim to 0.6 for realsense obstacles
#define PROXIMITY_INPUT_GAIN 0.8

enum {
    STATE_WAIT,
    STATE_REACH,
    STATE_IDLE
};


ArmInterface::ArmInterface(std::string _part, double _selfColPoints, const std::string& refGen, double _dT, bool _main):
        I(nullptr), part_name(std::move(_part)), useSelfColPoints(_selfColPoints), referenceGen(refGen), dT(_dT),
        mainPart(_main), iencsA(nullptr), iposDirA(nullptr), imodA(nullptr), iintmodeA(nullptr), iimpA(nullptr),
        ilimA(nullptr), encsA(nullptr), arm(nullptr), jntsA(0), chainActiveDOF(0), virtualArm(nullptr), avhdl(nullptr),
        fingerPos({80.,6.,57.,13.,0.,13.,0.,103.}), homePos({0., 0., 0., -34., 30., 0., 50., 0.,  0., 0.}),
        avoidance(false), notMovingCounter(0), minJerkTarget(nullptr), filter(nullptr), useSampling(true)
{
    if (part_name=="left_arm")
    {
        part_short="left";
    }
    else if (part_name=="right_arm")
    {
        part_short="right";
    }

    /******** iKin chain and variables, and transforms init *************************/
    arm = new iCub::iKin::iCubArm(part_short+"_v2");
    // Release / block torso links (blocked by default)
    for (int i = 0; i < NR_TORSO_JOINTS; i++)
    {
        arm->releaseLink(i);
    }
    //we set up the variables based on the current DOF - that is without torso joints if torso is blocked
    chainActiveDOF = arm->getDOF();

    //N.B. All angles in this thread are in degrees
    qA.resize(NR_ARM_JOINTS,0.0); //current values of arm joints (should be 7)
    q.resize(chainActiveDOF,0.0); //current joint angle values (10 if torso is on, 7 if off)
    q_last.resize(chainActiveDOF,0.0); //current joint angle values (10 if torso is on, 7 if off)
    qIntegrated.resize(chainActiveDOF,0.0); //joint angle pos predictions from integrator

    q_dot.resize(chainActiveDOF,0.0);
    vLimNominal.resize(chainActiveDOF,2);
    vLimAdapted.resize(chainActiveDOF,2);

    Time::delay(1);

    x_0.resize(3,0.0);
    x_t.resize(3,0.0);
    x_n.resize(3,0.0);
    x_d.resize(3,0.0);
    //store the home position
    // Vector pose = arm->EndEffPose();
    // x_home = pose.subVector(0,2);
    // o_home = pose.subVector(3,5)*pose(6);
    x_home = (part_short == "left")? Vector{-0.304, -0.202, 0.023}:Vector{-0.304, 0.202, 0.023};
    o_home = (part_short == "left")? Vector{-0.025, 0.603, -0.798, 2.838}:Vector{-0.152, -0.789, 0.596, 3.094};

    //palm facing inwards
    o_0.resize(4,0.0); o_0(1)=-0.707; o_0(2)=+0.707; o_0(3) = M_PI;
    //palm facing down
//    o_0 = (part_short == "right")? Vector{0.0, 1.0, 0.0, -0.95*M_PI} : Vector{0.0, 0.0, 1.0, M_PI};
    o_t = o_0;
    o_n = o_0;
    o_d = o_0;
}

void ArmInterface::setVMax(bool useTorso, double vMax)
{
    for (size_t r=0; r<chainActiveDOF; r++)
    {
        vLimNominal(r,0)=-vMax;
        vLimAdapted(r,0)=-vMax;
        vLimNominal(r,1)=vMax;
        vLimAdapted(r,1)=vMax;
    }
    if (!useTorso) // TODO chainActiveDOF is still 10
    {
        vLimAdapted.setSubcol({0.0,0.0,0.0}, 0,0);
        vLimNominal.setSubcol({0.0,0.0,0.0}, 0,0);
        vLimAdapted.setSubcol({0.0,0.0,0.0}, 0,1);
        vLimNominal.setSubcol({0.0,0.0,0.0}, 0,1);
    }
}

void ArmInterface::updateCollPoints()
{
    for (auto& colP : collisionPoints)
    {
        colP.duration -= dT;
//        if (colP.duration < 0.9)
            colP.magnitude *= 0.9; // vision 0.6; tactile 0.9
        if (colP.magnitude < 1e-3) colP.duration = -1;
    }
    collisionPoints.erase(std::remove_if(collisionPoints.begin(), collisionPoints.end(),
                                         [](const collisionPoint_t & colP) { return colP.duration <= 0; }),
                          collisionPoints.end());
}

bool ArmInterface::prepareDrivers(const std::string& robot, const std::string& name, bool stiffInteraction)
{
    yarp::os::Property OptA;
    OptA.put("robot",  robot);
    OptA.put("part",   part_name);
    OptA.put("device", "remote_controlboard");
    OptA.put("remote", "/"+robot+"/"+part_name);
    OptA.put("local",  "/"+name +"/"+part_name);
    if (!ddA.open(OptA))
    {
        yError("[reactCtrlThread]Could not open %s PolyDriver!",part_name.c_str());
        return false;
    }

    bool okA = false;

    if (ddA.isValid())
    {
        okA = ddA.view(iencsA) && ddA.view(iposDirA) && ddA.view(imodA)
           && ddA.view(ilimA) && ddA.view(iintmodeA) && ddA.view(iimpA);
    }
    iencsA->getAxes(&jntsA);
    printf("Joints A is %d\n", jntsA);
    encsA = new yarp::sig::Vector(jntsA,0.0);

    if (!okA)
    {
        yError("[reactCtrlThread]Problems acquiring %s interfaces!!!!", part_name.c_str());
        return false;
    }
    interactionModesOrig.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_STIFF);
    jointsToSetInteractionA.clear();
    for (int i=0; i<NR_ARM_JOINTS_FOR_INTERACTION_MODE;i++)
    {
        jointsToSetInteractionA.push_back(i);
    }
    iintmodeA->getInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesOrig.data());
    if(stiffInteraction)
    {
        interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_STIFF);
        iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesNew.data());
    }
    else // not working
    {
        interactionModesNew.resize(NR_ARM_JOINTS_FOR_INTERACTION_MODE,VOCAB_IM_COMPLIANT);
        iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesNew.data());
        iimpA->setImpedance(0,0.4,0.03);
        iimpA->setImpedance(1,0.4,0.03);
        iimpA->setImpedance(2,0.4,0.03);
        iimpA->setImpedance(3,0.2,0.01);
        iimpA->setImpedance(4,0.05,0.0);
    }
    return true;
}

void ArmInterface::release()
{
    yInfo("Putting back original interaction modes.");
    iintmodeA->setInteractionModes(NR_ARM_JOINTS_FOR_INTERACTION_MODE,jointsToSetInteractionA.data(),interactionModesOrig.data());
    interactionModesNew.clear();
    interactionModesOrig.clear();
    jointsToSetInteractionA.clear();

    yInfo("threadRelease(): deleting arm and torso encoder arrays and arm object.");
    delete encsA; encsA = nullptr;
    delete   arm;   arm = nullptr;
    yInfo("Closing controllers..");
    ddA.close();
    collisionPoints.clear();

    if(virtualArm != nullptr)
    {
        yDebug("deleting virtualArm..");
        delete virtualArm;
        virtualArm = nullptr;
    }
    if(I != nullptr)
    {
        yDebug("deleting integrator I..");
        delete I;
        I = nullptr;
    }

    if(minJerkTarget != nullptr)
    {
        yDebug("deleting minJerkTarget..");
        delete minJerkTarget;
        minJerkTarget = nullptr;
    }
    delete filter; filter = nullptr;
}

void ArmInterface::updateArm(const Vector& qT)
{
    iencsA->getEncoders(encsA->data());
    qA = encsA->subVector(0,NR_ARM_JOINTS-1);
    q_last = q;
    q.setSubvector(0,qT);
    q.setSubvector(NR_TORSO_JOINTS,qA);
    //yDebug() << norm(q-q_last)  << "\n";
    if (norm(q-q_last) < 0.002)
    {
        notMovingCounter++;
    }
    else
    {
        notMovingCounter = 0;
    }
    arm->setAng(q*CTRL_DEG2RAD);
    x_t = arm->EndEffPosition();
    o_t = arm->EndEffPose().subVector(3,6);
}

bool ArmInterface::alignJointsBound(IControlLimits* ilimT)
{
    yDebug("[reactCtrlThread][alignJointsBound] pre alignment:");
    printJointsBounds();

    std::deque<IControlLimits*> limits;
    limits.push_back(ilimT);
    limits.push_back(ilimA);
    if (!arm->alignJointsBounds(limits)) return false;
    arm->asChain()->operator()(1).setMin(CTRL_DEG2RAD*-5); // restrict torso roll
    arm->asChain()->operator()(1).setMax(CTRL_DEG2RAD*5); // restrict torso roll
//    arm->asChain()->operator()(0).setMin(CTRL_DEG2RAD*-15); // restrict torso pitch
//    arm->asChain()->operator()(0).setMax(CTRL_DEG2RAD*20); // restrict torso pitch
    yDebug("[reactCtrlThread][alignJointsBound] post alignment:");
    printJointsBounds();
    if (virtualArm == nullptr)
    {
        virtualArm = new iCubArm(*arm);  //Creates a new Limb from an already existing Limb object - but they will be too independent limbs from now on
    }
    return true;
}

void ArmInterface::printJointsBounds() const
{
    iCub::iKin::iKinChain &chain=*arm->asChain();
    for (size_t i = 0; i < chainActiveDOF; i++)
    {
        const double min=chain(i).getMin()*CTRL_RAD2DEG;
        const double max=chain(i).getMax()*CTRL_RAD2DEG;
        yDebug("[jointsBounds (deg)] i: %lu\tmin: %g\tmax %g",i,min,max);
    }
}

void ArmInterface::initialization(iKinChain* chain, iKinChain* torso,  int verbosity)
{
    //set grasping pose for fingers
//    for (size_t j=0; j<fingerPos.size(); j++)
//    {
//        imodA->setControlMode(8+j, VOCAB_CM_POSITION_DIRECT);
//        iposDirA->setPosition(8+j, fingerPos[j]);
//
//    }
    if(referenceGen == "minJerk")
    {
        minJerkTarget = new minJerkTrajGen(3,dT,1.0); //dim 3, dT, trajTime 1s - will be overwritten later
    }

    lim.resize(chainActiveDOF,2); //joint pos limits
    //filling joint pos limits Matrix
    iKinChain& armChain=*arm->asChain();
    for (size_t jointIndex=0; jointIndex<chainActiveDOF; jointIndex++)
    {
        lim(jointIndex,0)= CTRL_RAD2DEG*armChain(jointIndex).getMin();
        lim(jointIndex,1)= CTRL_RAD2DEG*armChain(jointIndex).getMax();
    }

    filter = new LPFilterSO3(o_home, dT, 1.);
    I = new Integrator(dT,q,lim);
    avhdl = std::make_unique<AvoidanceHandler>(*virtualArm->asChain(), collisionPoints, chain,
                                                      useSelfColPoints, part_short, encsA, torso, verbosity);
}

bool ArmInterface::checkRecoveryPath(Vector& next_x)
{
    if (!avoidance && !last_trajectory.empty())
    {
        if (norm(next_x-x_t) < 0.08 || norm(next_x-x_t) < norm(last_trajectory.back()-next_x))
        {
            last_trajectory.clear();
//            vLimAdapted *= 0.0;
//            slowCounter = 50;
            yInfo() << "Close to target position, recovery path aborted.\n";
        }
        else
        {
            next_x = last_trajectory.back();
            last_trajectory.pop_back();
            vLimAdapted *= 0.7;

            yInfo() << last_trajectory.size() <<  "; Recovery path " << next_x.toString(3,3) << "\n";
        }
        return true;
    }
    return false;
}

void ArmInterface::updateRecoveryPath()
{
    if (avoidance && (last_trajectory.empty() || norm(last_trajectory.back()-x_t) > 0.005))
    {
        last_trajectory.push_back(x_t);
        printf("Last traj push back\n");
//        avoidance = false;
    }
    avoidance = avhdl->existsCtrlPoint();
}


void ArmInterface::updateNextTarget(bool& vel_limited)
{
    Vector next_x = x_d;
    o_n = o_d;
    if (referenceGen == "minJerk" && useSampling)
    {
        minJerkTarget->computeNextValues(x_d);
        next_x = minJerkTarget->getPos();
        o_n = filter->next_value(o_d);
    }

    bool res = checkRecoveryPath(next_x);
//    if (res && last_trajectory.empty() && referenceGen == "minJerk")
//    {
//        yInfo() << next_x.toString(3);
//        minJerkTarget->init(x_t); //initial pos
//        minJerkTarget->setTs(dT); //time step
//        const double T = norm(x_d - x_t)  / 0.02;
//        minJerkTarget->setT(T);
//        minJerkTarget->computeNextValues(x_d);
//        next_x = minJerkTarget->getPos();
//        yInfo() << next_x.toString(3) << "\t T = " << T;
//        res = false;
//    }
    x_n = next_x;
    vel_limited = vel_limited | res;
}

void ArmInterface::resetTarget(const yarp::sig::Vector& _x_d, const yarp::sig::Vector& _o_d, double trajSpeed)
{
    q_dot.zero();
    virtualArm->setAng(q*CTRL_DEG2RAD); //with new target, we make the two chains identical at the start
    I->reset(q);
    o_0=o_t;
    o_d = _o_d;
    o_n = o_0;
    x_0=x_t;
    x_n=x_0;
    x_d=_x_d;
    notMovingCounter = 0;
    const double T = norm(x_d - x_0)  / trajSpeed;
    if(referenceGen == "minJerk"){
        minJerkTarget->init(x_0); //initial pos
        minJerkTarget->setTs(dT); //time step
        // calculate the time to reach from the distance to target and desired velocity
        minJerkTarget->setT(std::ceil(T * 10.0) / 10.0);
    }
    filter->reset(o_t,T);
}

/*********** public methods ****************************************************************************/

reactCtrlThread::reactCtrlThread(int _rate, std::string _name, std::string _robot,  const std::string& _part,
                                 const std::string& second_part, int _verbosity, bool _disableTorso,
                                 double _trajSpeed, double _globalTol, double _vMax, double _tol, double _timeLimit,
                                 std::string _referenceGen, bool _tactileCPOn, bool _visualCPOn, bool _proximityCPOn,
                                 bool _gazeControl, bool _stiffInteraction,
                                 bool _hittingConstraints, bool _orientationControl,
                                 bool _visTargetInSim, bool _visParticleInSim, bool _visCollisionPointsInSim,
                                 particleThread *_pT, double _restPosWeight, double _selfColPoints) :
        PeriodicThread(static_cast<double>(_rate)/1000.0), name(std::move(_name)), robot(std::move(_robot)),
        verbosity(_verbosity), useTorso(!_disableTorso), trajSpeed(_trajSpeed), globalTol(_globalTol), vMax(_vMax),
        tol(_tol), timeLimit(_timeLimit), referenceGen(std::move(_referenceGen)), tactileCollPointsOn(_tactileCPOn),
        visualCollPointsOn(_visualCPOn), proximityCollPointsOn(_proximityCPOn), gazeControl(_gazeControl),
        stiffInteraction(_stiffInteraction), hittingConstraints(_hittingConstraints), main_arm_constr(true),
        orientationControl(_orientationControl), visualizeCollisionPointsInSim(_visCollisionPointsInSim), counter(0),
        restPosWeight(_restPosWeight), state(STATE_WAIT), iencsT(nullptr), iposDirT(nullptr), imodT(nullptr),
        ilimT(nullptr), encsT(nullptr), jntsT(0), igaze(nullptr), contextGaze(0), movingTargetCircle(false), radius(0),
        frequency(0), streamingTarget(false), t_0(0), solverExitCode(0), timeToSolveProblem_s(0), comingHome(false),
        holding_position(false), visuhdl(verbosity, false, name, _visTargetInSim,
                                         referenceGen != "none" && _visParticleInSim)
{
    dT=getPeriod();
    obsWorldPos.resize(109);
    for (int k = 0; k < 109; k++)
    {
        obsWorldPos[k].resize(3,0.0);
    }

    prtclThrd=_pT;  //in case of referenceGen != uniformParticle, NULL will be received
    /******** iKin chain and variables, and transforms init *************************/
    main_arm =std::make_unique<ArmInterface>(_part, _selfColPoints, referenceGen, dT);
    second_arm = (second_part=="left_arm" || second_part=="right_arm")? std::make_unique<ArmInterface>(second_part, _selfColPoints, referenceGen, dT, false) : nullptr;
}

bool reactCtrlThread::threadInit()
{
    torso = new iCubTorso();
    printMessage(2,"[reactCtrlThread] threadInit()\n");

    //N.B. All angles in this thread are in degrees
    qT.resize(NR_TORSO_JOINTS,0.0); //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)
    setVMax(vMax);

    if (!prepareDrivers()) return false;

    circleCenter.resize(3,0.0);
    circleCenter(0) = -0.3; //for safety, we assign the x-coordinate on in it within iCub's reachable space
    Time::delay(1);
    updateArmChain();
    printf("%s\n", main_arm->arm->EndEffPose().toString(3).c_str());
    if (second_arm) printf("%s\n", second_arm->arm->EndEffPose().toString(3).c_str());
    main_arm->initialization(second_arm? second_arm->virtualArm->asChain() : nullptr, torso->asChain(), verbosity);
    if (second_arm) second_arm->initialization(main_arm->virtualArm->asChain(), torso->asChain(), verbosity);
    NeoObsInPort.open("/"+name+"/neo_obstacles:i");
    solver = std::make_unique<QPSolver>(main_arm->virtualArm, hittingConstraints,
                                        second_arm? second_arm->virtualArm : nullptr,
                                        vMax, orientationControl,dT,
                                        main_arm->homePos*CTRL_DEG2RAD, restPosWeight, main_arm->part_short);
    aggregPPSeventsInPort.open("/"+name+"/pps_events_aggreg:i");
    aggregSkinEventsInPort.open("/"+name+"/skin_events_aggreg:i");
    proximityEventsInPort.open("/"+name+"/proximity_events:i");
    proximityEvents2InPort.open("/"+name+"/proximity_events2:i");
//    streamedTargets.open("/"+name+"/streamedWholeBodyTargets:i");
    streamedTargets.open("/"+name+"/streamedTargets:i");

    outPort.open("/"+name +"/data:o"); //for dumping
    outObsPort.open("/"+name +"/obsdata:o"); //for dumping
    sensManagerPort.open("/"+name +"/sensManager:i"); //for dumping
    movementFinishedPort.open("/" + name + "/finished:o");
    proximityEventsVisuPort.open("/"+name+"/proximity:o");
    proximityEventsForiCubGuiPort.open("/"+name+"/prox_gui:o");

    writeConfigData();
    printMessage(5,"[reactCtrlThread] threadInit() finished.\n");
    yarp::os::Time::delay(0.2);
    t_1=Time::now();

    return true;
}

void reactCtrlThread::writeConfigData()
{
    fout_param.open("param.log");

    fout_param<<main_arm->part_short<<" "<<main_arm->chainActiveDOF<<" "<<main_arm->useSelfColPoints<<" ";
    for (size_t i=0; i<main_arm->chainActiveDOF; i++)
    {
        fout_param<<main_arm->lim(i,0)<<" "<<main_arm->lim(i,1)<<" ";
    }
    fout_param<<vMax<<" "<<trajSpeed<<" "<<tol<<" "<<globalTol<<" "<<dT<<" "<<timeLimit<<" "<<stiffInteraction<<" ";
    fout_param<<tactileCollPointsOn<< " "<<visualCollPointsOn<<" "<<proximityCollPointsOn<<" ";
    fout_param<<hittingConstraints<<" "<<restPosWeight<<" "<<orientationControl<<" "<<gazeControl<<" ";
    if (second_arm)
    {
        fout_param<<second_arm->part_short<<" "<<second_arm->chainActiveDOF<<" "<<second_arm->useSelfColPoints<<" ";
        for (size_t j = 0; j < second_arm->chainActiveDOF; j++)
        {
            fout_param << second_arm->lim(j, 0) << " " << second_arm->lim(j, 1) << " ";
        }
    }
    fout_param<<std::endl;
    yInfo("Written to param file and closing..");
    fout_param.close();
}

bool reactCtrlThread::prepareDrivers()
{
    /*****  Drivers, interfaces, control boards etc. ***********************************************************/
    bool ret = main_arm->prepareDrivers(robot, name, stiffInteraction);

    if (second_arm) ret = ret && second_arm->prepareDrivers(robot, name, stiffInteraction);
    if (!ret)
    {
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

    bool okT = false;

    if (ddT.isValid())
    {
        okT = ddT.view(iencsT) && ddT.view(iposDirT) && ddT.view(imodT) && ddT.view(ilimT);
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

    if(gazeControl)
    {
        Property OptGaze;
        OptGaze.put("device","gazecontrollerclient");
        OptGaze.put("remote","/iKinGazeCtrl");
        OptGaze.put("local","/"+name+"/gaze");

        if ((!ddG.open(OptGaze)) || (!ddG.view(igaze)))
        {
            yError(" could not open the Gaze Controller!");
            return false;
        }

        igaze->storeContext(&contextGaze);
        igaze->setSaccadesMode(false);
        igaze->setNeckTrajTime(1.0);
        igaze->setEyesTrajTime(0.5);
        igaze->blockEyes();
    }
    return true;
}

void reactCtrlThread::insertTestingCollisions()
{
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

    if (t_1 > 0 && robot == "icubSim")
    {
        bool active = false;
        collisionPoint_t collisionPointStruct{SKIN_LEFT_FOREARM, TACTILE_OBS};
        if (yarp::os::Time::now() - t_1 > 5. && counter < 100)
        {
            collisionPointStruct.x = {-0.027228, -0.054786, -0.0191051}; // {0.018, 0.095, 0.024};
            collisionPointStruct.n = {-0.886, 0.14, -0.431 }; // {0.568, -0.18, 0.406};
            counter++;
            active = true;
        }
        if (yarp::os::Time::now() - t_1 > 20. && counter < 250)
        {
            //            collisionPointStruct.x = {-0.0002, -0.0131,-0.0258434};//  {-0.02, 0.0, -0.005}; //  // {-0.031, -0.079, 0.005};
            //            collisionPointStruct.n = {-0.005, 0.238, -0.971}; //{0,0,-1}; // // {-0.739, 0.078, 0.105};
            collisionPointStruct.skin_part = SKIN_LEFT_HAND;
            collisionPointStruct.x = {-0.02, 0, 0.01};
            collisionPointStruct.n = {0, 0, 1};
            counter++;
            active = true;
        }
//        else if (yarp::os::Time::now() - t_1 > 25. && counter < 250)
//        {
//            collisionPointStruct.x = {0.026828, -0.054786, -0.0191051}; // {0.014, 0.081, 0.029};
//            collisionPointStruct.n = {0.883, 0.15, -0.385}; // {0.612, 0.066, 0.630};
//            counter++;
//            active = true;
//        }
        else if (yarp::os::Time::now() - t_1 > 35. && counter < 400)
        {
            collisionPointStruct.x = {-0.027228, -0.054786, -0.0191051}; // {0.018, 0.095, 0.024};
            collisionPointStruct.n = {-0.886, 0.14, -0.431 }; // {0.568, -0.18, 0.406};
            counter++;
            active = true;
        }
        if (active)
        {
            bool exists = false;
            for (auto& colPoint : main_arm->collisionPoints)
            {
                if (colPoint.x == collisionPointStruct.x)
                {
                    colPoint.reset();
                    exists = true;
                    break;
                }
            }
            if (!exists) main_arm->collisionPoints.push_back(collisionPointStruct);
        }
    }
}

bool reactCtrlThread::preprocCollisions()
{
    main_arm->updateCollPoints();
    if (second_arm) second_arm->updateCollPoints();
//    insertTestingCollisions();
    getCollisionsFromPorts();
    bool vel_limited = !main_arm->collisionPoints.empty();
    main_arm->avhdl->getVLIM(main_arm->Aobst, main_arm->bvalues, main_arm_constr);
    main_arm->updateRecoveryPath();
    if (second_arm)
    {
        vel_limited |= !second_arm->collisionPoints.empty();
        second_arm->avhdl->getVLIM(second_arm->Aobst, second_arm->bvalues, !main_arm_constr);
        second_arm->updateRecoveryPath();
    }
    return vel_limited;
}

void reactCtrlThread::getCollisionsFromPorts()
{
    if (tactileCollPointsOn)
    {
        printMessage(9,"[reactCtrlThread::run()] Getting tactile collisions from port.\n");
        getCollisionPointsFromPort(aggregSkinEventsInPort, TACTILE_INPUT_GAIN, TACTILE_OBS);
    }
    if (visualCollPointsOn) //note, these are not mutually exclusive - they can co-exist
    {
        // process the SensationManager port
        if (auto* sensManagerBottle = sensManagerPort.read(false))
        {
            int idx_v = 0;
            for (int i = 0; i < sensManagerBottle->size(); i++)
            {
                const Bottle bl = *(sensManagerBottle->get(i).asList());
                for(int j=0;j<bl.size();j++){
                    const Bottle b = *(bl.get(j).asList());
                    if (b.size()>=5 && idx_v < 100) {
                        obsWorldPos[9+idx_v] = Vector{b.get(0).asFloat64(), b.get(1).asFloat64(), b.get(2).asFloat64()};
                        idx_v++;

                    }
                }
            }
        }

        printMessage(9,"[reactCtrlThread::run()] Getting visual collisions from port.\n");
        getCollisionPointsFromPort(aggregPPSeventsInPort, VISUAL_INPUT_GAIN, VISUAL_OBS);
    }
    if (proximityCollPointsOn)
    {
        printMessage(9,"[reactCtrlThread::run()] Getting proximity collisions from port.\n");
        ts.update();
        skinContactList& sCLout = proximityEventsForiCubGuiPort.prepare();
        sCLout.clear();

        for (int k = 0; k < 2; k++)
        {
            Bottle* bot = k == 0? proximityEventsInPort.read(false) : proximityEvents2InPort.read(false);
            visuhdl.deleteiCubGuiObject("prox_obs"+std::to_string(k));
            if (bot) {
                getCollPointFromPort(bot, PROXIMITY_INPUT_GAIN, PROX_OBS);
                yarp::os::Bottle& b = proximityEventsVisuPort.prepare();
                b.clear();
                b.addFloat64(bot->get(13).asFloat64());
                proximityEventsVisuPort.write();

                Vector geocenter(3, 0.0); // geocenter from skin / average activation locus from the pps
                Vector normal(3, 0.0);
                const Vector force(3, 0.0);
                Vector moment(3, 0.0); // will remain zero

                // printf("Bottle %d contains %s \n", i,collPointBottle->toString().c_str());
                const SkinPart sp = static_cast<SkinPart>(bot->get(0).asInt32());
                geocenter(0) = bot->get(1).asFloat64();
                geocenter(1) = bot->get(2).asFloat64();
                geocenter(2) = bot->get(3).asFloat64();
                normal(0) = bot->get(4).asFloat64();
                normal(1) = bot->get(5).asFloat64();
                normal(2) = bot->get(6).asFloat64();
                const double normalized_activation = 20 * bot->get(13).asFloat64();
                moment(0) = -normalized_activation * normal(0);
                moment(1) = -normalized_activation * normal(1);
                moment(2) = -normalized_activation * normal(2);

                skinContact const sc(SkinPart_2_BodyPart[sp].body, sp, getLinkNum(sp), geocenter, geocenter, {}, normalized_activation, normal, force, moment);

                Matrix T_a;
                if ((main_arm->part_short == "left" && SkinPart_2_BodyPart[sp].body == RIGHT_ARM) ||
                    (main_arm->part_short == "right" && SkinPart_2_BodyPart[sp].body == LEFT_ARM))
                {
                    if (second_arm == nullptr) return;
                    T_a = second_arm->arm->getH(3+SkinPart_2_LinkNum[sp].linkNum, true);
                }
                else
                {
                    T_a = main_arm->arm->getH(3+SkinPart_2_LinkNum[sp].linkNum, true);
                }
                const Vector prox_obs = {geocenter(0), geocenter(1), normal(2)*(1.05-bot->get(13).asFloat64())/5 ,1};
                visuhdl.sendiCubGuiObject("prox_obs"+ std::to_string(k), T_a*prox_obs);
                sCLout.push_back(sc);
            }
            proximityEventsForiCubGuiPort.setEnvelope(ts);
            proximityEventsForiCubGuiPort.write();

        }
    }
    //after this point, we don't care where did the collision points come from - our relative confidence in the two modalities is expressed in the gains

    if (visualizeCollisionPointsInSim)
    {
        printMessage(5,"[reactCtrlThread::run()] will visualize collision points in simulator.\n");
        visuhdl.showCollisionPointsInSim(*main_arm->arm, main_arm->collisionPoints, second_arm? second_arm->arm:nullptr,
                                         second_arm? second_arm->collisionPoints : std::vector<collisionPoint_t>{},
                                         main_arm->avhdl->getSelfColPointsTorso());
    }
}



void reactCtrlThread::run()
{
    for (int k = 0; k < 109; k++)
    {
        obsWorldPos[k].zero();
    }
    updateArmChain();
    if (streamingTarget)    //read "trajectory" - in this special case, it is only set of next target positions for possibly multiple control points
    {
        if (readStreamingTarget())
        {
           state = STATE_REACH;
        }
//        std::vector<Vector> x_planned;
//        if (readMotionPlan(x_planned))
//        {
//            setNewTarget(x_planned[0],false);
//        }
    }
    bool vel_limited = false;
    switch (state)
    {
    case STATE_WAIT:
    {
        break;
    }
    case STATE_REACH:
    {
        if (second_arm)
        {
            printMessage(2, "norm(x_t-x_d) = %g \t norm(x2_t-x2_d) = %g\n",
                         norm(main_arm->x_t-main_arm->x_d), norm(second_arm->x_t-second_arm->x_d));
        }
        else
        {
            printMessage(2, "norm(x_t-x_d) = %g\n", norm(main_arm->x_t-main_arm->x_d));
        }

        auto o_t = axisAngleToMatrix(main_arm->o_t);
        auto o_d = axisAngleToMatrix(main_arm->o_d);
        Eigen::AngleAxisd error(o_d * o_t.transpose());
        const bool inTarget = norm(main_arm->x_t - main_arm->x_d) < globalTol && (error.angle() < 2*globalTol);
        //we keep solving until we reach the desired target
        if (!movingTargetCircle && !holding_position && !streamingTarget)
        {
//            yDebug("[reactCtrlThread] angle error %g", error.angle());
            if (inTarget && (second_arm == nullptr || norm(second_arm->x_t - second_arm->x_d) < globalTol)) {
                comingHome = false;
                yDebug("[reactCtrlThread] norm(x_t-x_d) %g\tglobalTol %g", norm(main_arm->x_t - main_arm->x_d), globalTol);
                state = STATE_IDLE;
                break;
            }

            if ((yarp::os::Time::now() - t_0 > timeLimit)) { // || main_arm->notMovingCounter > 1. / dT)) {
                main_arm->notMovingCounter = 0;
                yDebug("[reactCtrlThread] Target not reachable -  norm(x_t-x_d) %g", norm(main_arm->x_t - main_arm->x_d));
                state = STATE_IDLE;
                break;
            }
        }

        if (movingTargetCircle)
        {
            main_arm->x_d = getPosMovingTargetOnCircle();
        }

        if(gazeControl)
        {
            igaze->lookAtFixationPoint(Vector{-1.5,0.0,0.4}); //for now looking at final target (x_d), not at intermediate/next target x_n
//            igaze->lookAtFixationPoint(Vector{-1.2,-0.1,0.2}); //for now looking at final target (x_d), not at intermediate/next target x_n
        }
        vel_limited = preprocCollisions();

        if (!inTarget || movingTargetCircle || streamingTarget ||
            (second_arm != nullptr && norm(second_arm->x_t-second_arm->x_d) >= globalTol) || vel_limited)
        {
            nextMove(vel_limited);
        }
        else
        {
            main_arm->q_dot.zero();
            if (second_arm) second_arm->q_dot.zero();
        }
        updateArmChain(); //N.B. This is the second call within run(); may give more precise data for the logging; may also cost time

        break;
    }
    case STATE_IDLE:
    {
        yarp::os::Bottle& b = movementFinishedPort.prepare();
        b.clear();
        Vector p;
        p = main_arm->arm->EndEffPose();
        for (int i = 0; i < 7; i++)
        {
            b.addInt32(static_cast<int>(round(p(i) * M2MM)));
        }
        movementFinishedPort.write();
        main_arm->q_dot.zero();
        if (second_arm) second_arm->q_dot.zero();
        yInfo("[reactCtrlThread] finished.");
        state=STATE_WAIT;
        break;
    }
    default:
        yFatal("[reactCtrlThread] reactCtrlThread should never be here!!! Step: %d",state);
    }

    sendData();
    sendObsData();
    if (vel_limited) //if vLim was changed by the avoidanceHandler, we reset it
    {
        main_arm->vLimAdapted = main_arm->vLimNominal;
        if (second_arm) second_arm->vLimAdapted = second_arm->vLimNominal;
    }

    printMessage(2,"[reactCtrlThread::run()] finished, state: %d.\n\n\n",state);
}

void reactCtrlThread::nextMove(bool& vel_limited)
{
    main_arm->updateNextTarget(vel_limited);
    if (second_arm) second_arm->updateNextTarget(vel_limited);
    if (second_arm) {
        visuhdl.visualizeObjects(main_arm->x_d, main_arm->x_n, second_arm->x_d, second_arm->x_n);
    }
    else {
        visuhdl.visualizeObjects(main_arm->x_d, main_arm->x_n);
    }
    const double t_3 = yarp::os::Time::now();
    //this is the key function call where the reaching opt problem is solved
    solverExitCode = solveIK();
    timeToSolveProblem_s = yarp::os::Time::now() - t_3;
    main_arm->qIntegrated = main_arm->I->integrate(main_arm->q_dot);
    main_arm->virtualArm->setAng(main_arm->qIntegrated * CTRL_DEG2RAD);
    if (second_arm)
    {
        second_arm->qIntegrated = second_arm->I->integrate(second_arm->q_dot);
        second_arm->virtualArm->setAng(second_arm->qIntegrated * CTRL_DEG2RAD);
    }

    if (!controlArm("positionDirect"))
    {
        yError("I am not able to properly control the arm in positionDirect!");
    }
}

void reactCtrlThread::threadRelease()
{

    yInfo("threadRelease(): deleting arm and torso encoder arrays and arm object.");
    delete encsT; encsT = nullptr;
    delete torso; torso = nullptr;
    const bool stoppedOk = stopControlAndSwitchToPositionMode();
    if (stoppedOk) { yInfo("Sucessfully stopped arm and torso controllers"); }
    else { yWarning("Controllers not stopped successfully"); }
    yInfo("Closing controllers..");
    main_arm->release();
    if (second_arm) second_arm->release();
    ddT.close();

    if(gazeControl)
    {
        yInfo("Closing gaze controller..");
        igaze -> lookAtAbsAngles(Vector(3,0.0));
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
        ddG.close();
    }

    yInfo("Closing ports..");
    aggregPPSeventsInPort.interrupt();
    aggregPPSeventsInPort.close();
    aggregSkinEventsInPort.interrupt();
    aggregSkinEventsInPort.close();
    proximityEventsInPort.interrupt();
    proximityEventsInPort.close();
    proximityEvents2InPort.interrupt();
    proximityEvents2InPort.close();
    proximityEventsVisuPort.interrupt();
    proximityEventsVisuPort.close();
    proximityEventsForiCubGuiPort.interrupt();
    proximityEventsForiCubGuiPort.close();
    outPort.interrupt();
    outPort.close();
    outObsPort.interrupt();
    outObsPort.close();
    sensManagerPort.interrupt();
    sensManagerPort.close();
    visuhdl.closePorts();
    movementFinishedPort.interrupt();
    movementFinishedPort.close();
    NeoObsInPort.interrupt();
    NeoObsInPort.close();
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
        main_arm->arm->releaseLink(i);
    }
    main_arm->vLimAdapted.setSubcol({-vMax, -vMax, -vMax}, 0, 0);
    main_arm->vLimAdapted.setSubcol({vMax, vMax, vMax}, 0, 1);
    main_arm->vLimNominal.setSubcol({-vMax, -vMax, -vMax}, 0, 0);
    main_arm->vLimNominal.setSubcol({vMax, vMax, vMax}, 0, 1);

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
    main_arm->vLimAdapted.setSubcol({0.0,0.0,0.0}, 0, 0);
    main_arm->vLimAdapted.setSubcol({0.0,0.0,0.0}, 0, 1);
    main_arm->vLimNominal.setSubcol({0.0,0.0,0.0}, 0, 0);
    main_arm->vLimNominal.setSubcol({0.0,0.0,0.0}, 0, 1);
    return true;
}

bool reactCtrlThread::setVMax(const double _vMax)
{
    if (_vMax>=0.0)
    {
        vMax=_vMax;
        main_arm->setVMax(useTorso, vMax);
        if (second_arm) second_arm->setVMax(useTorso, vMax);
        return true;
    }
    return false;
}

bool reactCtrlThread::setNewTarget(const Vector& _x_d, bool _movingCircle)
{
    return setNewTarget(_x_d, main_arm->o_t, _movingCircle);
}

bool reactCtrlThread::setNewTarget(const Vector& _x_d, const Vector& _o_d, bool _movingCircle)
{
    if (_x_d.size()==3 && _o_d.size()==4)
    {
        streamingTarget = false;
        ee_dist_constr++;
        t_0=Time::now();
        t_1=Time::now();
        holding_position = _x_d == main_arm->x_t;
        movingTargetCircle = _movingCircle;
        updateArmChain(); //updates ==chain, q and x_t
        main_arm->resetTarget(_x_d,  comingHome? main_arm->o_home : _o_d, trajSpeed);

        if (second_arm)
        {
            second_arm->q_dot.zero();
            second_arm->virtualArm->setAng(second_arm->q*CTRL_DEG2RAD); //with new target, we make the two chains identical at the start
            second_arm->I->reset(second_arm->q);
            second_arm->x_0 = second_arm->x_t;
            second_arm->x_d = second_arm->x_t;
            second_arm->x_n = second_arm->x_t;
            second_arm->o_d = second_arm->o_t;
            second_arm->o_n = second_arm->o_t;
            second_arm->o_0 = second_arm->o_t;
            second_arm->useSampling = false;
        }

        yInfo("[reactCtrlThread] got new target: x_0: %s",main_arm->x_0.toString(3,3).c_str());
        yInfo("[reactCtrlThread]                 x_d: %s",main_arm->x_d.toString(3,3).c_str());
        //yInfo("[reactCtrlThread]                 vel: %s",vel.toString(3,3).c_str());

        visuhdl.visualizeObjects(main_arm->x_d, main_arm->x_0);
        state=STATE_REACH;

        return true;
    }
    return false;
}

bool reactCtrlThread::setBothTargets(const yarp::sig::Vector& _x_d, const yarp::sig::Vector& _x2_d, bool m_arm_constr)
{
    return second_arm != nullptr && setBothTargets(_x_d, main_arm->o_d, _x2_d, second_arm->o_d, m_arm_constr);
}

bool reactCtrlThread::setBothTargets(const yarp::sig::Vector& _x_d, const yarp::sig::Vector& _o_d, const yarp::sig::Vector& _x2_d, const yarp::sig::Vector& _o2_d, bool m_arm_constr)
{
    if (second_arm != nullptr && _x_d.size()==3 && _o_d.size()==4 && _x2_d.size()==3 && _o2_d.size()==4)
    {
        streamingTarget = false;
        ee_dist_constr++;
        t_0=Time::now();
        t_1=Time::now();
        holding_position = _x_d == main_arm->x_t;
        movingTargetCircle = false;
        updateArmChain(); //updates ==chain, q and x_t
        main_arm->resetTarget(_x_d, _o_d, trajSpeed);
        second_arm->resetTarget(_x2_d, _o2_d, trajSpeed);
        second_arm->useSampling = true;
        main_arm_constr = m_arm_constr;

        yInfo("[reactCtrlThread] got new target: x_0: %s",main_arm->x_0.toString(3,3).c_str());
        yInfo("[reactCtrlThread]                 x_d: %s",main_arm->x_d.toString(3,3).c_str());
        //yInfo("[reactCtrlThread]                 vel: %s",vel.toString(3,3).c_str());

        visuhdl.visualizeObjects(main_arm->x_d, main_arm->x_0);
        state=STATE_REACH;

        return true;
    }
    return false;
}

bool reactCtrlThread::setNewRelativeTarget(const Vector& _rel_x_d)
{
    streamingTarget = false;
    //    if(_rel_x_d == Vector(3,0.0)) return false;
    updateArmChain(); //updates chain, q and x_t
    const Vector _x_d = main_arm->x_t + _rel_x_d;
    return setNewTarget(_x_d,false);
}

bool reactCtrlThread::setNewCircularTarget(const double _radius, const double _frequency)
{
    streamingTarget = false;
    radius = _radius;
    frequency = _frequency;
    updateArmChain(); //updates chain, q and x_t
    circleCenter = main_arm->x_t; // set it to end-eff position at this point
    setNewTarget(getPosMovingTargetOnCircle(),true);
    return true;
}

bool reactCtrlThread::stopControlAndSwitchToPositionMode()
{
//    ee_dist_constr = -1;
    state=STATE_WAIT;
    return  setCtrlModes(jointsToSetPosA,"arm","position")  &&
           (second_arm == nullptr || setCtrlModes(jointsToSetPosA,"second_arm","position")) &&
           setCtrlModes(jointsToSetPosT,"torso","position");
}

bool reactCtrlThread::goHome()
{
    comingHome = true;
    return setNewTarget(main_arm->x_home, false);
}


bool reactCtrlThread::holdPosition()
{
    return setNewTarget(main_arm->x_t, false);
}


//************** protected methods *******************************/


int reactCtrlThread::solveIK()
{
    printMessage(3, "calling osqp with the following joint velocity limits (deg): \n %s \n", main_arm->vLimAdapted.toString(3, 3).c_str());
    // printf("calling ipopt with the following joint velocity limits (rad): \n %s \n",(main_arm->vLimAdapted*CTRL_DEG2RAD).toString(3,3).c_str());
    //  Remember: at this stage everything is kept in degrees because the robot is controlled in degrees.
    //  At the ipopt level it comes handy to translate everything in radians because iKin works in radians.

    Vector xr(7, 0.0);
    xr.setSubvector(0, main_arm->x_n);
    xr.setSubvector(3, main_arm->o_n);
    int count = 0;
    int exit_code = 0;
    size_t dim = main_arm->chainActiveDOF;
    auto obstacles = std::vector<Vector>{};
    //        main_arm->avoidance = false;
    if (Bottle *xdNew=NeoObsInPort.read(false))
    {
        //            main_arm->avoidance = true;
        for (int j = 0; j < xdNew->size()-2; j+=3)
        {
            obstacles.push_back(Vector{xdNew->get(0+j).asFloat64(), xdNew->get(1+j).asFloat64(), xdNew->get(2+j).asFloat64()});
        }
    }
    if (second_arm) {
        dim += second_arm->chainActiveDOF - NR_TORSO_JOINTS;
        Vector xr2(7, 0.0);
        xr2.setSubvector(0, second_arm->x_n);
        xr2.setSubvector(3, second_arm->o_n);
        solver->init(xr, main_arm->q_dot, main_arm->vLimAdapted, comingHome ? 10 : restPosWeight, main_arm->Aobst, main_arm->bvalues,
                     second_arm->Aobst, second_arm->bvalues, xr2, second_arm->q_dot, second_arm->vLimAdapted, ee_dist_constr > 0);
    } else {
        solver->init(xr, main_arm->q_dot, main_arm->vLimAdapted, comingHome ? 10 : restPosWeight, main_arm->Aobst, main_arm->bvalues);
    }
    Vector res(dim, 0.0);
    auto vals = std::vector<double>{0, std::numeric_limits<double>::max()};
//    auto vals = std::vector<double>{std::numeric_limits<double>::max()}; // added for bubbles

    Matrix bounds;
    bounds.resize(dim, 2);
    while (count < vals.size()) {
        exit_code = solver->optimize(vals[count], main_arm_constr);
        if (exit_code >= OSQP_SOLVED) {
            res = solver->get_resultInDegPerSecond(bounds);
            break;
        }
        count++;
    }
    if (exit_code >= OSQP_SOLVED)
    {
        main_arm->vLimAdapted = bounds.submatrix(0, main_arm->chainActiveDOF - 1, 0, 1) * CTRL_RAD2DEG;
    }
    main_arm->q_dot = res.subVector(0, main_arm->chainActiveDOF-1);
    if (second_arm)
    {
        second_arm->q_dot.setSubvector(0, res.subVector(0, 2));
        second_arm->q_dot.setSubvector(NR_TORSO_JOINTS, res.subVector(main_arm->chainActiveDOF, res.size() - 1));
        if (exit_code >= OSQP_SOLVED) {
            second_arm->vLimAdapted.setSubmatrix(bounds.submatrix(0, NR_TORSO_JOINTS - 1, 0, 1) * CTRL_RAD2DEG, 0, 0);
            second_arm->vLimAdapted.setSubmatrix(bounds.submatrix(main_arm->chainActiveDOF, bounds.rows() - 1, 0, 1) * CTRL_RAD2DEG, NR_TORSO_JOINTS, 0);
        }
    }
    if (exit_code == OSQP_TIME_LIMIT_REACHED)
    {
        yWarning("[reactCtrlThread] OSQP cpu time was higher than the rate of the thread!");
    }
    else if (exit_code < OSQP_SOLVED)
    {
        yWarning("[reactCtrlThread] OSQP solve did not succeed!");
    }
    if(verbosity >= 1){
        printf("x_n: %s\tx_d: %s\tdT %g\n",main_arm->x_n.toString(3,3).c_str(),main_arm->x_d.toString(3,3).c_str(),dT);
        printf("x_0: %s\tx_t: %s\n",       main_arm->x_0.toString(3,3).c_str(),main_arm->x_t.toString(3,3).c_str());
        printf("norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g\n",
               norm(main_arm->x_n-main_arm->x_t), norm(main_arm->x_d-main_arm->x_n), norm(main_arm->x_d-main_arm->x_t));
        printf("Result (solved velocities (deg/s)): %s\n",main_arm->q_dot.toString(3,3).c_str());
        if (second_arm)
        {
            printf("x2_d: %s\tx2_t: %s\tnorm(x2_home-x2_t): %g\n", second_arm->x_d.toString(3, 3).c_str(),
                   second_arm->x_t.toString(3, 3).c_str(), norm(second_arm->x_d- second_arm->x_t));
            printf("Result (solved velocities (deg/s)): %s\n",second_arm->q_dot.toString(3,3).c_str());
        }
    }

    return exit_code;
}


/**** kinematic chain, control, ..... *****************************/

void reactCtrlThread::updateArmChain()
{
    iencsT->getEncoders(encsT->data());
    qT[0]=(*encsT)[2];
    qT[1]=(*encsT)[1];
    qT[2]=(*encsT)[0];
    torso->setAng(qT*CTRL_DEG2RAD);
    main_arm->updateArm(qT);
    if (second_arm) second_arm->updateArm(qT);
}

bool reactCtrlThread::alignJointsBounds()
{
    return main_arm->alignJointsBound(ilimT) && (second_arm == nullptr || second_arm->alignJointsBound(ilimT));
}

bool reactCtrlThread::areJointsHealthyAndSet(std::vector<int> &jointsToSet, const std::string &_p, const std::string &_s)
{
    jointsToSet.clear();
    std::vector<int> modes(main_arm->jntsA);
    if (_p=="arm")
    {
        modes.resize(NR_ARM_JOINTS,VOCAB_CM_IDLE);
        main_arm->imodA->getControlModes(modes.data());
    }
    else if (_p=="torso")
    {
        modes.resize(NR_TORSO_JOINTS,VOCAB_CM_IDLE);
        imodT->getControlModes(modes.data());
    }
    else if (_p=="second_arm" && second_arm)
    {
        modes.resize(NR_ARM_JOINTS,VOCAB_CM_IDLE);
        second_arm->imodA->getControlModes(modes.data());
    }
    else { return false; }

    for (int i=0; i<modes.size(); i++)
    {
        if ((_p != "second_arm" && main_arm->arm->isLinkBlocked(i)) ||
            (_p == "second_arm" && second_arm && second_arm->arm->isLinkBlocked(i))) { continue; }
        if ((modes[i]==VOCAB_CM_HW_FAULT) || (modes[i]==VOCAB_CM_IDLE)) { return false; }

        // we will set only those that are not in correct modes already
        if ((_s=="velocity" && modes[i]!=VOCAB_CM_MIXED && modes[i]!=VOCAB_CM_VELOCITY)
            || (_s=="position" && modes[i]!=VOCAB_CM_MIXED && modes[i]!=VOCAB_CM_POSITION)
            || (_s=="positionDirect" && modes[i]!=VOCAB_CM_POSITION_DIRECT))
        {
            jointsToSet.push_back(i);
        }
    }
    if(verbosity >= 10){
        printf("[reactCtrlThread::areJointsHealthyAndSet] %s: ctrl Modes retrieved: ",_p.c_str());
        for (const int mode : modes){
            printf("%s ",Vocab32::decode(mode).c_str());
        }
        printf("\n");
        printf("Indexes of joints to set: ");
        for (const int joint : jointsToSet){
            printf("%d ",joint);
        }
        printf("\n");
    }

    return true;
}

bool reactCtrlThread::setCtrlModes(const std::vector<int> &jointsToSet, const std::string &_p, const std::string &_s)
{
    if (_s!="position" && _s!="velocity" && _s!="positionDirect") { return false; }

    if (jointsToSet.empty()) { return true; }

    std::vector<int> modes;
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
        main_arm->imodA->setControlModes(static_cast<int>(jointsToSet.size()),
                               jointsToSet.data(),
                               modes.data());
    }
    else if (_p=="torso")
    {
        imodT->setControlModes(static_cast<int>(jointsToSet.size()),
                               jointsToSet.data(),
                               modes.data());
    }
    else if (_p=="second_arm" && second_arm)
    {
        second_arm->imodA->setControlModes(static_cast<int>(jointsToSet.size()),
                                jointsToSet.data(),
                                modes.data());
    }
    else
    {
        return false;
    }
    return true;
}

//N.B. the targetValues can be either positions or velocities, depending on the control mode!
bool reactCtrlThread::controlArm(const std::string& _controlMode)
{
    std::vector<int> jointsToSetA;
    std::vector<int> jointsToSetA2;
    std::vector<int> jointsToSetT;
    if (!areJointsHealthyAndSet(jointsToSetA,"arm",_controlMode))
    {
        yWarning("[reactCtrlThread::controlArm] Stopping control because arm joints are not healthy!");
        return false;
    }

    if (!areJointsHealthyAndSet(jointsToSetT,"torso",_controlMode))
    {
        yWarning("[reactCtrlThread::controlArm] Stopping control because torso joints are not healthy!");
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

    if (second_arm)
    {
        if (!areJointsHealthyAndSet(jointsToSetA2, "second_arm", _controlMode))
        {
            yWarning("[reactCtrlThread::controlArm] Stopping control because second arm joints are not healthy!");
            return false;
        }

        if (!setCtrlModes(jointsToSetA2, "second_arm", _controlMode))
        {
            yError("[reactCtrlThread::controlArm] I am not able to set the second arm joints to %s mode!", _controlMode.c_str());
            return false;
        }
    }

    if(_controlMode == "positionDirect")
    {
        Vector posT(3,0.0);
        posT[0] = main_arm->qIntegrated[2]; //swapping pitch and yaw as per iKin vs. motor interface convention
        posT[1] = main_arm->qIntegrated[1];
        posT[2] = main_arm->qIntegrated[0]; //swapping pitch and yaw as per iKin vs. motor interface convention
        printMessage(2,"    positionDirect: torso (swap pitch & yaw): %s\n",posT.toString(3,3).c_str());
        iposDirT->setPositions(NR_TORSO_JOINTS, jointsToSetPosT.data(),posT.data());
        printMessage(1,"[reactCtrlThread::controlArm] Target joint positions (iKin order, deg): %s\n",main_arm->qIntegrated.toString(3,3).c_str());
        main_arm->iposDirA->setPositions(NR_ARM_JOINTS, jointsToSetPosA.data(), main_arm->qIntegrated.subVector(3,9).data()); //indexes 3 to 9 are the arm joints
        if (second_arm)
        {
            printMessage(1,"[reactCtrlThread::controlArm] Target joint positions2 (iKin order, deg): %s\n",second_arm->qIntegrated.toString(3,3).c_str());
            second_arm->iposDirA->setPositions(NR_ARM_JOINTS, jointsToSetPosA.data(), second_arm->qIntegrated.subVector(3,9).data());
        }
    }

    return true;
}


/***************** auxiliary computations  *******************************/

Vector reactCtrlThread::getPosMovingTargetOnCircle()
{
    Vector _x_d=circleCenter;
    //x-coordinate will stay constant; we set y, and z
    _x_d[1]+=radius*cos(2.0*M_PI*frequency*(yarp::os::Time::now()-t_0));
    _x_d[2]+=radius*sin(2.0*M_PI*frequency*(yarp::os::Time::now()-t_0));

    return _x_d;
}

/**** communication through ports in/out ****************/
void reactCtrlThread::getCollisionPointsFromPort(BufferedPort<Bottle> &inPort, double gain, int type)
{
    Bottle* collPointsMultiBottle = inPort.read(false);
    if(collPointsMultiBottle == nullptr)
    {
        printMessage(9,"[reactCtrlThread::getCollisionPointsFromPort]: no avoidance vectors on the port.\n") ;
        return;
    }
    printMessage(5,"[reactCtrlThread::getCollisionPointsFromPort]: There were %d bottles on the port.\n",
                 collPointsMultiBottle->size());
    for(int i=0; i< collPointsMultiBottle->size();i++)
    {
        Bottle* bot = collPointsMultiBottle->get(i).asList();
        printMessage(5, "Bottle %d contains %s \n", i, bot->toString().c_str());
        getCollPointFromPort(bot, gain, type);
    }
}

void reactCtrlThread::getCollPointFromPort(Bottle* bot, double gain, int type)
{
    const SkinPart sp = static_cast<SkinPart>(bot->get(0).asInt32());
//    if (type == TACTILE_OBS && (sp == SKIN_RIGHT_HAND || sp == SKIN_LEFT_HAND)) // added for bimanual task
//        return;
    // we take only those collision points that are relevant for the chain we are controlling + torso
    if (SkinPart_2_BodyPart[sp].body == LEFT_ARM || (sp == SKIN_FRONT_TORSO && useTorso) ||
        SkinPart_2_BodyPart[sp].body == RIGHT_ARM)
    {
        ArmInterface* arm_ptr;
        if ((main_arm->part_short == "left" && SkinPart_2_BodyPart[sp].body == RIGHT_ARM) ||
            (main_arm->part_short == "right" && SkinPart_2_BodyPart[sp].body == LEFT_ARM))
        {
            if (second_arm == nullptr) return;
            arm_ptr = second_arm.get();
        }
        else
        {
            arm_ptr = main_arm.get();
        }

        collisionPoint_t newColPoint{type};
        newColPoint.skin_part = sp;
        newColPoint.x = {bot->get(1).asFloat64(), bot->get(2).asFloat64(), bot->get(3).asFloat64()};
//        newColPoint.n = {bot->get(4).asFloat64(), bot->get(5).asFloat64(), bot->get(6).asFloat64()};
        newColPoint.magnitude = bot->get(13).asFloat64() * gain; //* 0.7 added for bimanual task

        if (type != VISUAL_OBS)
        {
            Matrix T_a;
            if (SkinPart_2_BodyPart[sp].body == TORSO)
            {
                T_a = torso->getH(SkinPart_2_LinkNum[sp].linkNum, true);
            }
            else
            {
                T_a = arm_ptr->arm->getH(3+SkinPart_2_LinkNum[sp].linkNum, true);
            }
            newColPoint.n = T_a.submatrix(0,2,0,2) * Vector{bot->get(4).asFloat64(), bot->get(5).asFloat64(), bot->get(6).asFloat64()};
            if (type == PROX_OBS) {
                obsWorldPos[(sp == SKIN_LEFT_HAND)? 0 : 1] = (T_a * Vector{bot->get(1).asFloat64(), bot->get(2).asFloat64(),
                                                    bot->get(6).asFloat64()*(1.05-bot->get(13).asFloat64())/5, 1}).subVector(0,2);
//                printf("Added proximity obstacle\n");
            } else if (type == TACTILE_OBS) {
                obsWorldPos[1+static_cast<int>(sp)] = (T_a * Vector{bot->get(1).asFloat64(), bot->get(2).asFloat64(), bot->get(3).asFloat64(), 1}).subVector(0,2);
//                printf("Added tactile obstacle with SP %d\n", sp);
            }
        }
        else
        {
            newColPoint.n = {bot->get(10).asFloat64(), bot->get(11).asFloat64(), bot->get(12).asFloat64()};
            newColPoint.duration *= 2; // added for realsense obstacles
        }


        bool exists = false;
        for (auto& colPoint : arm_ptr->collisionPoints)
        {
            if (norm(colPoint.x - newColPoint.x) < 1e-2 && colPoint.type == newColPoint.type)
            {
                colPoint.reset(newColPoint.magnitude);
                colPoint.x = colPoint.x;
                colPoint.n = newColPoint.n;
                exists = true;
                break;
            }
        }
        if (!exists) {
//            printf("New col points position %s\n", newColPoint.x.toString(3).c_str());
            arm_ptr->collisionPoints.push_back(newColPoint);
        }
    }
}

void reactCtrlThread::sendData()
{
    ts.update();
    printMessage(5,"[reactCtrlThread::sendData()]\n");
    if (outPort.getOutputCount()>0)
    {
        Vector closestPoint2Obs(3,0.0);
        Vector secondclosestPoint2Obs(3,0.0);
        Vector thirdclosestPoint2Obs(3,0.0);
        auto ctrlPoints = main_arm->avhdl->getCtrlPointsPosition();
        if (!ctrlPoints.empty())
        {
            closestPoint2Obs = ctrlPoints[0].first;
            if (ctrlPoints.size() > 1) {
                secondclosestPoint2Obs = ctrlPoints[1].first;
                if (ctrlPoints.size() > 2)
                {
                    thirdclosestPoint2Obs = ctrlPoints[2].first;
                }
            }
        }

        yarp::os::Bottle b;
        b.clear();

        //col 1-4
        b.addInt32(static_cast<int>(main_arm->chainActiveDOF));
        b.addInt32(solverExitCode);
        b.addFloat64(timeToSolveProblem_s);
        b.addInt32(main_arm->avoidance);

        //position
        //cols 5-7: the desired final target (for end-effector)
        vectorIntoBottle(main_arm->x_d,b);
        // 8:10 the end effector position in which the robot currently is
        vectorIntoBottle(main_arm->x_t,b);
        // 11:13 the current desired target given by the reference generation (particle / minJerk) (for end-effector)
        vectorIntoBottle(main_arm->x_n,b);
        //orientation
        //cols 14-17: the desired final orientation (for end-effector)
        vectorIntoBottle(main_arm->o_d,b);
        // 18:21 the end effector orientation in which the robot currently is
        vectorIntoBottle(main_arm->o_t,b);
        // 22:25 the current desired orientation given by referenceGen (currently not supported - equal to o_d)
        vectorIntoBottle(main_arm->o_n,b);

        //variable - if torso on: 26:35: joint velocities as solution to control and sent to robot
        vectorIntoBottle(main_arm->q_dot,b);
        //variable - if torso on: 36:45: actual joint positions
        vectorIntoBottle(main_arm->q,b);
        //joint pos from virtual chain IPopt is operating onl variable - if torso on: 46:55
        vectorIntoBottle(main_arm->qIntegrated,b);
        //variable - if torso on: 56:75; joint vel limits as input to QP, after avoidanceHandler,
        matrixIntoBottle(main_arm->vLimAdapted,b); // assuming it is row by row, so min_1, max_1, min_2, max_2 etc.
        vectorIntoBottle(closestPoint2Obs, b);
        vectorIntoBottle(secondclosestPoint2Obs, b);
        vectorIntoBottle(thirdclosestPoint2Obs, b);
        if (second_arm)
        {
            // col 85-86
            b.addInt32(static_cast<int>(second_arm->chainActiveDOF));
            b.addInt32(second_arm->avoidance);

            //position
            //cols 87-89: the desired final target (for end-effector)
            vectorIntoBottle(second_arm->x_d,b);
            // 90:92 the end effector position in which the robot currently is
            vectorIntoBottle(second_arm->x_t,b);
            // 93:95 the current desired target given by the reference generation (particle / minJerk) (for end-effector)
            vectorIntoBottle(second_arm->x_n,b);
            //orientation
            //cols 96-99: the desired final orientation (for end-effector)
            vectorIntoBottle(second_arm->o_d,b);
            // 100:103 the end effector orientation in which the robot currently is
            vectorIntoBottle(second_arm->o_t,b);
            // 104:107 the current desired orientation given by referenceGen (currently not supported - equal to o_d)
            vectorIntoBottle(second_arm->o_n,b);

            //variable - if torso on: 108:117 joint velocities as solution to control and sent to robot
            vectorIntoBottle(second_arm->q_dot,b);
            //variable - if torso on: 118:127 actual joint positions
            vectorIntoBottle(second_arm->q,b);
            //joint pos from virtual chain IPopt is operating onl variable - if torso on: 128:137
            vectorIntoBottle(second_arm->qIntegrated,b);
            //variable - if torso on: 138:157; joint vel limits as input to QP, after avoidanceHandler,
            matrixIntoBottle(second_arm->vLimAdapted,b); // assuming it is row by row, so min_1, max_1, min_2, max_2 etc.

            auto ctrlPoints2 = second_arm->avhdl->getCtrlPointsPosition();
            Vector closestPoint2Obs2(3,0.0);
            Vector secondclosestPoint2Obs2(3,0.0);
            Vector thirdclosestPoint2Obs2(3,0.0);
            if (!ctrlPoints2.empty())
            {
                closestPoint2Obs2 = ctrlPoints2[0].first;
                if (ctrlPoints2.size() > 1) {
                    secondclosestPoint2Obs2 = ctrlPoints2[1].first;
                    if (ctrlPoints2.size() > 2)
                    {
                        thirdclosestPoint2Obs2 = ctrlPoints2[2].first;
                    }
                }
            }
            vectorIntoBottle(closestPoint2Obs2, b);
            vectorIntoBottle(secondclosestPoint2Obs2, b);
            vectorIntoBottle(thirdclosestPoint2Obs2, b);


        }
        outPort.setEnvelope(ts);
        outPort.write(b);
    }
}


void reactCtrlThread::sendObsData()
{
    printMessage(5,"[reactCtrlThread::sendObsData()]\n");
    if (outObsPort.getOutputCount()>0)
    {
        yarp::os::Bottle b;
        b.clear();
        const int arms = (second_arm != nullptr) + 1;
        for (int l = 0; l < arms; l++) {
            auto ctrlPoints = (l == 0) ? main_arm->avhdl->getCtrlPoints() : second_arm->avhdl->getCtrlPoints();
//            const int offset =  (l == 0)? 0 : 44;
            const int visu_off =  (l == 0)? 0 : -4;
            const int cap = (l == 0)?44:34;
            int idx_prox = 0, idx_visu_h = 0, idx_visu_f = 0, idx_visu_u = 0, idx_visu_t = 0;
            int idx_tact_h = 0, idx_tact_f = 0, idx_tact_u = 0, idx_tact_t = 0;
            std::vector<Vector> closestPoints(cap);
            for (int k = 0; k < cap; k++) {
                closestPoints[k].resize(3, 0.0);
            }
            for (auto& ctrlPoint : ctrlPoints) {
                if (ctrlPoint.second == TACTILE_OBS) {
                    if (ctrlPoint.first.getDOF() == 10 && idx_tact_h < 4) {
                        closestPoints[4 + idx_tact_h] = ctrlPoint.first.EndEffPosition();
                        idx_tact_h++;
                    } else if (ctrlPoint.first.getDOF() == 8 && idx_tact_f < 4) {
                        closestPoints[4 + 4 + idx_tact_f] = ctrlPoint.first.EndEffPosition();
                        idx_tact_f++;
                    } else if (ctrlPoint.first.getDOF() == 6 && idx_tact_u < 4) {
                        closestPoints[4 + 8 + idx_tact_u] = ctrlPoint.first.EndEffPosition();
                        idx_tact_u++;
                    } else if (ctrlPoint.first.getDOF() == 3 && idx_tact_t < 4 && l == 0) {
                        closestPoints[4 + 12 + idx_tact_t] = ctrlPoint.first.EndEffPosition();
                        idx_tact_t++;
                    }
                } else if (ctrlPoint.second == VISUAL_OBS) {
                    if (ctrlPoint.first.getDOF() == 10 && idx_visu_h < 6) {
                        closestPoints[20 + idx_visu_h + visu_off] = ctrlPoint.first.EndEffPosition();
                        idx_visu_h++;
                    } else if (ctrlPoint.first.getDOF() == 8 && idx_visu_f < 6) {
                        closestPoints[20 + 6 + idx_visu_f + visu_off] = ctrlPoint.first.EndEffPosition();
                        idx_visu_f++;
                    } else if (ctrlPoint.first.getDOF() == 6 && idx_visu_u < 6) {
                        closestPoints[20 + 12 + idx_visu_u + visu_off] = ctrlPoint.first.EndEffPosition();
                        idx_visu_u++;
                    } else if (ctrlPoint.first.getDOF() == 3 && idx_visu_t < 6 && l == 0) {
                        closestPoints[20 + 18 + idx_visu_t + visu_off] = ctrlPoint.first.EndEffPosition();
                        idx_visu_t++;
                    }
                } else if (ctrlPoint.second == PROX_OBS and idx_prox < 4) {
                    closestPoints[idx_prox] = ctrlPoint.first.EndEffPosition();
                    idx_prox++;
                }
            }

            for (auto& closestPoint : closestPoints) {
                vectorIntoBottle(closestPoint, b);
            }
        }
        for (auto& obsP : obsWorldPos)
        {
            vectorIntoBottle(obsP, b);
        }

        outObsPort.setEnvelope(ts);
        outObsPort.write(b);
    }
}

bool reactCtrlThread::readMotionPlan(std::vector<Vector> &x_desired)
{
    Bottle *inPlan = streamedTargets.read(false);

    bool hasPlan = false;
    if(inPlan!=nullptr) {
        yInfo() << "received motionPlan";
        const size_t nbCtrlPts = inPlan->size();

        for (size_t i=0; i<nbCtrlPts; i++)
        {
            if (Bottle* inListTraj = inPlan->get(i).asList())
            {
                if (inListTraj->find("number-waypoints").asInt32()>0)
                {
                    const int nDim = inListTraj->find("number-dimension").asInt32();
                    if (nDim>=3)
                    {
                        if (Bottle* coordinate = inListTraj->find("waypoint_0").asList())
                        {
                            if (coordinate->size()==nDim)
                            {
                                Vector xCtrlPt(nDim, 0.0);
                                for (size_t k=0; k<nDim; k++)
                                {
                                    xCtrlPt[k]=coordinate->get(k).asFloat64();
                                }
                                const std::string ctrlPtName = inListTraj->find("control-point").asString();
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


bool reactCtrlThread::readStreamingTarget()
{
    if (Bottle *xdNew=streamedTargets.read(false))
    {
        main_arm->x_d = {xdNew->get(0).asFloat64(), xdNew->get(1).asFloat64(), xdNew->get(2).asFloat64()};
        main_arm->x_0 = main_arm->x_t;
        main_arm_constr = true;
        if (xdNew->size() <= 7)
        {
            if (xdNew->size() == 7 && xdNew->get(6).isFloat64()) {
                main_arm->o_d = {xdNew->get(3).asFloat64(), xdNew->get(4).asFloat64(),
                                 xdNew->get(5).asFloat64(), xdNew->get(6).asFloat64()};
            }
            else if (xdNew->size() >= 6 && second_arm)
            {
                second_arm->x_d = {xdNew->get(3).asFloat64(), xdNew->get(4).asFloat64(), xdNew->get(5).asFloat64()};
                second_arm->x_0 = second_arm->x_t;
                if (xdNew->size() == 7) {
                    main_arm_constr = xdNew->get(6).asInt32();
                }
            }
        }
        else if (second_arm)
        {
            second_arm->x_d = {xdNew->get(8).asFloat64(), xdNew->get(9).asFloat64(), xdNew->get(10).asFloat64()};
            second_arm->x_0 = second_arm->x_t;
            if (xdNew->size() == 12) {
                main_arm_constr = xdNew->get(11).asInt32();
            }
            if (xdNew->size() >= 15)
            {
                second_arm->o_d = {xdNew->get(11).asFloat64(), xdNew->get(12).asFloat64(),
                                   xdNew->get(13).asFloat64(), xdNew->get(14).asFloat64()};
                if (xdNew->size() == 16) {
                    main_arm_constr = xdNew->get(15).asInt32();
                }
            }
        }
//        ee_dist_constr = 1; // turn on for bimanual task
//        main_arm->minJerkTarget->init(main_arm->x_t);
//        main_arm->minJerkTarget->setTs(dT); //time step
//        main_arm->minJerkTarget->setT(0.1);
//        main_arm->minJerkTarget->computeNextValues(main_arm->x_d);

        return true;
    }
    return false;
}


int reactCtrlThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

        va_list ap;
        va_start(ap,f);
        const int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    return -1;
}

// empty line to make gcc happy
