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


#define TACTILE_INPUT_GAIN 1.5
#define VISUAL_INPUT_GAIN 1
#define PROXIMITY_INPUT_GAIN 1

#define STATE_WAIT              0
#define STATE_REACH             1
#define STATE_IDLE              2


/*********** public methods ****************************************************************************/

reactCtrlThread::reactCtrlThread(int _rate, string _name, string _robot,  string _part, string second_part,
                                 int _verbosity, bool _disableTorso,
                                 double _trajSpeed, double _globalTol, double _vMax, double _tol, double _timeLimit,
                                 string _referenceGen, bool _tactileCPOn, bool _visualCPOn, bool _proximityCPOn,
                                 bool _gazeControl, bool _stiffInteraction,
                                 bool _hittingConstraints, bool _orientationControl,
                                 bool _visTargetInSim, bool _visParticleInSim, bool _visCollisionPointsInSim,
                                 particleThread *_pT, double _restPosWeight, bool _selfColPoints) :
        PeriodicThread(static_cast<double>(_rate)/1000.0), name(std::move(_name)), second_part(std::move(second_part)),
        robot(std::move(_robot)), part(std::move(_part)), verbosity(_verbosity), useTorso(!_disableTorso),
        trajSpeed(_trajSpeed), globalTol(_globalTol), vMax(_vMax), tol(_tol), timeLimit(_timeLimit),
        referenceGen(std::move(_referenceGen)), tactileCollisionPointsOn(_tactileCPOn), visualCollisionPointsOn(_visualCPOn),
        proximityCollisionPointsOn(_proximityCPOn), gazeControl(_gazeControl), stiffInteraction(_stiffInteraction),
        hittingConstraints(_hittingConstraints), orientationControl(_orientationControl),
        visualizeCollisionPointsInSim(_visCollisionPointsInSim), counter(0), restPosWeight(_restPosWeight),
        selfColPoints(_selfColPoints), state(STATE_WAIT), minJerkTarget(nullptr), iencsT(nullptr), iposDirT(nullptr),
        imodT(nullptr), ilimT(nullptr), encsT(nullptr), jntsT(0), igaze(nullptr), contextGaze(0),
        movingTargetCircle(false), radius(0), frequency(0), streamingTarget(true), tactileColAvoidance(false),
        t_0(0), solverExitCode(0), timeToSolveProblem_s(0), comingHome(false), holding_position(false),
        visuhdl(verbosity, (robot == "icubSim"), name, _visTargetInSim,
                referenceGen != "none" && _visParticleInSim)
{
    dT=getPeriod();
    prtclThrd=_pT;  //in case of referenceGen != uniformParticle, NULL will be received
}

bool reactCtrlThread::threadInit()
{

    printMessage(2,"[reactCtrlThread] threadInit()\n");

    /******** iKin chain and variables, and transforms init *************************/
    main_arm =std::make_unique<ArmInterface>(part);
    second_arm = (second_part=="left_arm" || second_part=="right_arm")? std::make_unique<ArmInterface>(second_part) : nullptr;
//    //N.B. All angles in this thread are in degrees
    qT.resize(NR_TORSO_JOINTS,0.0); //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)
    setVMax(vMax);

    if (!prepareDrivers()) return false;
    main_arm->updateJointLimMatrix();
    second_arm->updateJointLimMatrix();
    /************ variables related to target and the optimization problem for ipopt *******/
    if(referenceGen == "minJerk")
    {
        minJerkTarget = new minJerkTrajGen(3,dT,1.0); //dim 3, dT, trajTime 1s - will be overwritten later
    }

    circleCenter.resize(3,0.0);
    circleCenter(0) = -0.3; //for safety, we assign the x-coordinate on in it within iCub's reachable space
    Time::delay(1);
    updateArmChain();

    main_arm->I = new Integrator(dT,main_arm->q,main_arm->lim);
    second_arm->I = new Integrator(dT,second_arm->q,second_arm->lim);


    /***************** ports and files*************************************************************************************/

    aggregPPSeventsInPort.open("/"+name+"/pps_events_aggreg:i");
    aggregSkinEventsInPort.open("/"+name+"/skin_events_aggreg:i");
    proximityEventsInPort.open("/"+name+"/proximity_events:i");

    streamedTargets.open("/"+name+"/streamedWholeBodyTargets:i");

    outPort.open("/"+name +"/data:o"); //for dumping
    movementFinishedPort.open("/" + name + "/finished:o");

    fout_param.open("param.log");

    /***** writing to param file ******************************************************/
    fout_param<<main_arm->chainActiveDOF<<" ";
    for (size_t i=0; i<main_arm->chainActiveDOF; i++)
    {
        fout_param<<main_arm->lim(i,0)<<" " << main_arm->lim(i,1)<<" ";
    }
    for (size_t j=0; j<main_arm->chainActiveDOF; j++)
    {
        fout_param<<main_arm->vLimNominal(j,0)<< " " <<main_arm->vLimNominal(j,1)<<" ";
    }
    fout_param<<-1<<" "<<trajSpeed<<" "<<tol<<" "<<globalTol<<" "<<dT<<" "<<0<<" "<<0<<" ";
    // the -1 used to be trajTime, keep it for compatibility with matlab scripts
    //the 0s used to be boundSmoothnessFlag and boundSmoothnessValue
    fout_param<<"2 "; // positionDirect
    fout_param<<"0 0 0 "; //used to be ipOptMemoryOn, ipOptFilterOn, filterTc
    if(stiffInteraction) fout_param<<"1 "; else fout_param<<"0 ";
    fout_param<<"0 "; // used to be additionalControlPoints
    fout_param<<endl;

    yInfo("Written to param file and closing..");
    fout_param.close();

    weighted_normal.resize(3,0);
    //the "tactile" handler will currently be applied to visual inputs (from PPS) as well
    main_arm->avhdl = std::make_unique<AvoidanceHandlerTactile>(*main_arm->virtualArm->asChain(),main_arm->collisionPoints,second_arm->virtualArm->asChain(), selfColPoints, main_arm->part_short, verbosity);
    second_arm->avhdl = std::make_unique<AvoidanceHandlerTactile>(*second_arm->virtualArm->asChain(),second_arm->collisionPoints,main_arm->virtualArm->asChain(), selfColPoints, second_arm->part_short, verbosity);

    solver = std::make_unique<QPSolver>(*main_arm->virtualArm, hittingConstraints, second_arm->virtualArm, vMax, orientationControl,
                                        dT, main_arm->homePos*CTRL_DEG2RAD, restPosWeight);
    printMessage(5,"[reactCtrlThread] threadInit() finished.\n");
    yarp::os::Time::delay(0.2);
    t_1=Time::now();
    return true;
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
        igaze->setNeckTrajTime(0.75);
        igaze->setEyesTrajTime(0.5);
    }
    return true;
}

bool reactCtrlThread::insertTestingCollisions()
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
    bool tactileCollision = false;

    if (t_1 > 0 && robot == "icubSim")
    {
        collisionPoint_t collisionPointStruct{SKIN_LEFT_FOREARM};
        if (yarp::os::Time::now() - t_1 > 16 && counter < 200)
        {
            //            collisionPointStruct.x = {-0.0002, -0.0131,-0.0258434};//  {-0.02, 0.0, -0.005}; //  // {-0.031, -0.079, 0.005};
            //            collisionPointStruct.n = {-0.005, 0.238, -0.971}; //{0,0,-1}; // // {-0.739, 0.078, 0.105};
            collisionPointStruct.skin_part = SKIN_LEFT_HAND;
            collisionPointStruct.x = {-0.02, 0, 0.01};
            collisionPointStruct.n = {0, 0, 1};
            counter++;
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
            tactileCollision = true;
        }
        else if (yarp::os::Time::now() - t_1 > 25 && counter < 300)
        {
            collisionPointStruct.x = {0.026828, -0.054786, -0.0191051}; // {0.014, 0.081, 0.029};
            collisionPointStruct.n = {0.883, 0.15, -0.385}; // {0.612, 0.066, 0.630};
            counter++;
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
            tactileCollision = true;
        }
        else if (yarp::os::Time::now() - t_1 > 35 && counter < 800)
        {
            collisionPointStruct.x = {-0.027228, -0.054786, -0.0191051}; // {0.018, 0.095, 0.024};
            collisionPointStruct.n = {-0.886, 0.14, -0.431 }; // {0.568, -0.18, 0.406};
            counter++;
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
            tactileCollision = true;
        }
    }
    return tactileCollision;
}

bool reactCtrlThread::preprocCollisions()
{
    main_arm->updateCollPoints(dT);
    if (second_arm) second_arm->updateCollPoints(dT);
    //   bool tactileCollision = insertTestingCollisions();
    bool tactileCollision = getCollisionsFromPorts();
    return tactileCollision;
}

bool reactCtrlThread::getCollisionsFromPorts()
{
    bool tactileCollision = false;
    if (tactileCollisionPointsOn)
    {
        printMessage(9,"[reactCtrlThread::run()] Getting tactile collisions from port.\n");
        getCollisionPointsFromPort(aggregSkinEventsInPort, TACTILE_INPUT_GAIN, main_arm->part_short,main_arm->collisionPoints, second_arm->collisionPoints);
        if (!main_arm->collisionPoints.empty()) { tactileCollision = true; }
    }
    if (visualCollisionPointsOn) //note, these are not mutually exclusive - they can co-exist
    {
        printMessage(9,"[reactCtrlThread::run()] Getting visual collisions from port.\n");
        getCollisionPointsFromPort(aggregPPSeventsInPort, VISUAL_INPUT_GAIN, main_arm->part_short,main_arm->collisionPoints, second_arm->collisionPoints);
        if (!main_arm->collisionPoints.empty()) { tactileCollision = true; }
    }
    if (proximityCollisionPointsOn)
    {
        printMessage(9,"[reactCtrlThread::run()] Getting proximity collisions from port.\n");
        getProximityFromPort(main_arm->collisionPoints, second_arm->collisionPoints);
        if (!main_arm->collisionPoints.empty()) { tactileCollision = true; }
    }
    //after this point, we don't care where did the collision points come from - our relative confidence in the two modalities is expressed in the gains

    if (visualizeCollisionPointsInSim)
    {
        printMessage(5,"[reactCtrlThread::run()] will visualize collision points in simulator.\n");
        visuhdl.showCollisionPointsInSim(*main_arm->arm, main_arm->collisionPoints, main_arm->avhdl->getSelfColPointsTorso());
    }
    return tactileCollision;
}

yarp::sig::Vector reactCtrlThread::updateNextTarget()
{
    Vector next_x = main_arm->x_d;
    if (referenceGen == "uniformParticle") {
        if ((norm(main_arm->x_n - main_arm->x_0) > norm(main_arm->x_d - main_arm->x_0)) || movingTargetCircle) //if the particle is farther than the final target, we reset the particle - it will stay with the target; or if target is moving
        {
            prtclThrd->resetParticle(main_arm->x_d);
        }
        next_x = prtclThrd->getParticle(); //to get next target
    }
    else if (referenceGen == "minJerk")
    {
        minJerkTarget->computeNextValues(main_arm->x_d);
        next_x = minJerkTarget->getPos();
    }

    if (!tactileColAvoidance && !last_trajectory.empty()) // TODO improve it for proximity (i.e., hysteresis)
    {
        if (norm(main_arm->x_d-main_arm->x_t) < 0.08) // TODO: find appropriate value
        {
            last_trajectory.clear();
            std::cout << "Close to target position, recovery path aborted.\n";
        }
        else
        {
            next_x = last_trajectory.back();
            if (last_trajectory.size() > 1)
            {
                last_trajectory.pop_back();
            }
            last_trajectory.pop_back();
            main_arm->vLimAdapted *= 0.75;

            yWarning() << last_trajectory.size() <<  "; Recovery path " << main_arm->x_n.toString(3,3) << "\n";
        }
    }
    return next_x;
}

void reactCtrlThread::run()
{
    //    double t2 = yarp::os::Time::now();
    //    if (state == STATE_REACH)
    //    {
    //        std::cout << t2-t_0 << " ";
    //    }
    updateArmChain();
    if (streamingTarget)    //read "trajectory" - in this special case, it is only set of next target positions for possibly multiple control points
    {
        std::vector<Vector> x_planned;
        if (readMotionPlan(x_planned))
        {
            setNewTarget(x_planned[0],false);
        }
    }

    bool tactileCollision = preprocCollisions();
    bool vel_limited = false;
    switch (state)
    {
    case STATE_WAIT:
    {
        break;
    }
    case STATE_REACH:
    {
        if (tactileColAvoidance)
        {
            last_trajectory.push_back(main_arm->x_t);
            tactileColAvoidance = false;
        }
        yInfo("[reactCtrlThread] norm(x_t-x_d) = %g",norm(main_arm->x_t-main_arm->x_d));

        //we keep solving until we reach the desired target
        if ((norm(main_arm->x_t-main_arm->x_d) < globalTol) && (norm(second_arm->x_t-second_arm->x_home) < 4*globalTol) && !movingTargetCircle  &&!holding_position)
        {
            comingHome = false;
            yDebug("[reactCtrlThread] norm(x_t-x_d) %g\tglobalTol %g",norm(main_arm->x_t-main_arm->x_d),globalTol);
            state=STATE_IDLE;
            break;
        }

        if (!movingTargetCircle && !holding_position && yarp::os::Time::now()-t_0 > timeLimit)
        {
            yDebug("[reactCtrlThread] Target not reachable -  norm(x_t-x_d) %g\tglobalTol %g",norm(main_arm->x_t-main_arm->x_d),globalTol);
            state=STATE_IDLE;
            break;
        }

        if (movingTargetCircle)
        {
            main_arm->x_d = getPosMovingTargetOnCircle();
        }

        if(gazeControl)
        {
            igaze->lookAtFixationPoint(main_arm->x_d); //for now looking at final target (x_d), not at intermediate/next target x_n
        }

        if (tactileCollisionPointsOn || visualCollisionPointsOn || proximityCollisionPointsOn)
        {
            weighted_normal = {0,0,0};
            main_arm->vLimAdapted=main_arm->avhdl->getVLIM(CTRL_DEG2RAD * main_arm->vLimNominal, weighted_normal) * CTRL_RAD2DEG;
            second_arm->vLimAdapted=second_arm->avhdl->getVLIM(CTRL_DEG2RAD * second_arm->vLimNominal, weighted_normal) * CTRL_RAD2DEG;
            vel_limited = !(main_arm->vLimAdapted == main_arm->vLimNominal) || !(second_arm->vLimAdapted == second_arm->vLimNominal);
            //    if (vel_limited) yDebug("main_arm->vLimAdapted = \n%s\n", main_arm->vLimAdapted.toString(3,3).c_str());
        }

        if (norm(main_arm->x_t-main_arm->x_d) >= globalTol || norm(second_arm->x_t-second_arm->x_home) >= 4*globalTol || movingTargetCircle || vel_limited)
        {
            tactileColAvoidance = tactileCollision;
            main_arm->x_n = updateNextTarget();
            visuhdl.visualizeObjects(main_arm->x_d, main_arm->x_n);
            //                yDebug("main_arm->vLimAdapted = %s\n\n",main_arm->vLimAdapted.toString(3,3).c_str());
            //                yDebug("second_arm->vLimAdapted = %s\n",second_arm->vLimAdapted.toString(3,3).c_str());
            double t_3 = yarp::os::Time::now();
            //this is the key function call where the reaching opt problem is solved
            solverExitCode = solveIK();
            timeToSolveProblem_s = yarp::os::Time::now() - t_3;

            main_arm->qIntegrated = main_arm->I->integrate(main_arm->q_dot);
            second_arm->qIntegrated = second_arm->I->integrate(second_arm->q_dot);
            if (!controlArm("positionDirect", main_arm->qIntegrated, second_arm->qIntegrated))
            {
                yError("I am not able to properly control the arm in positionDirect!");
            }
            main_arm->virtualArm->setAng(main_arm->qIntegrated * CTRL_DEG2RAD);
            second_arm->virtualArm->setAng(second_arm->qIntegrated * CTRL_DEG2RAD);
        }
        updateArmChain(); //N.B. This is the second call within run(); may give more precise data for the logging; may also cost time

        break;
    }
    case STATE_IDLE:
    {
        yarp::os::Bottle b;
        b.clear();
        Vector p;
        p = main_arm->arm->EndEffPose();
        for (int i = 0; i < 7; i++)
        {
            b.addInt32(static_cast<int>(round(p(i) * M2MM)));
        }
        movementFinishedPort.write(b);
        yInfo("[reactCtrlThread] finished.");
        state=STATE_WAIT;
        break;
    }
    default:
        yFatal("[reactCtrlThread] reactCtrlThread should never be here!!! Step: %d",state);
    }

    //    if (state == STATE_REACH)
    //    {
    //        std::cout << yarp::os::Time::now()-t_0 << " ";
    //    }
    sendData();
    if (vel_limited) //if vLim was changed by the avoidanceHandler, we reset it
    {
        main_arm->vLimAdapted = main_arm->vLimNominal;
        second_arm->vLimAdapted = second_arm->vLimNominal;
    }
    printMessage(2,"[reactCtrlThread::run()] finished, state: %d.\n\n\n",state);
    //    if (state == STATE_REACH) {
    //        double t3 = yarp::os::Time::now();
    //
    //     //   std::cout << t3-t_0 << " " << timeToSolveProblem_s << " " << t3-t2;
    //        if (t3-t2 > 0.01) { std::cout << " Alert!"; }
    //     //   std::cout << "\n";
    //    }
}

void reactCtrlThread::threadRelease()
{

    yInfo("threadRelease(): deleting arm and torso encoder arrays and arm object.");
    delete encsT; encsT = nullptr;
    bool stoppedOk = stopControlAndSwitchToPositionMode();
    if (stoppedOk) { yInfo("Sucessfully stopped arm and torso controllers"); }
    else { yWarning("Controllers not stopped successfully"); }
    yInfo("Closing controllers..");
    main_arm->release();
    second_arm->release();
    ddT.close();

    if(gazeControl)
    {
        yInfo("Closing gaze controller..");
        Vector ang(3,0.0);
        igaze -> lookAtAbsAngles(ang);
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
        ddG.close();
    }

    if(minJerkTarget != nullptr)
    {
        yDebug("deleting minJerkTarget..");
        delete minJerkTarget;
        minJerkTarget = nullptr;
    }

    yInfo("Closing ports..");
    aggregPPSeventsInPort.interrupt();
    aggregPPSeventsInPort.close();
    aggregSkinEventsInPort.interrupt();
    aggregSkinEventsInPort.close();
    outPort.interrupt();
    outPort.close();
    visuhdl.closePorts();
    movementFinishedPort.interrupt();
    movementFinishedPort.close();
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
    main_arm->vLimNominal(0,0)=main_arm->vLimNominal(2,0)=-vMax;
    main_arm->vLimAdapted(0,0)=main_arm->vLimAdapted(2,0)=-vMax;

    main_arm->vLimNominal(0,1)=main_arm->vLimNominal(2,1)=vMax;
    main_arm->vLimAdapted(0,1)=main_arm->vLimAdapted(2,1)=vMax;
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
    if (_x_d.size()==3)
    {
        t_0=Time::now();
        t_1=Time::now();
        holding_position = _x_d == main_arm->x_t;
        movingTargetCircle = _movingCircle;
        main_arm->q_dot.zero();
        updateArmChain(); //updates chain, q and x_t
        main_arm->virtualArm->setAng(main_arm->q*CTRL_DEG2RAD); //with new target, we make the two chains identical at the start
        main_arm->I->reset(main_arm->q);
        main_arm->o_n = comingHome? main_arm->o_home : main_arm->o_d;
        main_arm->x_0=main_arm->x_t;
        main_arm->x_n=main_arm->x_0;
        main_arm->x_d=_x_d;

        if (referenceGen == "uniformParticle"){
            yarp::sig::Vector vel(3,0.0);
            vel=trajSpeed * (main_arm->x_d-main_arm->x_0) / norm(main_arm->x_d-main_arm->x_0);
            if (!prtclThrd->setupNewParticle(main_arm->x_0,vel)){
                yWarning("prtclThrd->setupNewParticle(x_0,vel) returned false.\n");
                return false;
            }
        }
        else if(referenceGen == "minJerk"){
            minJerkTarget->init(main_arm->x_0); //initial pos
            minJerkTarget->setTs(dT); //time step
            // calculate the time to reach from the distance to target and desired velocity
            double T = sqrt( (main_arm->x_d(0)-main_arm->x_0(0))*(main_arm->x_d(0)-main_arm->x_0(0)) + (main_arm->x_d(1)-main_arm->x_0(1))*(main_arm->x_d(1)-main_arm->x_0(1)) +
                            (main_arm->x_d(2)-main_arm->x_0(2))*(main_arm->x_d(2)-main_arm->x_0(2)) )  / trajSpeed;
            minJerkTarget->setT(std::ceil(T * 10.0) / 10.0);
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

bool reactCtrlThread::setNewRelativeTarget(const Vector& _rel_x_d)
{
    streamingTarget = false;
    //    if(_rel_x_d == Vector(3,0.0)) return false;
    updateArmChain(); //updates chain, q and x_t
    Vector _x_d = main_arm->x_t + _rel_x_d;
    return setNewTarget(_x_d,false);
}

bool reactCtrlThread::setNewCircularTarget(const double _radius,const double _frequency)
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
    bool stoppedOk = stopControlAndSwitchToPositionModeHelper();
    if (stoppedOk)
    {
        yInfo("reactCtrlThread::stopControlAndSwitchToPositionMode(): Successfully stopped controllers");
    }
    else
    {
        yWarning("reactCtrlThread::stopControlAndSwitchToPositionMode(): Controllers not stopped successfully");
    }
    return stoppedOk;
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

    printMessage(3,"calling ipopt with the following joint velocity limits (deg): \n %s \n",main_arm->vLimAdapted.toString(3,3).c_str());
    //printf("calling ipopt with the following joint velocity limits (rad): \n %s \n",(main_arm->vLimAdapted*CTRL_DEG2RAD).toString(3,3).c_str());
    // Remember: at this stage everything is kept in degrees because the robot is controlled in degrees.
    // At the ipopt level it comes handy to translate everything in radians because iKin works in radians.

    Vector xr(6,0.0);
    xr.setSubvector(0,main_arm->x_n);
    xr.setSubvector(3,main_arm->o_n);
    Vector xr2(6,0.0);
    xr2.setSubvector(0,second_arm->x_home);
    xr2.setSubvector(3,second_arm->o_home);

    int count = 0;
    int exit_code = 0;
    Vector res(main_arm->chainActiveDOF+second_arm->chainActiveDOF-NR_TORSO_JOINTS, 0.0);
    solver->init(xr, main_arm->q_dot, main_arm->vLimAdapted,  xr2,  second_arm->q_dot, second_arm->vLimAdapted, comingHome? 10:restPosWeight);
    std::array<double,3> vals = {0, 0.05, std::numeric_limits<double>::max()};
    while(count < vals.size()) {
        exit_code = solver->optimize(vals[count]);
        if (exit_code == OSQP_SOLVED) {
            yInfo("Problem solved in %d run(s)\n", count+1);
            res = solver->get_resultInDegPerSecond();
            break;
        }
        count++;
    }
    main_arm->q_dot = res.subVector(0, main_arm->chainActiveDOF-1);
    second_arm->q_dot.setSubvector(0,res.subVector(0,2));
    second_arm->q_dot.setSubvector(NR_TORSO_JOINTS,res.subVector(main_arm->chainActiveDOF,res.size()-1));

    if (exit_code == OSQP_TIME_LIMIT_REACHED)
    {
        yWarning("[reactCtrlThread] OSQP cpu time was higher than the rate of the thread!");
    }
    else if (exit_code != OSQP_SOLVED)
    {
        yWarning("[reactCtrlThread] OSQP solve did not succeed!");
    }

    // printMessage(0,"t_d: %g\tt_t: %g\n",t_d-t_0, t_t-t_0);
    if(verbosity >= 1){
        printf("x_n: %s\tx_d: %s\tdT %g\n",main_arm->x_n.toString(3,3).c_str(),main_arm->x_d.toString(3,3).c_str(),dT);
        printf("x_0: %s\tx_t: %s\n",       main_arm->x_0.toString(3,3).c_str(),main_arm->x_t.toString(3,3).c_str());
        printf("norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g\n",
               norm(main_arm->x_n-main_arm->x_t), norm(main_arm->x_d-main_arm->x_n), norm(main_arm->x_d-main_arm->x_t));
        printf("x2_home: %s\tx2_t: %s\tnorm(x2_home-x2_t): %g\n",
               second_arm->x_home.toString(3,3).c_str(),second_arm->x_t.toString(3,3).c_str(),norm(second_arm->x_home-second_arm->x_t));
        printf("Result (solved velocities (deg/s)): %s\n",res.toString(3,3).c_str());
        printf("Result (solved velocities (deg/s)): %s\n",second_arm->q_dot.toString(3,3).c_str());
    }

    return exit_code;
}


/**** kinematic chain, control, ..... *****************************/

void reactCtrlThread::updateArmChain()
{
    main_arm->iencsA->getEncoders(main_arm->encsA->data());
    main_arm->qA=main_arm->encsA->subVector(0,NR_ARM_JOINTS-1);
    second_arm->iencsA->getEncoders(second_arm->encsA->data());
    second_arm->qA = second_arm->encsA->subVector(0,NR_ARM_JOINTS-1);
    iencsT->getEncoders(encsT->data());
    qT[0]=(*encsT)[2];
    qT[1]=(*encsT)[1];
    qT[2]=(*encsT)[0];

    main_arm->q.setSubvector(0,qT);
    second_arm->q.setSubvector(0,qT);
    main_arm->q.setSubvector(NR_TORSO_JOINTS,main_arm->qA);
    second_arm->q.setSubvector(NR_TORSO_JOINTS,second_arm->qA);
    main_arm->arm->setAng(main_arm->q*CTRL_DEG2RAD);
    second_arm->arm->setAng(second_arm->q*CTRL_DEG2RAD);
    second_arm->x_t = second_arm->arm->EndEffPosition();
    main_arm->x_t = main_arm->arm->EndEffPosition();
    main_arm->o_t = main_arm->arm->EndEffPose().subVector(3,5)*main_arm->arm->EndEffPose()[6];
    second_arm->o_t = second_arm->arm->EndEffPose().subVector(3,5)*second_arm->arm->EndEffPose()[6];
}

bool reactCtrlThread::alignJointsBounds()
{
    yDebug("[reactCtrlThread][alignJointsBounds] pre alignment:");
    printJointsBounds();

    deque<IControlLimits*> limits;
    limits.push_back(ilimT);
    limits.push_back(main_arm->ilimA);
    if (!main_arm->arm->alignJointsBounds(limits)) return false;

    deque<IControlLimits*> limits2;
    limits2.push_back(ilimT);
    limits2.push_back(second_arm->ilimA);
    if (!second_arm->arm->alignJointsBounds(limits2)) return false;

    yDebug("[reactCtrlThread][alignJointsBounds] post alignment:");
    printJointsBounds();

    return true;
}

void reactCtrlThread::printJointsBounds()
{
    iCub::iKin::iKinChain &chain=*main_arm->arm->asChain();
    for (size_t i = 0; i < main_arm->chainActiveDOF; i++)
    {
        double min=chain(i).getMin()*CTRL_RAD2DEG;
        double max=chain(i).getMax()*CTRL_RAD2DEG;
        yDebug("[jointsBounds (deg)] i: %lu\tmin: %g\tmax %g",i,min,max);
    }

    iCub::iKin::iKinChain &chain2=*second_arm->arm->asChain();
    for (size_t i = 0; i < second_arm->chainActiveDOF; i++)
    {
        double min=chain2(i).getMin()*CTRL_RAD2DEG;
        double max=chain2(i).getMax()*CTRL_RAD2DEG;
        yDebug("[jointsBounds (deg)] i: %lu\tmin: %g\tmax %g",i,min,max);
    }
}

bool reactCtrlThread::areJointsHealthyAndSet(vector<int> &jointsToSet, const string &_p, const string &_s)
{
    jointsToSet.clear();
    vector<int> modes(main_arm->jntsA);
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
    else if (_p=="second_arm")
    {
        modes.resize(NR_ARM_JOINTS,VOCAB_CM_IDLE);
        second_arm->imodA->getControlModes(modes.data());
    }
    else { return false; }

    for (int i=0; i<modes.size(); i++)
    {
        if ((_p != "second_arm" && main_arm->arm->isLinkBlocked(i)) ||
            (_p == "second_arm" && second_arm->arm->isLinkBlocked(i))) { continue; }
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
        for (int mode : modes){
            printf("%s ",Vocab32::decode(mode).c_str());
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

bool reactCtrlThread::setCtrlModes(const vector<int> &jointsToSet, const string &_p, const string &_s)
{
    if (_s!="position" && _s!="velocity" && _s!="positionDirect") { return false; }

    if (jointsToSet.empty()) { return true; }

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
    else if (_p=="second_arm")
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
bool reactCtrlThread::controlArm(const string& _controlMode, const yarp::sig::Vector &_targetValues, const yarp::sig::Vector &_targetValues2)
{
    vector<int> jointsToSetA;
    vector<int> jointsToSetA2;
    vector<int> jointsToSetT;
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

    if (!areJointsHealthyAndSet(jointsToSetA2,"second_arm",_controlMode))
    {
        yWarning("[reactCtrlThread::controlArm] Stopping control because second arm joints are not healthy!");
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

    if (!setCtrlModes(jointsToSetA2,"second_arm",_controlMode))
    {
        yError("[reactCtrlThread::controlArm] I am not able to set the second arm joints to %s mode!",_controlMode.c_str());
        return false;
    }

    if(_controlMode == "positionDirect")
    {
        printMessage(1,"[reactCtrlThread::controlArm] Target joint positions (iKin order, deg): %s\n",_targetValues.toString(3,3).c_str());
        printMessage(1,"[reactCtrlThread::controlArm] Target joint positions2 (iKin order, deg): %s\n",_targetValues2.toString(3,3).c_str());
        Vector posT(3,0.0);
        posT[0] = _targetValues[2]; //swapping pitch and yaw as per iKin vs. motor interface convention
        posT[1] = _targetValues[1];
        posT[2] = _targetValues[0]; //swapping pitch and yaw as per iKin vs. motor interface convention

        printMessage(2,"    positionDirect: torso (swap pitch & yaw): %s\n",posT.toString(3,3).c_str());
        iposDirT->setPositions(NR_TORSO_JOINTS, jointsToSetPosT.data(),posT.data());
        main_arm->iposDirA->setPositions(NR_ARM_JOINTS, jointsToSetPosA.data(), _targetValues.subVector(3,9).data()); //indexes 3 to 9 are the arm joints
        second_arm->iposDirA->setPositions(NR_ARM_JOINTS, jointsToSetPosA.data(), _targetValues2.subVector(3,9).data()); //indexes 3 to 9 are the arm joints
    }

    return true;
}

bool reactCtrlThread::stopControlAndSwitchToPositionModeHelper()
{
    state=STATE_IDLE;
    return  setCtrlModes(jointsToSetPosA,"arm","position")  &&
           setCtrlModes(jointsToSetPosA,"second_arm","position") &&
           setCtrlModes(jointsToSetPosT,"torso","position");
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


bool reactCtrlThread::getCollisionPointsFromPort(BufferedPort<Bottle> &inPort, double gain, const string& which_chain,
                                                 std::vector<collisionPoint_t> &collPoints,
                                                 std::vector<collisionPoint_t> &collPoints2)
{
    //printMessage(9,"[reactCtrlThread::getCollisionPointsFromPort].\n");
    collisionPoint_t collPoint;
    SkinPart sp;
    Bottle* collPointsMultiBottle = inPort.read(false);
    if(collPointsMultiBottle == nullptr)
    {
        printMessage(9,"[reactCtrlThread::getCollisionPointsFromPort]: no avoidance vectors on the port.\n") ;
        return false;
    }
    printMessage(5,"[reactCtrlThread::getCollisionPointsFromPort]: There were %d bottles on the port.\n",
                 collPointsMultiBottle->size());
    for(int i=0; i< collPointsMultiBottle->size();i++)
    {
        Bottle* bot = collPointsMultiBottle->get(i).asList();
        printMessage(5, "Bottle %d contains %s \n", i, bot->toString().c_str());
        sp = static_cast<SkinPart>(bot->get(0).asInt32());
        // we take only those collision points that are relevant for the chain we are controlling + torso
        if (SkinPart_2_BodyPart[sp].body == LEFT_ARM || (sp == SKIN_FRONT_TORSO && useTorso) ||
            SkinPart_2_BodyPart[sp].body == RIGHT_ARM)
        {
            collPoint.skin_part = sp;
            collPoint.x = {bot->get(1).asFloat64(), bot->get(2).asFloat64(), bot->get(3).asFloat64()};
            collPoint.n = {bot->get(4).asFloat64(), bot->get(5).asFloat64(), bot->get(6).asFloat64()};
            if (sp == SKIN_FRONT_TORSO) // normal direction from torso skin is wrong
            {
                collPoint.n(0) *= -1;
            }
            collPoint.magnitude = bot->get(13).asFloat64() * gain;
            std::vector<collisionPoint_t>* col_pts_ptr =
                ((which_chain == "left" && SkinPart_2_BodyPart[sp].body == RIGHT_ARM) ||
                 (which_chain == "right" && SkinPart_2_BodyPart[sp].body == LEFT_ARM)) ?
                    &collPoints2 : &collPoints;

            bool exists = false;
            for (auto& colPoint : *col_pts_ptr)
            {
                if (colPoint.x == collPoint.x)
                {
                    colPoint.reset(collPoint.magnitude);
                    exists = true;
                    break;
                }
            }
            if (!exists) col_pts_ptr->push_back(collPoint);
        }
    }
    return true;
}


bool reactCtrlThread::getProximityFromPort(std::vector<collisionPoint_t> &collPoints,
                                           std::vector<collisionPoint_t> &collPoints2)
{
    //    printMessage(9,"[reactCtrlThread::getProximityPointsFromPort].\n");
    collisionPoint_t collPoint;
    SkinPart sp;
    Bottle* bot = proximityEventsInPort.read(false);
    if(bot == nullptr)
    {
        printMessage(9,"[reactCtrlThread::getProximityPointsFromPort]: no avoidance vectors on the port.\n") ;
        return false;
    }
    sp =  static_cast<SkinPart>(bot->get(0).asInt32());
    if (SkinPart_2_BodyPart[sp].body == LEFT_ARM || (sp == SKIN_FRONT_TORSO && useTorso) ||
        SkinPart_2_BodyPart[sp].body == RIGHT_ARM)
    {
        printMessage(0,"Proximity magnitude %.2f \n", bot->get(7).asFloat64());
        collPoint.skin_part = sp;
        collPoint.x = {bot->get(1).asFloat64(), bot->get(2).asFloat64(), bot->get(3).asFloat64()};
        collPoint.n = {bot->get(4).asFloat64(), bot->get(5).asFloat64(), bot->get(6).asFloat64()};
        collPoint.magnitude = bot->get(7).asFloat64() * PROXIMITY_INPUT_GAIN;
        std::vector<collisionPoint_t>* col_pts_ptr =
            ((main_arm->part_short == "left" && SkinPart_2_BodyPart[sp].body == RIGHT_ARM) ||
             (main_arm->part_short == "right" && SkinPart_2_BodyPart[sp].body == LEFT_ARM)) ? &collPoints2 : &collPoints;

        bool exists = false;
        for (auto& colPoint : *col_pts_ptr)
        {
            if (colPoint.x == collPoint.x)
            {
                colPoint.reset(collPoint.magnitude);
                exists = true;
                break;
            }
        }
        if (!exists) col_pts_ptr->push_back(collPoint);
    }
    return true;
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
            b.addInt32(static_cast<int>(main_arm->chainActiveDOF));
            //position
            //cols 2-4: the desired final target (for end-effector)
            vectorIntoBottle(main_arm->x_d,b);
            // 5:7 the end effector position in which the robot currently is
            vectorIntoBottle(main_arm->x_t,b);
            // 8:10 the current desired target given by the reference generation (particle / minJerk) (for end-effector)
            vectorIntoBottle(main_arm->x_n,b);

            //orientation
            //cols 11-13: the desired final orientation (for end-effector)
            vectorIntoBottle(main_arm->o_d,b);
            // 14:16 the end effector orientation in which the robot currently is
            vectorIntoBottle(main_arm->o_t,b);
            // 17:19 the current desired orientation given by referenceGen (currently not supported - equal to o_d)
            vectorIntoBottle(main_arm->o_n,b);

            //variable - if torso on: 20:29: joint velocities as solution to control and sent to robot
            vectorIntoBottle(main_arm->q_dot,b);
            //variable - if torso on: 30:39: actual joint positions
            vectorIntoBottle(main_arm->q,b);
            //variable - if torso on: 40:59; joint vel limits as input to ipopt, after avoidanceHandler,
            matrixIntoBottle(main_arm->vLimAdapted,b); // assuming it is row by row, so min_1, max_1, min_2, max_2 etc.
            b.addInt32(solverExitCode);
            b.addFloat64(timeToSolveProblem_s);
            //joint pos from virtual chain IPopt is operating onl variable - if torso on: 60:69
            vectorIntoBottle(main_arm->qIntegrated,b);

            //TODO - remove and update indexes - desired elbow position; variable - if torso on and positionDirect: 70:72 ;
            Vector elbow_d(3,0.0);
            vectorIntoBottle( elbow_d , b);

            //actual elbow position on real chain; variable - if torso on and positionDirect: 73:75 ;
            vectorIntoBottle( (*(main_arm->arm->asChain())).getH((*(main_arm->arm->asChain())).getDOF()-4-1).getCol(3).subVector(0,2) , b);


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
                if (inListTraj->find("number-waypoints").asInt32()>0)
                {
                    int nDim = inListTraj->find("number-dimension").asInt32();
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
                                string ctrlPtName = inListTraj->find("control-point").asString();
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
    return -1;
}

