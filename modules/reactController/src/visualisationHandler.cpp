//
// Created by Jakub Rozlivek on 7/28/21.
//

#include "visualisationHandler.h"

/***************** auxiliary computations  *******************************/

VisualisationHandler::VisualisationHandler(int _verbosity, bool _use_sim, const string& port_name,
                                           bool visualizeTarget, bool visualizeParticle):
                                           verbosity(_verbosity), use_sim(_use_sim), name("VisuHandler"){
    T_world_root = zeros(4,4);
    T_world_root(0,1)=-1;
    T_world_root(1,2)=1; T_world_root(1,3)=0.5976;
    T_world_root(2,0)=-1; T_world_root(2,3)=-0.026;
    T_world_root(3,3)=1;

    /*** visualize in iCubGui  ***************/
    visualizeIniCubGui = true;
    visualizeParticleInSim = visualizeParticleIniCubGui = visualizeParticle;
    visualizeTargetInSim = visualizeTargetIniCubGui = visualizeTarget;

    if (visualizeIniCubGui)
        outPortiCubGui.open("/"+port_name+"/gui:o");

    if(use_sim){
        string port2icubsim = "/" + port_name + "/sim:o";
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

        Vector x_0_sim{0,0,0};
        createStaticSphere(0.03,x_0_sim);
        createStaticSphere(0.02,x_0_sim);
    }
}

void VisualisationHandler::convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos)
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

void VisualisationHandler::convertPosFromLinkToRootFoR(iCub::iKin::iCubArm& arm, const Vector &pos,const SkinPart skinPart, Vector &outPos)
{
    Matrix T_root_to_link = yarp::math::zeros(4,4);
    int torsoDOF = 3;
    T_root_to_link = arm.getH(SkinPart_2_LinkNum[skinPart].linkNum + (skinPart != SKIN_FRONT_TORSO)*torsoDOF);
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

void VisualisationHandler::closePorts()
{
    if (visualizeIniCubGui)
        yInfo("Resetting objects in iCubGui..");
    if (outPortiCubGui.getOutputCount()>0)
    {
        Bottle b;
        b.addString("reset");
        outPortiCubGui.write(b);
    }

    if(use_sim){
        yInfo("Deleting objects from simulator world.");
        cmd.clear();
        cmd.addString("world");
        cmd.addString("del");
        cmd.addString("all");
        portToSimWorld.write(cmd);
    }

    yInfo("Closing ports..");
    if (outPortiCubGui.isOpen()){
        outPortiCubGui.interrupt();
        outPortiCubGui.close();
    }
    if (portToSimWorld.isOpen()){
        portToSimWorld.interrupt();
        portToSimWorld.close();
    }
}

void VisualisationHandler::visualizeObjects(const Vector& x_d, const Vector& x_n, const std::vector<ControlPoint>& additionalControlPoints)
{
    if(visualizeTargetInSim){
        Vector x_d_sim(3,0.0);
        convertPosFromRootToSimFoR(x_d,x_d_sim);
        moveSphere(1,x_d_sim);
    }

    if (visualizeTargetIniCubGui){
        sendiCubGuiObject("target", x_d);
        if(!additionalControlPoints.empty())
            sendiCubGuiObject("additionalTargets", additionalControlPoints);
    }

    if(visualizeParticleIniCubGui){
        sendiCubGuiObject("particle", x_n);
    }

    if (visualizeParticleInSim)
    {
        Vector x_n_sim(3,0.0);
        convertPosFromRootToSimFoR(x_n,x_n_sim);
        moveSphere(2,x_n_sim); //sphere created as second (particle) will keep the index 2
    }
}

/**** visualizations using iCubGui **************************************/


void VisualisationHandler::sendiCubGuiObject(const std::string& object_type, Vector x)
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
            obj.addDouble(1000*x(0));
            obj.addDouble(1000*x(1));
            obj.addDouble(1000*x(2));

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
            obj.addDouble(1000*x(0));
            obj.addDouble(1000*x(1));
            obj.addDouble(1000*x(2));

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

void VisualisationHandler::sendiCubGuiObject(const std::string& object_type, const std::vector<ControlPoint>& additionalControlPointsVector)
{
    if (outPortiCubGui.getOutputCount()>0) {
        Bottle obj;
        for (const auto &controlPoint : additionalControlPointsVector) {
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
        outPortiCubGui.write(obj);
    }
}

void VisualisationHandler::deleteiCubGuiObject(const string& object_type)
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

void VisualisationHandler::createStaticSphere(double _radius, const Vector &pos)
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

void VisualisationHandler::moveSphere(int index, const Vector &pos)
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

void VisualisationHandler::createStaticBox(const Vector &pos)
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

void VisualisationHandler::moveBox(int index, const Vector &pos)
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

void VisualisationHandler::showCollisionPointsInSim(iCub::iKin::iCubArm& arm,
                                                    const std::vector<collisionPoint_t>& collisionPoints)
{
    size_t nrCollisionPoints = collisionPoints.size(); //+avhdl->getSelfColPoints().size();
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
        convertPosFromLinkToRootFoR(arm, collisionPoint.x,collisionPoint.skin_part,posRoot);
        convertPosFromRootToSimFoR(posRoot,posSim);
        moveBox(j,posSim); //just move a box from the sim world
        j++;
        posRoot.zero(); posSim.zero();
    }


//    for(const auto & collisionPoint : avhdl->getSelfColPoints()) {
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


int VisualisationHandler::printMessage(const int l, const char *f, ...) const
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
