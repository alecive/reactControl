//
// Created by Jakub Rozlivek on 7/28/21.
//

#ifndef VISUALISATIONHANDLER_H
#define VISUALISATIONHANDLER_H

#include <iCub/iKin/iKinFwd.h>

#include "common.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using std::string;


class VisualisationHandler
{
public:

    VisualisationHandler(int _verbosity, bool _use_sim, const string& port_name, bool visuTarget, bool visuParticle);

    /***************** auxiliary computations  *******************************/

    void convertPosFromRootToSimFoR(const yarp::sig::Vector &pos, yarp::sig::Vector &outPos);

    static void convertPosFromLinkToRootFoR(iCub::iKin::iCubArm& arm, const yarp::sig::Vector &pos,
                                     iCub::skinDynLib::SkinPart skinPart, yarp::sig::Vector &outPos);



    /***************************** visualizations in icubGui  ****************************/
    //uses corresponding global variables for target pos (x_d) or particle pos (x_n) and creates bottles for the port to iCubGui
    void sendiCubGuiObject(const std::string& object_type, Vector x);
    
    void deleteiCubGuiObject(const std::string& object_type);

    /****************** visualizations in icub simulator   *************************************/
    /**
    * Creates a sphere (not affected by gravity) in the iCub simulator through the /icubSim/world port
    * @param radius
    * @param pos
    */
    void createStaticSphere(double radius, const yarp::sig::Vector &pos);

    void moveSphere(int index, const yarp::sig::Vector &pos);

    void createStaticBox(const yarp::sig::Vector &pos);

    void moveBox(int index, const yarp::sig::Vector &pos);

    void showCollisionPointsInSim(iCub::iKin::iCubArm& arm, const std::vector<collisionPoint_t>& collisionPoints,
                                  const std::vector<Vector>& selfColPoints);

    void visualizeObjects(const Vector& x_d, const Vector& x_n);

    void closePorts();

private:
    yarp::os::Port outPortiCubGui;
    yarp::os::Port portToSimWorld;
    yarp::os::Bottle    cmd;
    yarp::sig::Matrix T_world_root; //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR



    bool visualizeIniCubGui;
    bool visualizeParticleIniCubGui;
    bool visualizeTargetIniCubGui;
    bool use_sim;
    int collisionPointsVisualizedCount; //objects will be created in simulator and then their positions updated every iteration
    yarp::sig::Vector collisionPointsSimReservoirPos; //inactive collision points will be stored in the world

    bool visualizeTargetInSim;  // will use the yarp rpc /icubSim/world to visualize the target
    // will use the yarp rpc /icubSim/world to visualize the particle (trajectory - intermediate targets)
    bool visualizeParticleInSim;


    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;

    /**
    * Prints a message according to the verbosity level:
    * @param l will be checked against the global var verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(int l, const char *f, ...) const;
};

#endif //VISUALISATIONHANDLER_H
