//
// Created by rozliv on 25.7.22.
//

#ifndef REACT_CONTROL_NEOQP_H
#define REACT_CONTROL_NEOQP_H

#include "common.h"
#include <iCub/iKin/iKinFwd.h>
#include "OsqpEigen/OsqpEigen.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

struct colPoint_t{
    iCub::skinDynLib::SkinPart skin_part;
    yarp::sig::Vector x; //position (x,y,z) in the FoR of the respective skin part
    yarp::sig::Vector obs;
    yarp::sig::Vector n; //direction of normal vector at that point - derived from taxel normals, pointing out of the skin
    yarp::sig::Vector vel; // velocity vector
    double distance;

    colPoint_t(): skin_part(iCub::skinDynLib::SKIN_PART_UNKNOWN), distance(0)
    {
        x.resize(3);
        obs.resize(3);
        n.resize(3);
        vel.resize(3);
    }
    colPoint_t(iCub::skinDynLib::SkinPart _skinPart, std::vector<double> pos, std::vector<double> obst, std::vector<double> nrm,
               std::vector<double> velo, double dist): skin_part(_skinPart), distance(dist)
    {
        x.resize(3);
        x = {pos[0], pos[1], pos[2]};
        n.resize(3);
        n = {nrm[0], nrm[1], nrm[2]};
        vel = {velo[0], velo[1], velo[2]};
        obs = {obst[0], obst[1], obst[2]};
    }
    colPoint_t(iCub::skinDynLib::SkinPart _skinPart, Vector pos, Vector obst, Vector nrm, Vector velo, double dist):
            skin_part(_skinPart), x(pos), obs(obst), n(nrm), vel(velo), distance(dist) {}


};


class NeoQP
{
    iCubArm *arm;
    std::string part;
    Vector q0,v0, p0, Jm;
    Matrix H0, J0, bounds, pos_lim;
    Vector pr, v_des;
    int chain_dof;
    bool hitting_constraints;
    double dt, vmax, w_d, w_q, w_n;
    double w_eps{1};
    double di{0.2};
    double ds{0.1};
    std::vector<double> bvalue;
    std::vector<Vector> Aobstacles;
    std::vector<Matrix> Hessian;
    std::vector<colPoint_t> totalColPoints;
    std::vector<std::vector<yarp::sig::Vector>> selfColPoints;


    // self-avoidance constraints
    constexpr static double shou_m = (23.-28.)/(80.+37.)*CTRL_DEG2RAD;
    constexpr static double shou_n = (28.-(23.-28.)/(80.+37.)*(-37.))*CTRL_DEG2RAD;
    constexpr static double elb_m = (40.-90.)/(105.-85.)*CTRL_DEG2RAD;
    constexpr static double elb_n = (90.-(40.-90.)/(105.-85.)*85.)*CTRL_DEG2RAD;


    Eigen::SparseMatrix<double> hessian, linearMatrix;
    Eigen::VectorXd gradient, lowerBound, upperBound;
    OsqpEigen::Solver solver;

    /****************************************************************/
    void update_bounds(double pos_error);
    void set_hessian();
    void update_gradient();
    void update_hessian();
    void update_constraints();
    void computePosLims();
    void generateRobotColPoints(yarp::sig::Vector* data);
    bool computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR);
    void checkCollisions(const std::vector<Vector> &obstacles);
    void computeObstacles(const std::vector<Vector> &obstacles);
//    void computeBounds();

public:
    NeoQP(iCubArm *chain_, bool hitConstr, double vmax_, double dT_, const std::string& part_, yarp::sig::Vector* data);
    ~NeoQP();

    void init(const Vector &_xr, const Vector &_v0, const std::vector<Vector>& obstacles); //, const std::vector<Vector>& Alims, const std::vector<double>& blims);
    Vector get_resultInDegPerSecond();
    int optimize(double pos_error);
};


#endif // REACT_CONTROL_NEOQP_H
