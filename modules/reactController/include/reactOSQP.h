////
//// Created by Jakub Rozlivek on 7/14/21.
////
//
//#ifndef __REACTQP_H__
//#define __REACTQP_H__
//
//#include "common.h"
//#include <iCub/iKin/iKinFwd.h>
//#include "OsqpEigen/OsqpEigen.h"
//
//
//using namespace yarp::os;
//using namespace yarp::sig;
//using namespace yarp::math;
//using namespace iCub::ctrl;
//using namespace iCub::iKin;
//using namespace iCub::skinDynLib;
//
//
//struct colPoint_t{
//    iCub::skinDynLib::SkinPart skin_part;
//    yarp::sig::Vector x; //position (x,y,z) in the FoR of the respective skin part
//    yarp::sig::Vector obs;
//    yarp::sig::Vector n; //direction of normal vector at that point - derived from taxel normals, pointing out of the skin
//    yarp::sig::Vector vel; // velocity vector
//    double distance;
//
//    colPoint_t(): skin_part(iCub::skinDynLib::SKIN_PART_UNKNOWN), distance(0)
//    {
//        x.resize(3);
//        obs.resize(3);
//        n.resize(3);
//        vel.resize(3);
//    }
//    colPoint_t(iCub::skinDynLib::SkinPart _skinPart, std::vector<double> pos, std::vector<double> obst, std::vector<double> nrm,
//               std::vector<double> velo, double dist): skin_part(_skinPart), distance(dist)
//    {
//        x.resize(3);
//        x = {pos[0], pos[1], pos[2]};
//        n.resize(3);
//        n = {nrm[0], nrm[1], nrm[2]};
//        vel = {velo[0], velo[1], velo[2]};
//        obs = {obst[0], obst[1], obst[2]};
//    }
//    colPoint_t(iCub::skinDynLib::SkinPart _skinPart, Vector pos, Vector obst, Vector nrm, Vector velo, double dist):
//            skin_part(_skinPart), x(pos), obs(obst), n(nrm), vel(velo), distance(dist) {}
//
//
//};
//
///****************************************************************/
//class QPSolver
//{
////    std::unique_ptr<ArmHelper> main_arm, second_arm;
//    double dt, w1, w2, w3, w4, orig_w2;
//    iCubArm *arm;
//    Vector q0, v0, rest_jnt_pos, rest_w; //, normal;
//    Matrix v_lim, J0, bounds;
//    Vector v_des, manip;
//    double adapt_w5, vmax, manip_thr;
//    Vector qGuardMinExt, qGuardMinInt, qGuardMaxExt, qGuardMaxInt;
//    int chain_dof; //, offset, vars_offset, constr_offset;
//    bool hitting_constraints;
//    // self-avoidance constraints
//    constexpr static double shou_m = (23.-28.)/(80.+37.)*CTRL_DEG2RAD;
//    constexpr static double shou_n = (28.-(23.-28.)/(80.+37.)*(-37.))*CTRL_DEG2RAD;
//    constexpr static double elb_m = (40.-90.)/(105.-85.)*CTRL_DEG2RAD;
//    constexpr static double elb_n = (90.-(40.-90.)/(105.-85.)*85.)*CTRL_DEG2RAD;
//
//    Eigen::SparseMatrix<double> hessian, linearMatrix;
//    Eigen::VectorXd gradient, lowerBound, upperBound;
//    OsqpEigen::Solver solver;
//    std::vector<double> bvalue;
//    std::vector<Vector> Aobstacles;
//    std::vector<colPoint_t> totalColPoints;
//    std::vector<std::vector<yarp::sig::Vector>> selfColPoints;
//    double w_eps{1};
//    double di{0.2};
//    double ds{0.1};
//    std::string part;
//
//
//    /****************************************************************/
//    void update_bounds(double pos_error);
//    void set_hessian();
//    void update_gradient();
//    void update_hessian(double pos_error);
//    void update_constraints();
//    void generateRobotColPoints(yarp::sig::Vector* data);
//    bool computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR);
//    void checkCollisions(const std::vector<Vector> &obstacles);
//    void computeObstacles(const std::vector<Vector> &obstacles);
//    void computeGuard();
//    void computeBounds();
//
//public:
//    QPSolver(iCubArm *chain_, bool hittingConstraints_, iCubArm* second_chain_, double vmax_,
//             bool orientationControl_, double dT_, const Vector& restPos, double restPosWeight, const std::string& part_, yarp::sig::Vector* data);
//    ~QPSolver();
//
//    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, double rest_pos_w,
//              //              const std::vector<yarp::sig::Vector>& Aobs, const std::vector<double> &bvals,
//              const std::vector<Vector>& obstacles,
//              const Vector &_xr2 = {}, const Vector &_v02 = {}, const Matrix &_v2_lim = {});
//    Vector get_resultInDegPerSecond(Matrix& bounds);
//    int optimize(double pos_error);
//};
//
//
//#endif //__REACTQP_H__


//
// Created by Jakub Rozlivek on 7/14/21.
//

#ifndef __REACTQP_H__
#define __REACTQP_H__

#include "common.h"
#include <iCub/iKin/iKinFwd.h>
#include "OsqpEigen/OsqpEigen.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::skinDynLib;

struct ArmHelper
{
    iCubArm *arm;
    Vector q0, v0, rest_jnt_pos, rest_w; //, normal;
    Matrix v_lim, J0, bounds;
    Vector v_des, manip;
    double adapt_w5, dt, vmax, manip_thr;
    Vector qGuardMinExt, qGuardMinInt, qGuardMaxExt, qGuardMaxInt;
    int chain_dof, offset, vars_offset, constr_offset;
    bool hitting_constraints;
    // self-avoidance constraints
    constexpr static double shou_m = (23.-28.)/(80.+37.)*CTRL_DEG2RAD;
    constexpr static double shou_n = (28.-(23.-28.)/(80.+37.)*(-37.))*CTRL_DEG2RAD;
    constexpr static double elb_m = (40.-90.)/(105.-85.)*CTRL_DEG2RAD;
    constexpr static double elb_n = (90.-(40.-90.)/(105.-85.)*85.)*CTRL_DEG2RAD;


    ArmHelper(iCubArm *chain_, double dt_, int offset_, double vmax_, double manip_thr_, const Vector& restPos,
              bool hitting_constr_, int vars_offset_=0, int constr_offset_=0);

    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim);
    void computeGuard();
    void computeBounds();
    void updateBounds(Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound, double pos_error);
    void addConstraints(Eigen::SparseMatrix<double>& linearMatrix) const;
};

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

/****************************************************************/
class QPSolver
{
    std::unique_ptr<ArmHelper> main_arm, second_arm;
    double dt, w1, w2, w3, w4, orig_w2;
    int vars_offset, constr_offset;

    Eigen::SparseMatrix<double> hessian, linearMatrix;
    Eigen::VectorXd gradient, lowerBound, upperBound;
    OsqpEigen::Solver solver;
    std::vector<double> bvalue;
    std::vector<Vector> Aobstacles;
    std::vector<colPoint_t> totalColPoints;
    std::vector<std::vector<yarp::sig::Vector>> selfColPoints;
    double w_eps{1};
    double di{0.2};
    double ds{0.1};
    std::string part;
    bool use_constr;


    /****************************************************************/
    void update_bounds(double pos_error);
    void set_hessian();
    void update_gradient();
    void update_hessian(double pos_error);
    void update_constraints();
    void generateRobotColPoints(yarp::sig::Vector* data);
    bool computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR);
    void checkCollisions(const std::vector<Vector> &obstacles);
    void computeObstacles(const std::vector<Vector> &obstacles);

public:
    QPSolver(iCubArm *chain_, bool hittingConstraints_, iCubArm* second_chain_, double vmax_,
             bool orientationControl_, double dT_, const Vector& restPos, double restPosWeight, const std::string& part_, yarp::sig::Vector* data);
    ~QPSolver();

    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, double rest_pos_w,
              const std::vector<yarp::sig::Vector>& Aobs, const std::vector<double> &bvals,
//              const std::vector<Vector>& obstacles,
              const Vector &_xr2 = {}, const Vector &_v02 = {}, const Matrix &_v2_lim = {});
    Vector get_resultInDegPerSecond(Matrix& bounds);
    int optimize(double pos_error);
};


#endif //__REACTQP_H__
