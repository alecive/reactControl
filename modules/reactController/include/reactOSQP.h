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
    Vector q0, v0, rest_jnt_pos, rest_w, v_des;
    Matrix v_lim, J0, bounds;
    double adapt_w5, dt, vmax;
    Vector qGuardMinExt, qGuardMinInt, qGuardMaxExt, qGuardMaxInt;
    int chain_dof, offset, vars_offset, constr_offset;
    bool hit_constr;
    // self-avoidance constraints
    constexpr static double shou_m = (23.-28.)/(80.+37.)*CTRL_DEG2RAD;
    constexpr static double shou_n = (28.-(23.-28.)/(80.+37.)*(-37.))*CTRL_DEG2RAD;
    constexpr static double elb_m = (40.-90.)/(105.-85.)*CTRL_DEG2RAD;
    constexpr static double elb_n = (90.-(40.-90.)/(105.-85.)*85.)*CTRL_DEG2RAD;


    ArmHelper(iCubArm *chain_, double dt_, int offset_, double vmax_, const Vector& restPos,
              bool hitting_constr_, int vars_offset_=0, int constr_offset_=0);

    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim);
    void computeGuard();
    void computeBounds();
    void updateBounds(Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound, double pos_error);
    void addConstraints(Eigen::SparseMatrix<double>& linearMatrix, int obs_contr) const;
};


/****************************************************************/
class QPSolver
{
    std::unique_ptr<ArmHelper> main_arm, second_arm;
    double dt, w2, w3, w4, orig_w2;
    int vars_offset, constr_offset;

    Eigen::SparseMatrix<double> hessian, linearMatrix;
    Eigen::VectorXd gradient, lowerBound, upperBound;
    OsqpEigen::Solver solver;
    int obs_constr_num{80};
    std::string part;


    /****************************************************************/
    void update_bounds(double pos_error, bool main_arm_constr=true);
    void set_hessian();
    void update_gradient();
    void update_hessian(double pos_error);
    void update_constraints();

public:
    QPSolver(iCubArm *chain_, bool hittingConstraints_, iCubArm* second_chain_, double vmax_,
             bool orientationControl_, double dT_, const Vector& restPos, double restPosWeight, const std::string& part_);
    ~QPSolver();

    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, double rest_pos_w,
              const std::vector<yarp::sig::Vector>& Aobs, const std::vector<double> &bvals,
              const std::vector<yarp::sig::Vector>& Aobs2={}, const std::vector<double> &bvals2={},
              const Vector &_xr2 = {}, const Vector &_v02 = {}, const Matrix &_v2_lim = {});
    Vector get_resultInDegPerSecond(Matrix& bounds);
    int optimize(double pos_error, bool main_arm_constr=true);
};


#endif //__REACTQP_H__
