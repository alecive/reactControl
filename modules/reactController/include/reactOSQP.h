//
// Created by Jakub Rozlivek on 7/14/21.
//

#ifndef __REACTQP_H__
#define __REACTQP_H__

#include "common.h"
#include <iCub/iKin/iKinFwd.h>
#include "OsqpEigen/OsqpEigen.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/****************************************************************/
class QPSolver
{
    iCubArm &arm;

    bool hitting_constraints;
    bool additional_control_points_flag;

    Matrix q_lim,v_lim;
    Vector q0,v0,v,p0, rest_jnt_pos, rest_weights, rest_err, normal;
    Matrix H0, J0;
    Vector pr, v_des;
    Matrix bounds;
    double dt, w1, w2, w3, w4, min_type;

    std::vector<ControlPoint> &additional_control_points;
    int extra_ctrl_points_nr, chain_dof;
    double additional_control_points_tol;

    double shou_m,shou_n;
    double elb_m,elb_n;

    Vector qGuard;
    Vector qGuardMinExt;
    Vector qGuardMinInt;
    Vector qGuardMinCOG;
    Vector qGuardMaxExt;
    Vector qGuardMaxInt;
    Vector qGuardMaxCOG;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    OsqpEigen::Solver solver;

    /****************************************************************/
    void computeSelfAvoidanceConstraints();
    void computeGuard();
    void computeBounds();
    void update_bounds(double ori_error);
    void set_hessian();
    void update_gradient();
    void update_constraints();

public:
    QPSolver(iCubArm &chain_, std::vector<ControlPoint> &additional_control_points_, bool hittingConstraints_,
                  bool orientationControl_, bool additionalControlPoints_, double dT_, double restPosWeight=0.0);
    ~QPSolver();
    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, const Vector &col_normal);
    Vector get_resultInDegPerSecond() {
        Eigen::VectorXd sol = solver.getSolution();
        for (int i = 0; i < chain_dof; ++i) {
            if (sol[i] < lowerBound[i])
                v[i] = lowerBound[i];
            else if (sol[i] > upperBound[i])
                v[i] = upperBound[i];
            else
                v[i] = sol[i];
        }
        return CTRL_RAD2DEG*v;
    }

    int optimize(double pos_error) {
        update_bounds(pos_error);
        Eigen::VectorXd primalVar(chain_dof+6);
        primalVar.setZero();
        for (int i = 0; i < chain_dof; i++)
            primalVar[i] = std::min(std::max(bounds(i, 0), v0[i]), bounds(i, 1));
        solver.setPrimalVariable(primalVar);
        solver.solve();
        return static_cast<int>(solver.workspace()->info->status_val);
    }

};


#endif //__REACTQP_H__
