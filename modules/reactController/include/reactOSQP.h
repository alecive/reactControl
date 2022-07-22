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
    iCubArm &arm, *second_arm;

    bool hitting_constraints;

    Matrix v_lim, v2_lim;
    Vector q0,v0, q02, v02, p02, p0, rest_jnt_pos, rest_w; //, normal;
    Matrix H0, J0, H02, J02;
    Vector pr, v_des, manip, pr2, v2_des, manip2;
    Matrix bounds;
    double dt, w1, w2, w3, w4, w5, min_type, vmax, adapt_w5, manip_thr, orig_w2, adapt_w52;

    int chain_dof, secondchain_dof, vars_offset, constr_offset;

    double shou_m,shou_n;
    double elb_m,elb_n;

    Vector qGuardMinExt;
    Vector qGuardMinInt;
    Vector qGuardMaxExt;
    Vector qGuardMaxInt;

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
    QPSolver(iCubArm &chain_, bool hittingConstraints_, iCubArm* second_chain_, double vmax_, bool orientationControl_, double dT_, const Vector& restPos,
                  double restPosWeight=0.0);
    ~QPSolver();
    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, double rest_pos_w);
    void init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, const Vector &_xr2, const Vector &_v02, const Matrix &_v2_lim, double rest_pos_w);

    Vector get_resultInDegPerSecond()
    {
        Eigen::VectorXd sol = solver.getSolution();
        Vector v(chain_dof+secondchain_dof,0.0);
        for (int i = 0; i < chain_dof; ++i)
        {
            v[i] = max(min(sol[i], upperBound[i]), lowerBound[i]);
        }
        for (int i = 0; i < secondchain_dof; ++i)
        {
            v[i+chain_dof] = max(min(sol[i+vars_offset], upperBound[i+constr_offset]), lowerBound[i+constr_offset]);
        }
        return CTRL_RAD2DEG*v;
    }

    int optimize(double pos_error)
    {
        update_bounds(pos_error);
        Eigen::VectorXd primalVar(hessian.rows());
        primalVar.setZero();
        for (int i = 0; i < chain_dof; i++)
        {
            primalVar[i] = std::min(std::max(bounds(i, 0), v0[i]), bounds(i, 1));
        }
        if (secondchain_dof > 0)
        {
            for (int i = 0; i < secondchain_dof; i++)
            {
                primalVar[i+vars_offset] = std::min(std::max(bounds(i+chain_dof, 0), v02[i]), bounds(i+chain_dof, 1));
            }
        }
        solver.setPrimalVariable(primalVar);
        solver.solve();
        return static_cast<int>(solver.workspace()->info->status_val);
    }
};


#endif //__REACTQP_H__
