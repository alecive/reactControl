//
// Created by Jakub Rozlivek on 7/14/21.
//

#include "reactOSQP.h"


/****************************************************************/
void QPSolver::computeSelfAvoidanceConstraints()
{
    double joint1_0= 28.0*CTRL_DEG2RAD;
    double joint1_1= 23.0*CTRL_DEG2RAD;
    double joint2_0=-37.0*CTRL_DEG2RAD;
    double joint2_1= 80.0*CTRL_DEG2RAD;
    shou_m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    shou_n=joint1_0-shou_m*joint2_0;

    double joint3_0= 85.0*CTRL_DEG2RAD;
    double joint3_1=105.0*CTRL_DEG2RAD;
    double joint4_0= 90.0*CTRL_DEG2RAD;
    double joint4_1= 40.0*CTRL_DEG2RAD;
    elb_m=(joint4_1-joint4_0)/(joint3_1-joint3_0);
    elb_n=joint4_0-elb_m*joint3_0;
}

/****************************************************************/
void QPSolver::computeGuard()
{
    double guardRatio=0.1;
    int dofs = chain_dof+secondchain_dof;
    qGuardMinExt.resize(dofs);
    qGuardMinInt.resize(dofs);
    qGuardMaxExt.resize(dofs);
    qGuardMaxInt.resize(dofs);
    iKinChain &chain=*arm.asChain();

    for (size_t i=0; i < chain_dof; i++)
    {
        double q_min = chain(i).getMin();
        double q_max = chain(i).getMax();
        double qGuard=0.25*guardRatio*(q_max-q_min);

        qGuardMinExt[i]=q_min+qGuard;
        qGuardMinInt[i]=qGuardMinExt[i]+qGuard;

        qGuardMaxExt[i]=q_max-qGuard;
        qGuardMaxInt[i]=qGuardMaxExt[i]-qGuard;
    }
    if (secondchain_dof > 0) {
        iKinChain& second_chain = *second_arm->asChain();
        for (size_t i=0; i < secondchain_dof; i++)
        {
            double q_min = second_chain(i+3).getMin();
            double q_max = second_chain(i+3).getMax();
            double qGuard=0.25*guardRatio*(q_max-q_min);

            qGuardMinExt[i+chain_dof]=q_min+qGuard;
            qGuardMinInt[i+chain_dof]=qGuardMinExt[i+chain_dof]+qGuard;

            qGuardMaxExt[i+chain_dof]=q_max-qGuard;
            qGuardMaxInt[i+chain_dof]=qGuardMaxExt[i+chain_dof]-qGuard;
        }
    }
}

/****************************************************************/
void QPSolver::computeBounds()
{
    bounds.resize(chain_dof+secondchain_dof, 2);
    for (size_t i=0; i < chain_dof; i++)
    {
        double qi=q0[i];
        double dmin;
        double dmax;
        if ((qi>=qGuardMinInt[i]) && (qi<=qGuardMaxInt[i]))
        {
            dmin=dmax=1.0;
        }
        else if (qi<qGuardMinInt[i])
        {
            dmin=(qi<=qGuardMinExt[i] ? 0.0 : (qi- qGuardMinExt[i])/(qGuardMinInt[i]-qGuardMinExt[i]));
            dmax=1.0;
        }
        else
        {
            dmin=1.0;
            dmax=(qi>=qGuardMaxExt[i] ? 0.0 : (qi- qGuardMaxExt[i])/(qGuardMaxInt[i]-qGuardMaxExt[i]));
        }
        bounds(i, 0) = max(dmin*-vmax, v_lim(i, 0));  // apply joint limit bounds only when it is stricter than the avoidance limits
        bounds(i, 1) = min(dmax*vmax, v_lim(i, 1));
    }

    for (size_t i=0; i < secondchain_dof; i++)
    {
        double qi=q02[i+3];
        double dmin;
        double dmax;
        if ((qi>=qGuardMinInt[i]) && (qi<=qGuardMaxInt[i]))
        {
            dmin=dmax=1.0;
        }
        else if (qi<qGuardMinInt[i])
        {
            dmin=(qi<=qGuardMinExt[i] ? 0.0 : (qi- qGuardMinExt[i])/(qGuardMinInt[i]-qGuardMinExt[i]));
            dmax=1.0;
        }
        else
        {
            dmin=1.0;
            dmax=(qi>=qGuardMaxExt[i] ? 0.0 : (qi- qGuardMaxExt[i])/(qGuardMaxInt[i]-qGuardMaxExt[i]));
        }
        bounds(i+chain_dof, 0) = max(dmin*-vmax, v2_lim(i+3, 0));  // apply joint limit bounds only when it is stricter than the avoidance limits
        bounds(i+chain_dof, 1) = min(dmax*vmax, v2_lim(i+3, 1));
    }
}


//public:
/****************************************************************/
QPSolver::QPSolver(iCubArm &chain_, bool hittingConstraints_, iCubArm* second_chain_, double vmax_, bool orientationControl_,
                             double dT_, const Vector& restPos, double restPosWeight_) :
        arm(chain_), hitting_constraints(hittingConstraints_), second_arm(second_chain_), dt(dT_), shou_m(0), shou_n(0), elb_m(0), elb_n(0),
        w1(1), w2(restPosWeight_), orig_w2(restPosWeight_), w3(10), w4(1), w5(0),  min_type(1), vmax(vmax_), adapt_w5(1), manip_thr(0.03), secondchain_dof(0)
{
    chain_dof = static_cast<int>(arm.getDOF());
    pr.resize(3, 0.0);
    v0.resize(chain_dof, 0.0); // v=v0;
    v_lim.resize(chain_dof, 2);
    bounds.resize(chain_dof, 2);
    manip.resize(chain_dof);
    // normal.resize(3,0.0);
    if (!orientationControl_)
        w4 = 0;

    if (hitting_constraints) {
        computeSelfAvoidanceConstraints();
    }

    if (second_chain_ != nullptr)
        secondchain_dof = 7; // static_cast<int>(second_arm->getDOF());
    computeGuard();
    rest_jnt_pos = restPos;
    rest_w = {1, 1, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    rest_err.resize(chain_dof, 0.0);
    vars_offset = chain_dof + 6;
    constr_offset = chain_dof + 12 + 3 + hitting_constraints * 3;

    int vars = vars_offset + (secondchain_dof > 0) * (secondchain_dof + 6);
    int constr = constr_offset + (secondchain_dof > 0) * (6 + secondchain_dof + 6 + 3 + hitting_constraints * 3);

    hessian.resize(vars, vars);
    set_hessian();
    gradient.resize(vars);
    gradient.setZero();
    lowerBound.resize(constr);
    lowerBound.setZero();
    upperBound.resize(constr);
    upperBound.setZero();
    linearMatrix.resize(constr, vars);
    for (int i = 0; i < chain_dof + 6; ++i) {
        linearMatrix.insert(i, i) = 1;
    }
    for (int i = 0; i < 6; ++i) {
        linearMatrix.insert(i + chain_dof + 6, chain_dof + i) = -1;
    }

    linearMatrix.insert(chain_dof + 12, 3) = 1.71 * dt;
    linearMatrix.insert(chain_dof + 12, 4) = -1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 1, 3) = 1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 1, 4) = -1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 1, 5) = -1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 2, 4) = dt;
    linearMatrix.insert(chain_dof + 12 + 2, 5) = dt;
    if (hitting_constraints) {
        linearMatrix.insert(chain_dof + 12 + 3, 4) = dt;
        linearMatrix.insert(chain_dof + 12 + 3, 5) = shou_m * dt;
        linearMatrix.insert(chain_dof + 12 + 4, 6) = -elb_m * dt;
        linearMatrix.insert(chain_dof + 12 + 4, 7) = dt;
        linearMatrix.insert(chain_dof + 12 + 5, 6) = elb_m * dt;
        linearMatrix.insert(chain_dof + 12 + 5, 7) = dt;
    }

    J0 = arm.GeoJacobian();
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < chain_dof; ++j) {
            if (chain_dof != 10 || j != 1) {
                linearMatrix.insert(i + chain_dof + 6, j) = J0(i, j);
            }
        }
    }

    if (secondchain_dof > 0) {
        manip2.resize(secondchain_dof);
        for (int i = 0; i < secondchain_dof + 6; ++i) {
            linearMatrix.insert(i + constr_offset, i + vars_offset) = 1;
        }
        for (int i = 0; i < 6; ++i) {
            linearMatrix.insert(i + constr_offset + secondchain_dof + 6, vars_offset + secondchain_dof + i) = -1;
        }

        linearMatrix.insert(secondchain_dof + 12 + constr_offset, 0 + vars_offset) = 1.71 * dt;
        linearMatrix.insert(secondchain_dof + 12 + constr_offset, 1 + vars_offset) = -1.71 * dt;
        linearMatrix.insert(secondchain_dof + 12 + 1 + constr_offset, 0 + vars_offset) = 1.71 * dt;
        linearMatrix.insert(secondchain_dof + 12 + 1 + constr_offset, 1 + vars_offset) = -1.71 * dt;
        linearMatrix.insert(secondchain_dof + 12 + 1 + constr_offset, 2 + vars_offset) = -1.71 * dt;
        linearMatrix.insert(secondchain_dof + 12 + 2 + constr_offset, 1 + vars_offset) = dt;
        linearMatrix.insert(secondchain_dof + 12 + 2 + constr_offset, 2 + vars_offset) = dt;
        if (hitting_constraints) {
            linearMatrix.insert(secondchain_dof + 12 + 3 + constr_offset, 1 + vars_offset) = dt;
            linearMatrix.insert(secondchain_dof + 12 + 3 + constr_offset, 2 + vars_offset) = shou_m * dt;
            linearMatrix.insert(secondchain_dof + 12 + 4 + constr_offset, 3 + vars_offset) = -elb_m * dt;
            linearMatrix.insert(secondchain_dof + 12 + 4 + constr_offset, 4 + vars_offset) = dt;
            linearMatrix.insert(secondchain_dof + 12 + 5 + constr_offset, 3 + vars_offset) = elb_m * dt;
            linearMatrix.insert(secondchain_dof + 12 + 5 + constr_offset, 4 + vars_offset) = dt;
        }

        J02 = second_arm->GeoJacobian();
        for (int i = 0; i < 6; ++i) {
            linearMatrix.insert(i + constr_offset + secondchain_dof + 6, 0) = J02(i, 0);
            linearMatrix.insert(i + constr_offset + secondchain_dof + 6, 2) = J02(i, 2);
            for (int j = 0; j < secondchain_dof; ++j) {
                linearMatrix.insert(i + constr_offset + secondchain_dof + 6, j + vars_offset) = J02(i, j + 3);
            }
        }
    }

    solver.data()->setNumberOfVariables(vars);
    solver.data()->setNumberOfConstraints(constr); // hiting_constraints*6
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);
    solver.settings()->setMaxIteration(20000);
    solver.settings()->setAbsoluteTolerance(1e-5);
    solver.settings()->setRelativeTolerance(1e-5);
    solver.settings()->setTimeLimit(0.1*dt);
    solver.settings()->setCheckTermination(10);
    solver.settings()->setPolish(true);
    solver.settings()->setRho(0.001);
    solver.settings()->setPrimalInfeasibilityTollerance(1e-4);
    solver.settings()->setDualInfeasibilityTollerance(1e-4);
    solver.settings()->setVerbosity(false);

    solver.initSolver();
}
QPSolver::~QPSolver() = default;

/****************************************************************/
void QPSolver::init(const Vector &_xr, const Vector &_xr2, const Vector &_v0, const Vector &_v02,
                    const Matrix &_v_lim, const Matrix &_v2_lim, double rest_pos_w)
{
    yAssert(6 <= _xr.length())
    yAssert(v0.length() == _v0.length())
    yAssert((v_lim.rows() == _v_lim.rows()) && (v_lim.cols() == _v_lim.cols()))
    for (int r=0; r < _v_lim.rows(); r++)
    {
        yAssert(_v_lim(r, 0) <= _v_lim(r, 1));
    }
    w2 = (rest_pos_w >= 0) ? rest_pos_w : orig_w2;
    //normal = col_normal; // TODO: add decay_rate
    v_lim= CTRL_DEG2RAD * _v_lim;
    v0= CTRL_DEG2RAD * _v0;
    q0=arm.getAng();
    H0=arm.getH();
    p0=H0.getCol(3).subVector(0,2);
    pr=_xr.subVector(0, 2);
    Vector ang=_xr.subVector(3,5);
    double ang_mag=norm(ang);
    if (ang_mag>0.0)  ang/=ang_mag;

    ang.push_back(ang_mag);

    Matrix R = axis2dcm(ang).submatrix(0,2,0,2)*H0.submatrix(0,2,0,2).transposed();
    v_des.resize(6,0);
    v_des.setSubvector(0, (pr-p0) / dt);
    v_des.setSubvector(3, dcm2rpy(R) / dt);
    J0=arm.GeoJacobian();

    adapt_w5 = 1 - sqrt(det(J0*J0.transposed()))/manip_thr;
    if (adapt_w5 <= 0)
    {
        adapt_w5 = 0;
    }
    else
    {
        Vector offset = Vector(chain_dof, 0.0);
        double delta = 0.02;
        for (int i = 0; i < chain_dof; ++i) {
            offset[i] = delta;
            Matrix Jplus = arm.GeoJacobian(q0 + offset); // Be aware - arm joint angles are changed
            Matrix Jminus = arm.GeoJacobian(q0 - offset);
            manip[i] = (sqrt(det(Jplus * Jplus.transposed())) - sqrt(det(Jminus * Jminus.transposed()))) / (2 * delta);
            offset[i] = 0.0;
        }
        arm.setAng(q0); //restore original joint angles
    }

    if (secondchain_dof > 0) {
        v2_lim= CTRL_DEG2RAD * _v2_lim;
        v02= CTRL_DEG2RAD * _v02;
        q02=second_arm->getAng();
        H02=second_arm->getH(9, true);
        p02=H02.getCol(3).subVector(0,2);
        pr2=_xr2.subVector(0, 2);

        ang=_xr2.subVector(3,5);
        ang_mag=norm(ang);
        if (ang_mag>0.0)  ang/=ang_mag;

        ang.push_back(ang_mag);

        R = axis2dcm(ang).submatrix(0,2,0,2)*H02.submatrix(0,2,0,2).transposed();
        v2_des.resize(6,0);
        v2_des.setSubvector(0, (pr2-p02) / dt);
        v2_des.setSubvector(3, dcm2rpy(R) / dt);
        J02=second_arm->GeoJacobian();
        adapt_w52 = 1 - sqrt(det(J02*J02.transposed()))/manip_thr;

        if (adapt_w52 <= 0)
        {
            adapt_w52 = 0;
        }
        else
        {
            Vector offset = Vector(secondchain_dof+3, 0.0);
            double delta = 0.02;
            for (int i = 0; i < secondchain_dof; ++i) {
                offset[i+3] = delta;
                Matrix Jplus = second_arm->GeoJacobian(q02 + offset); // Be aware - arm joint angles are changed
                Matrix Jminus = second_arm->GeoJacobian(q02 - offset);
                manip2[i] = (sqrt(det(Jplus * Jplus.transposed())) - sqrt(det(Jminus * Jminus.transposed()))) / (2 * delta);
                offset[i+3] = 0.0;
            }
            second_arm->setAng(q02); // restore original joint angles
        }
    }

    computeBounds();
    update_gradient();
    update_constraints();
}


/****************************************************************/
void QPSolver::update_bounds(double pos_error)
{
    for (int i = 0; i < chain_dof; i++)
    {
        if (bounds(i,0) <= bounds(i,1))
        {
            lowerBound[i] = bounds(i, 0);
            upperBound[i] = bounds(i, 1);
        }
        else
        {
            lowerBound[i] = upperBound[i] = 0;
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        lowerBound[chain_dof+i] = -pos_error; // normal(i) != 0? -10 : 0; //  normal(i)/dt/10 : 0; // normal(i)/dt/10;
        upperBound[chain_dof+i] = pos_error; //normal(i) != 0?  pos_error : 0; //  normal(i)/dt/10 : 0;
    }

    for (int i = 3; i < 6; ++i)
    {
        lowerBound[chain_dof+i] = -std::numeric_limits<double>::max();
        upperBound[chain_dof+i] = std::numeric_limits<double>::max();
    }

    for (int i = 0; i < 6; ++i)
    {
        lowerBound[i+chain_dof+6] = v_des[i];
        upperBound[i+chain_dof+6] = v_des[i];
    }
    

    // shoulder's cables length
    lowerBound[6+chain_dof+6]=-347.00*CTRL_DEG2RAD-(1.71*(q0[3] - q0[4]));
    upperBound[6+chain_dof+6]=std::numeric_limits<double>::max();
    lowerBound[7+chain_dof+6]=-366.57*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    upperBound[7+chain_dof+6]=112.42*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    lowerBound[8+chain_dof+6]=-66.60*CTRL_DEG2RAD-(q0[4]+q0[5]);
    upperBound[8+chain_dof+6]=213.30*CTRL_DEG2RAD-(q0[4]+q0[5]);
    if (hitting_constraints)
    {
        // avoid hitting torso
        lowerBound[9+chain_dof+6]=shou_n - (q0[4] + shou_m*q0[5]);
        upperBound[9+chain_dof+6]=std::numeric_limits<double>::max();

        // avoid hitting forearm
        lowerBound[10+chain_dof+6]=-std::numeric_limits<double>::max();
        upperBound[10+chain_dof+6]=elb_n - (-elb_m*q0[6] + q0[7]);
        lowerBound[11+chain_dof+6]=-elb_n - (elb_m*q0[6] + q0[7]);
        upperBound[11+chain_dof+6]=std::numeric_limits<double>::max();
    }

    if (secondchain_dof > 0)
    {
        for (int i = 0; i < secondchain_dof; i++)
        {
            if (bounds(i+chain_dof,0) <= bounds(i+chain_dof,1))
            {
                lowerBound[i+constr_offset] = bounds(i+chain_dof, 0);
                upperBound[i+constr_offset] = bounds(i+chain_dof, 1);
            }
            else
            {
                lowerBound[i+constr_offset] = upperBound[i+constr_offset] = 0;
            }
        }

        for (int i = 0; i < 3; ++i)
        {
            lowerBound[secondchain_dof+i+constr_offset] = -std::numeric_limits<double>::max(); // normal(i) != 0? -10 : 0; //  normal(i)/dt/10 : 0; // normal(i)/dt/10;
            upperBound[secondchain_dof+i+constr_offset] = std::numeric_limits<double>::max(); //normal(i) != 0?  pos_error : 0; //  normal(i)/dt/10 : 0;
        }

        for (int i = 3; i < 6; ++i)
        {
            lowerBound[secondchain_dof+i+constr_offset] = -std::numeric_limits<double>::max();
            upperBound[secondchain_dof+i+constr_offset] = std::numeric_limits<double>::max();
        }

        for (int i = 0; i < 6; ++i)
        {
            lowerBound[i+secondchain_dof+6+constr_offset] = v2_des[i];
            upperBound[i+secondchain_dof+6+constr_offset] = v2_des[i];
        }

        // shoulder's cables length
        lowerBound[6+secondchain_dof+6+constr_offset]=-347.00*CTRL_DEG2RAD-(1.71*(q02[3] - q02[4]));
        upperBound[6+secondchain_dof+6+constr_offset]=std::numeric_limits<double>::max();
        lowerBound[7+secondchain_dof+6+constr_offset]=-366.57*CTRL_DEG2RAD-(1.71*(q02[3]-q02[4]-q02[5]));
        upperBound[7+secondchain_dof+6+constr_offset]=112.42*CTRL_DEG2RAD-(1.71*(q02[3]-q02[4]-q02[5]));
        lowerBound[8+secondchain_dof+6+constr_offset]=-66.60*CTRL_DEG2RAD-(q02[4]+q02[5]);
        upperBound[8+secondchain_dof+6+constr_offset]=213.30*CTRL_DEG2RAD-(q02[4]+q02[5]);
        if (hitting_constraints)
        {
            // avoid hitting torso TODO: not working
            lowerBound[9+secondchain_dof+6+constr_offset]=-std::numeric_limits<double>::max();//shou_n - (q02[4] + shou_m*q02[5]);
            upperBound[9+secondchain_dof+6+constr_offset]=std::numeric_limits<double>::max();

            // avoid hitting forearm
            lowerBound[10+secondchain_dof+6+constr_offset]=-std::numeric_limits<double>::max();
            upperBound[10+secondchain_dof+6+constr_offset]=elb_n - (-elb_m*q02[6] + q02[7]);
            lowerBound[11+secondchain_dof+6+constr_offset]=-elb_n - (elb_m*q02[6] + q02[7]);
            upperBound[11+secondchain_dof+6+constr_offset]=std::numeric_limits<double>::max();
        }
    }
    solver.updateBounds(lowerBound,upperBound);
}


/****************************************************************/
void QPSolver::update_gradient()
{
    gradient.setZero();

    for (int i=0; i < chain_dof; i++)
    {
        gradient[i] = -2 * w1 * v0[i] * min_type + w2 * rest_w[i] * dt * 2 * (q0[i] - rest_jnt_pos[i]) - w5 * adapt_w5 * dt * manip[i];
    }
    if (chain_dof == 10)
    {
        gradient[1] = 0.0;
    }
    if (secondchain_dof > 0) {
        for (int i=0; i < secondchain_dof; i++)
        {
            gradient[i+vars_offset] = 0;//-2 * w1 * v02[i] * min_type + w2 * rest_w[i+3]*10 * dt * 2 * (q02[i+3] - rest_jnt_pos[i+3]) - w5 * adapt_w52 * dt * manip2[i];
        }
    }
    solver.updateGradient(gradient);
}


/****************************************************************/
void QPSolver::set_hessian()
{
    for (int i = 0; i < chain_dof; ++i)
    {
        if (chain_dof != 10 || i != 1)
        {
            hessian.insert(i, i) = 2 * w1 + 2 * w2 * dt * dt * rest_w[i];
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        hessian.insert(chain_dof+i,chain_dof+i) = 2*w3;
    }

    for (int i = 3; i < 6; ++i)
    {
        hessian.insert(chain_dof+i,chain_dof+i) = 2*w4;
    }
    if (secondchain_dof > 0) {
        for (int i = 0; i < secondchain_dof; ++i)
        {
            hessian.insert(i+vars_offset, i+vars_offset) =0;// 2 * w1 + 2 * w2 * dt * dt * rest_w[i+3]*10;
        }

        for (int i = 0; i < 3; ++i)
        {
            hessian.insert(secondchain_dof+i+vars_offset,secondchain_dof+i+vars_offset) = 10*2*w3;
        }

        for (int i = 3; i < 6; ++i)
        {
            hessian.insert(secondchain_dof+i+vars_offset,secondchain_dof+i+vars_offset) = 5*2*w4;
        }
    }
}


/****************************************************************/
void QPSolver::update_constraints()
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < chain_dof; ++j)
        {
            if (chain_dof != 10 || j != 1)
            {
                linearMatrix.coeffRef(i + chain_dof + 6, j) = J0(i, j);
            }
        }
    }
    if (secondchain_dof > 0) {
        for (int i = 0; i < 6; ++i) {
            linearMatrix.coeffRef(i + constr_offset + secondchain_dof + 6, 0) = J02(i, 0);
            linearMatrix.coeffRef(i + constr_offset + secondchain_dof + 6, 2) = J02(i, 2);
            for (int j = 0; j < secondchain_dof; ++j)
            {
                linearMatrix.coeffRef(i + constr_offset + secondchain_dof + 6, j+vars_offset) = J02(i, j+3);
            }
        }
    }
    solver.updateLinearConstraintsMatrix(linearMatrix);
}
