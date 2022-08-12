//
// Created by Jakub Rozlivek on 7/14/21.
//

#include "reactOSQP.h"

ArmHelper::ArmHelper(iCubArm *chain_, double dt_, int offset_,  double vmax_, double manip_thr_, const Vector& restPos,
                     bool hitting_constr_, int vars_offset_, int constr_offset_):
        arm(chain_), offset(offset_), dt(dt_), vmax(vmax_), manip_thr(manip_thr_), adapt_w5(1), rest_jnt_pos(restPos),
        hitting_constraints(hitting_constr_), vars_offset(vars_offset_), constr_offset(constr_offset_)
{
    chain_dof = static_cast<int>(arm->getDOF())-offset;
    v0.resize(chain_dof, 0.0);
    v_lim.resize(chain_dof, 2);
    manip.resize(chain_dof);
    rest_w = {1, 1, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    J0 = arm->GeoJacobian();
    computeGuard();
}

void ArmHelper::init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim)
{
    yAssert(7 <= _xr.length());
    yAssert(v0.length() == _v0.length());
    yAssert((v_lim.rows() == _v_lim.rows()) && (v_lim.cols() == _v_lim.cols()));
    for (int r=0; r < _v_lim.rows(); r++)
    {
        yAssert(_v_lim(r, 0) <= _v_lim(r, 1));
    }
    v_lim= CTRL_DEG2RAD * _v_lim;
    v0= CTRL_DEG2RAD * _v0;
    q0=arm->getAng();
    Matrix H0=arm->getH();
    Vector p0=H0.getCol(3).subVector(0,2);
    Vector pr=_xr.subVector(0, 2);
    Vector ang=_xr.subVector(3,6);
    Matrix R = axis2dcm(ang).submatrix(0,2,0,2)*H0.submatrix(0,2,0,2).transposed();
    v_des.resize(6,0);
    v_des.setSubvector(0, (pr-p0) / dt);
    v_des.setSubvector(3, dcm2rpy(R) / dt);
    J0=arm->GeoJacobian();

    adapt_w5 = (manip_thr > 0)? 1 - sqrt(det(J0*J0.transposed()))/manip_thr : 0;  // not used anymore
    if (adapt_w5 <= 0)
    {
        adapt_w5 = 0;
    }
    else
    {
        Vector off = Vector(chain_dof+offset, 0.0);
        double delta = 0.02;
        for (int i = 0; i < chain_dof; ++i) {
            off[i+offset] = delta;
            Matrix Jplus = arm->GeoJacobian(q0 + off); // Be aware - arm joint angles are changed
            Matrix Jminus = arm->GeoJacobian(q0 - off);
            manip[i] = (sqrt(det(Jplus * Jplus.transposed())) - sqrt(det(Jminus * Jminus.transposed()))) / (2 * delta);
            off[i+offset] = 0.0;
        }
        arm->setAng(q0); //restore original joint angles
    }
    computeBounds();
}

void ArmHelper::computeGuard()
{
    double guardRatio=0.1;
    qGuardMinExt.resize(chain_dof);
    qGuardMinInt.resize(chain_dof);
    qGuardMaxExt.resize(chain_dof);
    qGuardMaxInt.resize(chain_dof);
    iKinChain &chain=*arm->asChain();

    for (size_t i=0; i < chain_dof; i++)
    {
        double q_min = chain(i+offset).getMin();
        double q_max = chain(i+offset).getMax();
        double qGuard=0.25*guardRatio*(q_max-q_min);

        qGuardMinExt[i]=q_min+qGuard;
        qGuardMinInt[i]=qGuardMinExt[i]+qGuard;
        qGuardMaxExt[i]=q_max-qGuard;
        qGuardMaxInt[i]=qGuardMaxExt[i]-qGuard;
    }
}

void ArmHelper::computeBounds()
{
    bounds.resize(chain_dof, 2);
    for (size_t i=0; i < chain_dof; i++)
    {
        double qi=q0[i+offset];
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
        bounds(i, 0) = std::max(dmin*-vmax, v_lim(i+offset, 0));  // apply joint limit bounds only when it is stricter than the avoidance limits
        bounds(i, 1) = std::min(dmax*vmax, v_lim(i+offset, 1));
    }
}

void ArmHelper::updateBounds(Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound, double pos_error)
{
    for (int i = 0; i < chain_dof; i++)
    {
        if (bounds(i,0) <= bounds(i,1))
        {
            lowerBound[i+constr_offset] = bounds(i, 0);
            upperBound[i+constr_offset] = bounds(i, 1);
        }
        else
        {
            lowerBound[i+constr_offset] = upperBound[i+constr_offset] = 0;
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        lowerBound[chain_dof+i+constr_offset] = -pos_error; // normal(i) != 0? -10 : 0; //  normal(i)/dt/10 : 0; // normal(i)/dt/10;
        upperBound[chain_dof+i+constr_offset] = pos_error; //normal(i) != 0?  pos_error : 0; //  normal(i)/dt/10 : 0;
    }

    for (int i = 3; i < 6; ++i)
    {
        lowerBound[chain_dof+i+constr_offset] = -std::numeric_limits<double>::max();
        upperBound[chain_dof+i+constr_offset] = std::numeric_limits<double>::max();
    }

    for (int i = 0; i < 6; ++i)
    {
        lowerBound[i+chain_dof+6+constr_offset] = v_des[i];
        upperBound[i+chain_dof+6+constr_offset] = v_des[i];
    }

    // shoulder's cables length
    lowerBound[6+chain_dof+6+constr_offset]=-347.00*CTRL_DEG2RAD-(1.71*(q0[3] - q0[4]));
    upperBound[6+chain_dof+6+constr_offset]=std::numeric_limits<double>::max();
    lowerBound[7+chain_dof+6+constr_offset]=-366.57*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    upperBound[7+chain_dof+6+constr_offset]=112.42*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    lowerBound[8+chain_dof+6+constr_offset]=-66.60*CTRL_DEG2RAD-(q0[4]+q0[5]);
    upperBound[8+chain_dof+6+constr_offset]=213.30*CTRL_DEG2RAD-(q0[4]+q0[5]);
    if (hitting_constraints)
    {
        // avoid hitting torso
        lowerBound[9+chain_dof+6+constr_offset]=-shou_n - (q0[4] + shou_m*q0[5]);
        upperBound[9+chain_dof+6+constr_offset]=std::numeric_limits<double>::max();

        // avoid hitting forearm
        lowerBound[10+chain_dof+6+constr_offset]=-std::numeric_limits<double>::max();
        upperBound[10+chain_dof+6+constr_offset]=elb_n - (-elb_m*q0[6] + q0[7]);
        lowerBound[11+chain_dof+6+constr_offset]=-elb_n - (elb_m*q0[6] + q0[7]);
        upperBound[11+chain_dof+6+constr_offset]=std::numeric_limits<double>::max();
    }
}

void ArmHelper::addConstraints(Eigen::SparseMatrix<double>& linearMatrix) const
{
    for (int i = 0; i < chain_dof + 6; ++i)
    {
        linearMatrix.insert(i + constr_offset, i + vars_offset) = 1;
    }
    for (int i = 0; i < 6; ++i)
    {
        linearMatrix.insert(i + constr_offset + chain_dof + 6, vars_offset + chain_dof + i) = -1;
    }
    linearMatrix.insert(chain_dof + 12 + constr_offset, 0 + vars_offset) = 1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + constr_offset, 1 + vars_offset) = -1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 1 + constr_offset, 0 + vars_offset) = 1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 1 + constr_offset, 1 + vars_offset) = -1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 1 + constr_offset, 2 + vars_offset) = -1.71 * dt;
    linearMatrix.insert(chain_dof + 12 + 2 + constr_offset, 1 + vars_offset) = dt;
    linearMatrix.insert(chain_dof + 12 + 2 + constr_offset, 2 + vars_offset) = dt;
    if (hitting_constraints)
    {
        linearMatrix.insert(chain_dof + 12 + 3 + constr_offset, 1 + vars_offset) = dt;
        linearMatrix.insert(chain_dof + 12 + 3 + constr_offset, 2 + vars_offset) = shou_m * dt;
        linearMatrix.insert(chain_dof + 12 + 4 + constr_offset, 3 + vars_offset) = -elb_m * dt;
        linearMatrix.insert(chain_dof + 12 + 4 + constr_offset, 4 + vars_offset) = dt;
        linearMatrix.insert(chain_dof + 12 + 5 + constr_offset, 3 + vars_offset) = elb_m * dt;
        linearMatrix.insert(chain_dof + 12 + 5 + constr_offset, 4 + vars_offset) = dt;
    }

    for (int i = 0; i < 6; ++i)
    {
        linearMatrix.insert(i + constr_offset + chain_dof + 6, 0) = J0(i, 0);
        linearMatrix.insert(i + constr_offset + chain_dof + 6, 2) = J0(i, 2);
        for (int j = 3; j < chain_dof+offset; ++j)
        {
            linearMatrix.insert(i + constr_offset + chain_dof + 6, j - offset + vars_offset) = J0(i, j);
        }
    }
}

//public:
/****************************************************************/
QPSolver::QPSolver(iCubArm *chain_, bool hitConstr, iCubArm* second_chain_, double vmax_, bool orientationControl_,
                             double dT_, const Vector& restPos, double restPosWeight_) :
        second_arm(nullptr), dt(dT_), w1(1), w2(restPosWeight_), orig_w2(restPosWeight_), w3(10), w4(1)
{
    double manip_thr = 0.0;  // not used anymore
    main_arm = std::make_unique<ArmHelper>(chain_, dt, 0, vmax_*CTRL_DEG2RAD, manip_thr, restPos, hitConstr);

    vars_offset = main_arm->chain_dof + 6;
    constr_offset = main_arm->chain_dof + 12 + 3 + hitConstr * 3;
    second_arm = second_chain_ ? std::make_unique<ArmHelper>(second_chain_, dt, 3, vmax_*CTRL_DEG2RAD, manip_thr,restPos,
                                                        hitConstr,vars_offset, constr_offset) : nullptr;
    if (!orientationControl_) w4 = 0;
    int vars = vars_offset;
    int constr = constr_offset;
    if (second_arm)
    {
        vars += second_arm->chain_dof + 6;
        constr += 6 + second_arm->chain_dof + 6 + 3 + hitConstr * 3;
    }
    hessian.resize(vars, vars);
    set_hessian();
    gradient.resize(vars);
    gradient.setZero();
    lowerBound.resize(constr);
    lowerBound.setZero();
    upperBound.resize(constr);
    upperBound.setZero();
    linearMatrix.resize(constr, vars);

    main_arm->addConstraints(linearMatrix);
    if (second_arm != nullptr) second_arm->addConstraints(linearMatrix);

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
void QPSolver::init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, double rest_pos_w,
                    const Vector &_xr2, const Vector &_v02, const Matrix &_v2_lim)
{
    w2 = (rest_pos_w >= 0) ? rest_pos_w : orig_w2;
    main_arm->init(_xr, _v0, _v_lim);
    if (second_arm) second_arm->init(_xr2, _v02, _v2_lim);
    update_gradient();
    update_constraints();
}


/****************************************************************/
void QPSolver::update_bounds(double pos_error)
{
    main_arm->updateBounds(lowerBound, upperBound, pos_error);
    if (second_arm) second_arm->updateBounds(lowerBound, upperBound, std::numeric_limits<double>::max());
    solver.updateBounds(lowerBound,upperBound);
}


/****************************************************************/
void QPSolver::update_gradient()
{
    gradient.setZero();

    for (int i=0; i < main_arm->chain_dof; i++)
    {
        gradient[i] = w2 * main_arm->rest_w[i] * dt * 2 * (main_arm->q0[i] - main_arm->rest_jnt_pos[i]); // - w5 * main_arm->adapt_w5 * dt * main_arm->manip[i];
    }
    if (main_arm->chain_dof == 10)
    {
        gradient[1] = 0.0;
    }
    if (second_arm != nullptr) {
        for (int i=0; i < second_arm->chain_dof; i++)
        {
            gradient[i+vars_offset] = 0;//-2 * w1 * second_arm->v0[i] * min_type + w2 * second_arm->rest_w[i+3]*10 * dt * 2 * (second_arm->q0[i+3] - second_arm->rest_jnt_pos[i+3]) - w5 * second_arm->adapt_w5 * dt * second_arm->manip[i];
        }
    }
    solver.updateGradient(gradient);
}


/****************************************************************/
void QPSolver::set_hessian()
{
    for (int i = 0; i < main_arm->chain_dof; ++i)
    {
        if (main_arm->chain_dof != 10 || i != 1)
        {
            hessian.insert(i, i) = 2 * w1*main_arm->rest_w[i] + 2 * w2 * dt * dt * main_arm->rest_w[i];
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        hessian.insert(main_arm->chain_dof+i,main_arm->chain_dof+i) = 2*w3;
    }

    for (int i = 3; i < 6; ++i)
    {
        hessian.insert(main_arm->chain_dof+i,main_arm->chain_dof+i) = 2*w4;
    }
    if (second_arm != nullptr)
    {
        for (int i = 0; i < second_arm->chain_dof; ++i)
        {
            hessian.insert(i+vars_offset, i+vars_offset) = 2 * w1 + 2 * w2 * dt * dt * second_arm->rest_w[i+3];
        }

        for (int i = 0; i < 3; ++i)
        {
            hessian.insert(second_arm->chain_dof+i+vars_offset,second_arm->chain_dof+i+vars_offset) = 2*w3;
        }

        for (int i = 3; i < 6; ++i)
        {
            hessian.insert(second_arm->chain_dof+i+vars_offset,second_arm->chain_dof+i+vars_offset) = 2*w4;
        }
    }
}


/****************************************************************/
void QPSolver::update_constraints()
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < main_arm->chain_dof; ++j)
        {
            if (main_arm->chain_dof != 10 || j != 1)
            {
                linearMatrix.coeffRef(i + main_arm->chain_dof + 6, j) = main_arm->J0(i, j);
            }
        }
    }
    if (second_arm != nullptr)
    {
        for (int i = 0; i < 6; ++i)
        {
            linearMatrix.coeffRef(i + constr_offset + second_arm->chain_dof + 6, 0) = second_arm->J0(i, 0);
            linearMatrix.coeffRef(i + constr_offset + second_arm->chain_dof + 6, 2) = second_arm->J0(i, 2);
            for (int j = 0; j < second_arm->chain_dof; ++j)
            {
                linearMatrix.coeffRef(i + constr_offset + second_arm->chain_dof + 6, j+vars_offset) = second_arm->J0(i, j+3);
            }
        }
    }
    solver.updateLinearConstraintsMatrix(linearMatrix);
}

Vector QPSolver::get_resultInDegPerSecond()
{
    Eigen::VectorXd sol = solver.getSolution();
    int dim = main_arm->chain_dof;
    if (second_arm) dim += second_arm->chain_dof;
    Vector v(dim,0.0);
    for (int i = 0; i < main_arm->chain_dof; ++i)
    {
        v[i] = std::max(std::min(sol[i], upperBound[i]), lowerBound[i]);
    }
    if (second_arm)
    {
        for (int i = 0; i < second_arm->chain_dof; ++i)
        {
            v[i + main_arm->chain_dof] = std::max(std::min(sol[i + vars_offset], upperBound[i + constr_offset]), lowerBound[i + constr_offset]);
        }
    }
    return CTRL_RAD2DEG*v;
}

int QPSolver::optimize(double pos_error)
{
    update_bounds(pos_error);
    Eigen::VectorXd primalVar(hessian.rows());
    primalVar.setZero();
    for (int i = 0; i < main_arm->chain_dof; i++)
    {
        primalVar[i] = std::min(std::max(main_arm->bounds(i, 0), main_arm->v0[i]), main_arm->bounds(i, 1));
    }
    if (second_arm != nullptr)
    {
        for (int i = 0; i < second_arm->chain_dof; i++)
        {
            primalVar[i+vars_offset] = std::min(std::max(second_arm->bounds(i, 0), second_arm->v0[i+3]), second_arm->bounds(i, 1));
        }
    }
    solver.setPrimalVariable(primalVar);
    solver.solve();
    return static_cast<int>(solver.workspace()->info->status_val);
}
