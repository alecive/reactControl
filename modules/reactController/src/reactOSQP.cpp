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
    qGuardMinExt.resize(chain_dof);
    qGuardMinInt.resize(chain_dof);
    qGuardMaxExt.resize(chain_dof);
    qGuardMaxInt.resize(chain_dof);
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
}

/****************************************************************/
void QPSolver::computeBounds()
{
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
}


//public:
/****************************************************************/
QPSolver::QPSolver(iCubArm &chain_, bool hittingConstraints_, double vmax_, bool orientationControl_,
                             double dT_, const Vector& restPos, double restPosWeight_) :
        arm(chain_), hitting_constraints(hittingConstraints_), dt(dT_), shou_m(0), shou_n(0), elb_m(0), elb_n(0),
        w1(1), w2(restPosWeight_), w3(1), w4(0.1), w5(1),  min_type(1), vmax(vmax_)
{
    chain_dof = static_cast<int>(arm.getDOF());
    pr.resize(3,0.0);
    v0.resize(chain_dof, 0.0); v=v0;
    v_lim.resize(chain_dof, 2);
    bounds.resize(chain_dof, 2);
    manip.resize(chain_dof);
   // normal.resize(3,0.0);
    if (!orientationControl_) w4 = 0;

    if (hitting_constraints)
    {
        computeSelfAvoidanceConstraints();
    }
    computeGuard();
    rest_jnt_pos = restPos;
    rest_w = {1, 1, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    rest_err.resize(chain_dof, 0.0);

    hessian.resize(chain_dof+6, chain_dof+6);
    set_hessian();
    gradient.resize(chain_dof+6);
    gradient.setZero();
    lowerBound.resize(6 + chain_dof+6 + 3 +  hitting_constraints*3);
    lowerBound.setZero();
    upperBound.resize(6 + chain_dof+6 + 3 + hitting_constraints*3);
    upperBound.setZero();
    linearMatrix.resize(6 + chain_dof+6 + 3 + hitting_constraints*3, chain_dof+6);
    for (int i = 0; i < chain_dof+6; ++i)
    {
        linearMatrix.insert(i,i) = 1;
    }
    for (int i = 0; i < 6; ++i)
    {
        linearMatrix.insert(i+chain_dof+6,chain_dof+i) = -1;
    }

    linearMatrix.insert(chain_dof+12, 3) = 1.71 *dt;
    linearMatrix.insert(chain_dof+12, 4) = -1.71 *dt;
    linearMatrix.insert(chain_dof+12+1, 3) = 1.71 *dt;
    linearMatrix.insert(chain_dof+12+1, 4) = -1.71 *dt;
    linearMatrix.insert(chain_dof+12+1, 5) = -1.71 *dt;
    linearMatrix.insert(chain_dof+12+2, 4) = dt;
    linearMatrix.insert(chain_dof+12+2, 5) = dt;
    if (hitting_constraints)
    {
        linearMatrix.insert(chain_dof+12+3, 4) = dt;
        linearMatrix.insert(chain_dof+12+3, 5) = shou_m *dt;
        linearMatrix.insert(chain_dof+12+4, 6) = -elb_m *dt;
        linearMatrix.insert(chain_dof+12+4, 7) = dt;
        linearMatrix.insert(chain_dof+12+5, 6) = elb_m *dt;
        linearMatrix.insert(chain_dof+12+5, 7) = dt;
    }

    J0 = arm.GeoJacobian();
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < chain_dof; ++j)
        {
            if (chain_dof != 10 || j != 1)
            {
                linearMatrix.insert(i + chain_dof + 6, j) = J0(i, j);
            }
        }
    }

    solver.data()->setNumberOfVariables(chain_dof+6);
    solver.data()->setNumberOfConstraints(6 + chain_dof+6 + 3 + hitting_constraints*3); //hiting_constraints*6
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
void QPSolver::init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, double rest_pos_w)
{
    yAssert(6 <= _xr.length())
    yAssert(v0.length() == _v0.length())
    yAssert((v_lim.rows() == _v_lim.rows()) && (v_lim.cols() == _v_lim.cols()))
    for (int r=0; r < _v_lim.rows(); r++)
    {
        yAssert(_v_lim(r, 0) <= _v_lim(r, 1));
    }
    if (rest_pos_w >= 0)   // TODO: remember old value
    {
        w2 = rest_pos_w;
    }
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
    Vector offset = Vector(chain_dof,0.0);
    double delta = 0.02;
    for (int i = 0; i < chain_dof; ++i) {
        offset[i] = delta;
        Matrix Jplus = arm.GeoJacobian(q0+offset); // Be aware - arm joint angles are changed
        Matrix Jminus = arm.GeoJacobian(q0-offset);
        manip[i] = (sqrt(det(Jplus*Jplus.transposed())) - sqrt(det(Jminus*Jminus.transposed())))/(2*delta);
        offset[i] = 0.0;
    }
    arm.setAng(q0); //restore original joint angles

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
    solver.updateBounds(lowerBound,upperBound);
}


/****************************************************************/
void QPSolver::update_gradient()
{
    gradient.setZero();

    for (int i=0; i < chain_dof; i++)
    {
        gradient[i] = -2*w1*v0[i]*min_type + w2 * rest_w[i] * dt * 2 * (q0[i] - rest_jnt_pos[i]) - w5 * dt * manip[i];
    }
    if (chain_dof == 10)
    {
        gradient[1] = 0.0;
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
    solver.updateLinearConstraintsMatrix(linearMatrix);
}
