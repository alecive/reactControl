//
// Created by Jakub Rozlivek on 7/14/21.
//

#include "reactOSQP.h"


/****************************************************************/
void QPSolver::computeSelfAvoidanceConstraints()
{
    double joint1_0, joint1_1;
    double joint2_0, joint2_1;
    joint1_0= 28.0*CTRL_DEG2RAD;
    joint1_1= 23.0*CTRL_DEG2RAD;
    joint2_0=-37.0*CTRL_DEG2RAD;
    joint2_1= 80.0*CTRL_DEG2RAD;
    shou_m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    shou_n=joint1_0-shou_m*joint2_0;

    double joint3_0, joint3_1;
    double joint4_0, joint4_1;
    joint3_0= 85.0*CTRL_DEG2RAD;
    joint3_1=105.0*CTRL_DEG2RAD;
    joint4_0= 90.0*CTRL_DEG2RAD;
    joint4_1= 40.0*CTRL_DEG2RAD;
    elb_m=(joint4_1-joint4_0)/(joint3_1-joint3_0);
    elb_n=joint4_0-elb_m*joint3_0;
}

/****************************************************************/
void QPSolver::computeGuard()
{
    double guardRatio=0.1;
    qGuard.resize(chain_dof);
    qGuardMinExt.resize(chain_dof);
    qGuardMinInt.resize(chain_dof);
    qGuardMinCOG.resize(chain_dof);
    qGuardMaxExt.resize(chain_dof);
    qGuardMaxInt.resize(chain_dof);
    qGuardMaxCOG.resize(chain_dof);

    for (size_t i=0; i < chain_dof; i++)
    {
        qGuard[i]=0.25*guardRatio*(q_lim(i,1)-q_lim(i,0));

        qGuardMinExt[i]=q_lim(i,0)+qGuard[i];
        qGuardMinInt[i]=qGuardMinExt[i]+qGuard[i];
        qGuardMinCOG[i]=0.5*(qGuardMinExt[i]+qGuardMinInt[i]);

        qGuardMaxExt[i]=q_lim(i,1)-qGuard[i];
        qGuardMaxInt[i]=qGuardMaxExt[i]-qGuard[i];
        qGuardMaxCOG[i]=0.5*(qGuardMaxExt[i]+qGuardMaxInt[i]);
    }
}

/****************************************************************/
void QPSolver::computeBounds()
{
    for (size_t i=0; i < chain_dof; i++)
    {
        double qi=q0[i];
        if ((qi>=qGuardMinInt[i]) && (qi<=qGuardMaxInt[i]))
            bounds(i,0)=bounds(i,1)=1.0;
        else if (qi<qGuardMinInt[i])
        {
            bounds(i,0)=(qi<=qGuardMinExt[i]?0.0:
                         0.5*(1.0+tanh(+10.0*(qi-qGuardMinCOG[i])/qGuard[i])));
            bounds(i,1)=1.0;
        }
        else
        {
            bounds(i,0)=1.0;
            bounds(i,1)=(qi>=qGuardMaxExt[i]?0.0:
                         0.5*(1.0+tanh(-10.0*(qi-qGuardMaxCOG[i])/qGuard[i])));
        }
    }

    for (size_t i=0; i < chain_dof; i++)
    {
        bounds(i,0)*=v_lim(i,0);
        bounds(i,1)*=v_lim(i,1);
    }
}


//public:
/****************************************************************/
QPSolver::QPSolver(iCubArm &chain_, std::vector<ControlPoint> &additional_control_points_,
                             bool hittingConstraints_, bool orientationControl_, bool additionalControlPoints_,
                             double dT_, const Vector& restPos, double restPosWeight_) :
        arm(chain_), additional_control_points(additional_control_points_), hitting_constraints(hittingConstraints_),
        additional_control_points_flag(additionalControlPoints_), dt(dT_), extra_ctrl_points_nr(0),
        additional_control_points_tol(0.0001), shou_m(0), shou_n(0), elb_m(0), elb_n(0),
        w1(1), w2(restPosWeight_), w3(1), w4(0.1), min_type(1)
{
    chain_dof = static_cast<int>(arm.getDOF());
    pr.resize(3,0.0);
    v0.resize(chain_dof, 0.0); v=v0;
    q_lim.resize(chain_dof, 2);
    v_lim.resize(chain_dof, 2);
    normal.resize(3,0.0);
    iKinChain &chain=*arm.asChain();
    if (!orientationControl_)
        w4 = 0;
    for (size_t r=0; r < chain_dof; r++)
    {
        q_lim(r,0)=chain(r).getMin();
        q_lim(r,1)=chain(r).getMax();

        v_lim(r,1)=std::numeric_limits<double>::max();
        v_lim(r,0)=-v_lim(r,1);
    }
    bounds=v_lim;
    if (hitting_constraints)
        computeSelfAvoidanceConstraints();
    computeGuard();
    rest_jnt_pos = restPos;
    rest_weights = {1,1,1,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
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
    for (int i = 0; i < chain_dof+6; ++i) {
        linearMatrix.insert(i,i) = 1;
    }
    for (int i = 0; i < 6; ++i) {
        linearMatrix.insert(i+chain_dof+6,chain_dof+i) = -1;
    }

    linearMatrix.insert(chain_dof+12, 3) = 1.71 *dt;
    linearMatrix.insert(chain_dof+12, 4) = -1.71 *dt;
    linearMatrix.insert(chain_dof+12+1, 3) = 1.71 *dt;
    linearMatrix.insert(chain_dof+12+1, 4) = -1.71 *dt;
    linearMatrix.insert(chain_dof+12+1, 5) = -1.71 *dt;
    linearMatrix.insert(chain_dof+12+2, 4) = dt;
    linearMatrix.insert(chain_dof+12+2, 5) = dt;
    if (hitting_constraints) {
        linearMatrix.insert(chain_dof+12+3, 4) = dt;
        linearMatrix.insert(chain_dof+12+3, 5) = shou_m *dt;
        linearMatrix.insert(chain_dof+12+4, 6) = -elb_m *dt;
        linearMatrix.insert(chain_dof+12+4, 7) = dt;
        linearMatrix.insert(chain_dof+12+5, 6) = elb_m *dt;
        linearMatrix.insert(chain_dof+12+5, 7) = dt;
    }

    J0 = arm.GeoJacobian();
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < chain_dof; ++j) {
            if (chain_dof != 10 || j != 1) {
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
void QPSolver::init(const Vector &_xr, const Vector &_v0, const Matrix &_v_lim, const Vector &col_normal, double rest_pos_w)
{
    yAssert(6 <= _xr.length())
    yAssert(v0.length() == _v0.length())
    yAssert((v_lim.rows() == _v_lim.rows()) && (v_lim.cols() == _v_lim.cols()))
    for (int r=0; r < _v_lim.rows(); r++)
        yAssert(_v_lim(r, 0) <= _v_lim(r, 1));
    if (rest_pos_w >= 0)  // TODO: remember old value
        w2 = rest_pos_w;
    normal = col_normal; // TODO: add decay_rate
    v_lim= CTRL_DEG2RAD * _v_lim;
    v0= CTRL_DEG2RAD * _v0;
    q0=arm.getAng();
    H0=arm.getH();
    p0=H0.getCol(3).subVector(0,2);
    pr=_xr.subVector(0, 2);
    Vector ang=_xr.subVector(3,5);
    double ang_mag=norm(ang);
    if (ang_mag>0.0)
        ang/=ang_mag;
    ang.push_back(ang_mag);

    Matrix R = axis2dcm(ang).submatrix(0,2,0,2)*H0.submatrix(0,2,0,2).transposed();
    v_des.resize(6,0);
    v_des.setSubvector(0, (pr-p0) / dt);
    v_des.setSubvector(3, dcm2rpy(R) / dt);


    J0=arm.GeoJacobian();
    if (additional_control_points_flag)
    {
        //e.g., 0.0001 corresponds to 0.01 m error in actual Euclidean distance
        //N.B. for the end-effector, tolerance is 0
        if (additional_control_points.empty())
        {
            yWarning("[QPSolver::init()]: additional_control_points_flag is on but additional_control_points.size is 0.");
            additional_control_points_flag = false;
            extra_ctrl_points_nr = 0;
        }
        else
        {
            if (additional_control_points.size() == 1)
            {
                extra_ctrl_points_nr = 1;
            }
            else  //additional_control_points.size() > 1
            {
                extra_ctrl_points_nr = 1;
                yWarning("[QPSolver::init()]: currently only one additional control point - Elbow - is supported; requested %lu control points.",additional_control_points.size());
            }
            for (auto & additional_control_point : additional_control_points)
            {
                if(additional_control_point.type == "Elbow")
                {
                    Matrix H5=arm.getH(chain_dof - 4 - 1);
                    additional_control_point.p0 = H5.getCol(3).subVector(0,2);
                    Matrix J = arm.GeoJacobian(chain_dof - 4 - 1);
                    additional_control_point.J0_xyz = J.submatrix(0, 2, 0, chain_dof - 4 - 1);
                }
                else
                    yWarning("[QPSolver::get_nlp_info]: other control points type than Elbow are not supported (this was %s).",additional_control_point.type.c_str());
            }
        }
    }
    else
        extra_ctrl_points_nr = 0;

    computeBounds();
    update_gradient();
    update_constraints();
}


/****************************************************************/
void QPSolver::update_bounds(double pos_error)
{
    for (int i = 0; i < chain_dof; i++) {
        if (bounds(i,0) <= bounds(i,1)) {
            lowerBound[i] = bounds(i, 0);
            upperBound[i] = bounds(i, 1);
        } else {
            lowerBound[i] = upperBound[i] = 0;
        }
    }

    for (int i = 0; i < 3; ++i) {
        lowerBound[chain_dof+i] = -pos_error; // normal(i) != 0? -10 : 0; //  normal(i)/dt/10 : 0; // normal(i)/dt/10;
        upperBound[chain_dof+i] = pos_error; //normal(i) != 0?  pos_error : 0; //  normal(i)/dt/10 : 0;
    }

    for (int i = 3; i < 6; ++i) {
        lowerBound[chain_dof+i] = -std::numeric_limits<double>::max();
        upperBound[chain_dof+i] = std::numeric_limits<double>::max();
    }

    for (int i = 0; i < 6; ++i) {
        lowerBound[i+chain_dof+6] = v_des[i];
        upperBound[i+chain_dof+6] = v_des[i];
    }

    if(additional_control_points_flag)
    {
        for(int j=0; j<extra_ctrl_points_nr; j++){
            lowerBound[6+j+chain_dof+6]=0.0;
            upperBound[6+j+chain_dof+6]= additional_control_points_tol;
        }
    }

    // shoulder's cables length
    lowerBound[6+extra_ctrl_points_nr+chain_dof+6]=-347.00*CTRL_DEG2RAD-(1.71*(q0[3] - q0[4]));
    upperBound[6+extra_ctrl_points_nr+chain_dof+6]=std::numeric_limits<double>::max();
    lowerBound[7+extra_ctrl_points_nr+chain_dof+6]=-366.57*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    upperBound[7+extra_ctrl_points_nr+chain_dof+6]=112.42*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    lowerBound[8+extra_ctrl_points_nr+chain_dof+6]=-66.60*CTRL_DEG2RAD-(q0[4]+q0[5]);
    upperBound[8+extra_ctrl_points_nr+chain_dof+6]=213.30*CTRL_DEG2RAD-(q0[4]+q0[5]);
    if (hitting_constraints)
    {
        // avoid hitting torso
        lowerBound[9+extra_ctrl_points_nr+chain_dof+6]=shou_n - (q0[4] + shou_m*q0[5]);
        upperBound[9+extra_ctrl_points_nr+chain_dof+6]=std::numeric_limits<double>::max();

        // avoid hitting forearm
        lowerBound[10+extra_ctrl_points_nr+chain_dof+6]=-std::numeric_limits<double>::max();
        upperBound[10+extra_ctrl_points_nr+chain_dof+6]=elb_n - (-elb_m*q0[6] + q0[7]);
        lowerBound[11+extra_ctrl_points_nr+chain_dof+6]=-elb_n - (elb_m*q0[6] + q0[7]);
        upperBound[11+extra_ctrl_points_nr+chain_dof+6]=std::numeric_limits<double>::max();
    }
    solver.updateBounds(lowerBound,upperBound);
}


/****************************************************************/
void QPSolver::update_gradient()
{
    gradient.setZero();
    for (int i=0; i < chain_dof; i++) {
        gradient[i] = -2*w1*v0[i]*min_type +  w2 * rest_weights[i]* rest_weights[i]* dt * 2*(q0[i] - rest_jnt_pos[i]);
    }
    if (chain_dof == 10) {
        gradient[1] = 0.0;
    }
    solver.updateGradient(gradient);
}


/****************************************************************/
void QPSolver::set_hessian()
{
    for (int i = 0; i < chain_dof; ++i) {
        if (chain_dof != 10 || i != 1) {
            hessian.insert(i, i) = 2 * w1 + 2 * w2 * dt * dt * rest_weights[i] * rest_weights[i];
        }
    }

    for (int i = 0; i < 3; ++i) {
        hessian.insert(chain_dof+i,chain_dof+i) = 2*w3;
    }

    for (int i = 3; i < 6; ++i) {
        hessian.insert(chain_dof+i,chain_dof+i) = 2*w4;
    }
}


/****************************************************************/
void QPSolver::update_constraints()
{
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < chain_dof; ++j) {
            if (chain_dof != 10 || j != 1) {
                linearMatrix.coeffRef(i + chain_dof + 6, j) = J0(i, j);
            }
        }
    }
    solver.updateLinearConstraintsMatrix(linearMatrix);
}
