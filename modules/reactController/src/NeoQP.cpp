//
// Created by rozliv on 25.7.22.
//

#include "NeoQP.h"

#include <fstream>

using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::iKin;
using namespace iCub::skinDynLib;

void NeoQP::generateRobotColPoints(yarp::sig::Vector* data)
{
    selfColPoints.resize(4);
        // front chest, back, face, back of head, ears (2x), hip (3x), front chest low band (relative coords to SKIN_FRONT_TORSO)
        std::vector<std::vector<double>> posx{{-0.13, -0.122, -0.084}, {0.05,  0.065, 0.08}, {-0.12, -0.112, -0.082},
                                               {0.07,  0.105,  0.115}, {-0.03, 0., 0.03}, {-0.03, -0.01, 0.01},
                                               {-0.03, 0.}, {-0.03, 0.}, {-0.03, 0.}, {-0.11, -0.102, -0.064}};
        std::vector<std::vector<double>> posy{{0.02, -0.05, -0.12}, {0.02, -0.05, -0.12, -0.19}, {0.09, 0.17, 0.25},
                                               {0.09, 0.17,  0.25}, {0.09, 0.18}, {0.27},
                                               {-0.05}, {-0.12}, {-0.19}, {-0.19}};
        std::vector<std::vector<double>> posz{{-0.025, 0.03,   0.085}, {-0.085, -0.025, 0.045}, {-0.04, 0.025, 0.09},
                                               {-0.1, -0.035, 0.03}, {-0.095, -0.005, 0.085}, {-0.075, -0.005, 0.065},
                                               {-0.09, 0.09}, {-0.1, 0.1}, {-0.11, 0.11}, {-0.025, 0.03,   0.085}};

        for (int l = 0; l < posz.size(); ++l)
        {
            for (int i = 0; i < posy[l].size(); ++i)
            {
                for (int j = 0; j < posz[l].size(); ++j)
                {
                    if (l == 4 && i == 1 && j == 1) continue;
                    selfColPoints[3].push_back({posx[l][j], posy[l][i], posz[l][j], 1});
                }
            }
        }
        selfColPoints[0] = {{-0.0049, 0.0012,  0.0}, {-0.0059, 0.0162,  0.0}, {-0.0199, 0.0122,  0.0}, {-0.0049, -0.0143,  0.0}, {-0.0304, 0.0122,  0.0}};
        std::vector<std::string> fingers{"thumb", "index", "middle", "ring", "little"};
        for (int i = 0; i < 5; i++)
        {
            iCubFinger finger(part+"_"+fingers[i]);
            Vector vec;
            finger.getChainJoints(*data, vec);
            finger.setAng((M_PI/180.0)*vec);
            printf("%s pos is \t %s\n", fingers[i].c_str(), finger.getH().subcol(0,3,3).toString().c_str());
            selfColPoints[0].push_back(finger.getH().subcol(0,3,3));
        }
        if (part == "left")
        {
            // selfcol with right forearm
            selfColPoints[1] = {{0.0112, 0.0872, -0.0450}, {-0.0297, 0.0899, -0.0237}, {-0.0277, 0.0557, 0.0143}, {0.0022, 0.0510, -0.0426}, {0.0249, 0.0457, 0.0208},
                                {-0.0275, 0.0571, -0.0237}, {0.0307, 0.0932, -0.0224}, {0.0241, 0.0744, -0.0347}, {0.0254, 0.0571, 0.0184}, {-0.0156, 0.0677, -0.0408},
                                {0.0281, 0.0566, -0.0243}, {-0.0114, 0.0351, 0.0301}, {0.0313, 0.0643, 0.0101}, {0.0102, 0.0318, 0.0309}, {-0.0117, 0.0898, -0.0444}};
            // selfcol with right arm
            selfColPoints[2] = {{0.0579, -0.0643, 0.0159}, {-0.0188, -0.1008, 0.0184}, {0.0542, 0.0040, -0.0178}, {0.0303, -0.0565, -0.0299}, {0.0254, -0.0885, -0.0309},
                                {0.0471, -0.0916, -0.0261}, {0.0249, -0.0897, 0.0329}, {-0.0288, -0.0811, 0.0004}, {0.0327, -0.0133, 0.0317}, {0.0599, -0.0302, -0.0006},
                                {0.0445, -0.0428, 0.0297}, {0.0569, -0.0727, -0.0185}, {-0.0087, -0.0931, -0.0270}, {0.0594, -0.0455, 0.0136}, {0.0553, -0.0237, -0.0204},
                                {0.0311, -0.0135, -0.0299}, {-0.0169, -0.0813, 0.0263}, {0.0505, -0.0030, 0.0258}, {-0.0198, -0.1015, -0.0080}, {0.0498, -0.0862, 0.0276},
                                {0.0165, -0.0341, -0.0310}, {0.0167, -0.0313, 0.0336}, {-0.0151, -0.0707, -0.0251}, {0.0245, -0.0622, 0.0329}, {0.0579, -0.0491, -0.0170}};
        }
        else
        {
            // selfcol with left forearm
            selfColPoints[1] = {{0.0105, -0.0891, 0.0443}, {-0.0320, -0.0924, 0.0232}, {-0.0297, -0.0553, -0.0149}, {0.0265, -0.0569, 0.0231}, {-0.0243, -0.0490, 0.0323},
                                {0.0219, -0.0472, -0.0214}, {-0.0131, -0.0344, -0.0303}, {0.0089, -0.0494, 0.0422}, {0.0282, -0.0637, -0.0111}, {0.0278, -0.0883, 0.0243},
                                {-0.0142, -0.0906, 0.0443}, {0.0077, -0.0307, -0.0308}, {0.0132, -0.0673, 0.0416}, {-0.0144, -0.0681, 0.0433}, {-0.0309, -0.0683, 0.0231}};
            // selfcol with left arm
            selfColPoints[2] = {{-0.0480, 0.0274, 0.0287}, {-0.0592, 0.0235, -0.0131}, {0.0133, 0.1019, -0.0175}, {-0.0498, 0.0862, 0.0276}, {-0.0254, 0.0885, -0.0309},
                                {-0.0238, 0.0205, -0.0306}, {-0.0447, 0.0312, -0.0278}, {0.0194, 0.1016, 0.0156}, {-0.0522, -0.0014, -0.0198}, {-0.0600, 0.0605, 0.0011},
                                {-0.0249, 0.0897, 0.0329}, {0.0133, 0.0773, -0.0257}, {-0.0596, 0.0369, 0.0118}, {-0.0439, 0.0020, 0.0277}, {0.0171, 0.0819, 0.0262},
                                {-0.0173, 0.0288, 0.0336}, {-0.0520, 0.0587, 0.0246}, {-0.0587, 0.0429, -0.0151}, {0.0285, 0.0830, 0.0001}, {-0.0301, 0.0469, 0.0322},
                                {-0.0463, 0.0921, -0.0264}, {-0.0575, 0.0763, -0.0188}, {-0.0229, 0.0553, -0.0307}, {-0.0218, 0.0659, 0.0333}, {-0.0513, 0.0608, -0.0247}};
        }
        for (int i = 0; i < 3; ++i)
        {
            for (auto & j : selfColPoints[i])
            {
                j.push_back(1);
            }
        }
}

//creates a full transform as given by a DCM matrix at the pos and norm w.r.t. the original frame, from the pos and norm (one axis set arbitrarily)
bool NeoQP::computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR)
{
    if (norm == zeros(3))
    {
        FoR=eye(4);
        return false;
    }

    yarp::sig::Vector y(3,0.0);
    yarp::sig::Vector z = norm;
    if (z[0] == 0.0)
    {
        z[0] = 0.00000001;    // Avoid the division by 0
    }
    y[0] = -z[2]/z[0]; //y is in normal plane
    y[2] = 1; //this setting is arbitrary
    yarp::sig::Vector x = -1*(cross(z,y));

    // Let's make them unitary vectors:
    x = x / yarp::math::norm(x);
    y = y / yarp::math::norm(y);
    z = z / yarp::math::norm(z);

    FoR=eye(4);
    FoR.setSubcol(x,0,0);
    FoR.setSubcol(y,0,1);
    FoR.setSubcol(z,0,2);
    FoR.setSubcol(pos,0,3);

    return true;
}

/****************************************************************/
std::deque<Vector> NeoQP::getCtrlPointsPosition()
{
    std::deque<Vector> ctrlPoints;
    for (auto & ctrlPointPos : ctrlPointsPositions)
    {
        ctrlPoints.push_back(ctrlPointPos);
    }
    return ctrlPoints;
}

void NeoQP::checkCollisions(const std::vector<Vector> &obstacles)
{
    totalColPoints.clear();
    std::vector<Matrix> transforms;
//    transforms.reserve(4);
    std::vector<int> indexes = {SkinPart_2_LinkNum[SKIN_LEFT_HAND].linkNum + 3,
                                SkinPart_2_LinkNum[SKIN_LEFT_FOREARM].linkNum + 3,
                                SkinPart_2_LinkNum[SKIN_LEFT_UPPER_ARM].linkNum + 3,
                                SkinPart_2_LinkNum[SKIN_FRONT_TORSO].linkNum};
    std::vector<SkinPart> skinparts = (part == "left")? std::vector{SKIN_LEFT_HAND, SKIN_LEFT_FOREARM, SKIN_LEFT_UPPER_ARM, SKIN_FRONT_TORSO} :
                                                       std::vector{SKIN_RIGHT_HAND, SKIN_RIGHT_FOREARM, SKIN_RIGHT_UPPER_ARM, SKIN_FRONT_TORSO};
    for (int i = 0; i < 4; ++i)
    {
        transforms.push_back(arm->getH(indexes[i]));
    }
    int index = 0;
    for (const auto & obstacle : obstacles) {
        for (int j = 0; j < transforms.size(); ++j) {
            double neardist = std::numeric_limits<double>::max();
            int nearest = -1;
            for (int k = 0; k < selfColPoints[j].size(); k++) { //const auto& colPoint : selfColPoints[j]) {
                const Vector pos = transforms[j] * selfColPoints[j][k];
                {
                    const double n = yarp::math::norm2(pos.subVector(0, 2) - obstacle);
                    if (n < neardist) {
                        nearest = k;
                        neardist = n;
                    }
                }
            }
            neardist = sqrt(neardist);
            if (neardist < di)
            {
                const Vector pos = transforms[j] * selfColPoints[j][nearest];
//                auto obs_pos = obstacle;
//                obs_pos.push_back(1);
//                obs_pos = SE3inv(transforms[j]) * obs_pos;
//                obs_pos.pop_back();
                const Vector n = obstacle - pos.subVector(0, 2);
                const colPoint_t cp {skinparts[j], selfColPoints[j][nearest].subVector(0,2), obstacle, n / yarp::math::norm(n), {0,0,0}, neardist};
                totalColPoints.push_back(cp);
                yDebug("colPoint %s with pos = %s and dist = %.3f\n", SkinPart_s[skinparts[j]].c_str(), cp.obs.toString().c_str(), neardist);
            }
            index++;
        }
    }
}


/****************************************************************/
void NeoQP::computeObstacles(const std::vector<Vector> &obstacles)
{
    ctrlPointsPositions.clear();
    checkCollisions(obstacles);
    const int dim_offset = 3;  // 3 if dim == 10; 0 if dim == 7
    int i = 0;
    for(const auto & colPoint : totalColPoints)
    {
        double ds_ = ds;
        iKinChain customChain = *arm->asChain(); //instantiates a new chain, copying from the old (full) one

        if ((colPoint.skin_part == SKIN_LEFT_FOREARM) || (colPoint.skin_part == SKIN_RIGHT_FOREARM))
        {
            ds_ = 0.08;
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset);
        }
        else if ((colPoint.skin_part == SKIN_LEFT_UPPER_ARM) || (colPoint.skin_part == SKIN_RIGHT_UPPER_ARM))
        {
            ds_ = 0.05;
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset);
            customChain.rmLink(4+dim_offset); customChain.rmLink(3+dim_offset);
        }
        else if (colPoint.skin_part == SKIN_FRONT_TORSO)
        {
            ds_ = 0.05;
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset); customChain.rmLink(4+dim_offset);
            customChain.rmLink(3+dim_offset); customChain.rmLink(2+dim_offset); customChain.rmLink(1+dim_offset);
            customChain.rmLink(0+dim_offset);
        }
        // SetHN to move the end effector toward the point to be controlled - the average locus of collision threat from safety margin
        yarp::sig::Matrix HN = eye(4);
        computeFoR(colPoint.x, colPoint.n, HN);
        customChain.setHN(HN); //setting the end-effector transform to the collision point w.r.t subchain

        const Matrix J=customChain.GeoJacobian().submatrix(0,2,0,customChain.getDOF()-1); //first 3 rows ~ dPosition/dJoints - Jd
//        Vector normal = customChain.getH().getCol(2).subVector(0,2); //get the end-effector frame of the standard or custom chain (control point derived from skin), takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the 4 in the homogenous transf. format
//        Vector s=(J.transposed()*normal) * avoidingSpeed * colPoint.magnitude; //project movement along the normal into joint velocity space and scale by default avoidingSpeed and magnitude of skin (or PPS) activation
//        Vector n = (colPoint.obs - colPoint.x)/norm(colPoint.obs - colPoint.x);
//        printf("%s\n", customChain.getH().toString(3).c_str());
//        auto R = customChain.getH().submatrix(0,2,0,2).transposed();
//        Matrix Jev;
//        Jev.resize(6,6);
//        Jev.zero();
//        Jev.setSubmatrix(R, 0,0);
//        Jev.setSubmatrix(R, 3,3);
//        J = R * J;
        ds_ = 0.05;
        Aobstacles[i].setSubvector(0,J.transposed()*colPoint.n);
        bvalue[i] = w_eps*(colPoint.distance-ds)/(di-ds_) - dot(colPoint.n, colPoint.vel);
        i++;
        ctrlPointsPositions.push_back(customChain.EndEffPosition());
    }
}


void NeoQP::computePosLims()
{
    iKinChain &chain=*arm->asChain();
    for (size_t i=0; i < chain_dof; i++)
    {
        pos_lim(i,0) = chain(i).getMin();
        pos_lim(i,1) = chain(i).getMax();
    }
}

/*
if (second_arm)
{
    dim += second_arm->chainActiveDOF-NR_TORSO_JOINTS;
    Vector xr2(6,0.0);
    xr2.setSubvector(0, second_arm->x_n);
    xr2.setSubvector(3, second_arm->o_d);
#ifndef NEO_TEST
    solver->init(xr, main_arm->q_dot, main_arm->vLimAdapted, comingHome? 10:restPosWeight, xr2, second_arm->q_dot, second_arm->vLimAdapted);
#endif
}
else
{
#ifdef NEO_TEST
    solver->init(xr, main_arm->q_dot, main_arm->vLimAdapted, Alims, blims);
#else
    solver->init(xr, main_arm->q_dot, main_arm->vLimAdapted, comingHome? 10:restPosWeight);
#endif
}
*/

/*
#ifdef NEO_TEST
    solver = std::make_unique<NeoQP>(main_arm->virtualArm, hittingConstraints, vMax, dT);
#else
    solver = std::make_unique<QPSolver>(main_arm->virtualArm, hittingConstraints,
                                        second_arm? second_arm->virtualArm : nullptr,
                                        vMax, orientationControl,dT,
                                        main_arm->homePos*CTRL_DEG2RAD, restPosWeight);
#endif
 */
/*
#define NEO_TEST

#ifdef NEO_TEST
#include "NeoQP.h"
#else
#include "reactOSQP.h"
#endif
 */
/*
#ifdef NEO_TEST
std::unique_ptr<NeoQP> solver;
#else
std::unique_ptr<QPSolver> solver;
#endif
// Neo stuff
std::vector<Vector> Alims;
std::vector<double> blims;

 */

/*
double w_eps = 1;
double di = 0.3;
double ds = 0.05;
double d = 0.1;
double b = w_eps*(d-ds)/(di/ds) - colPoint.magnitude;
Alims.push_back(J.transposed()*normal);
blims.push_back(b);
 */
//public:
/****************************************************************/
NeoQP::NeoQP(iCubArm *chain_, bool hitConstr, double vmax_, double dT_,  const std::string& part_, yarp::sig::Vector* data) :
        arm(chain_), hitting_constraints(hitConstr), dt(dT_), vmax(vmax_*CTRL_DEG2RAD), part(part_)
{
    chain_dof = static_cast<int>(arm->getDOF());
    const int vars = chain_dof + 6;
    const int constr = chain_dof + 12 + 3 + hitting_constraints * 3 + chain_dof+obs_constr_num;
    w_n = 1;
    w_q = 0.01;
    w_d = 2;
    pr.resize(3, 0.0);
    v0.resize(chain_dof, 0.0); // v=v0;
    pos_lim.resize(chain_dof,2);
    computePosLims();
    hessian.resize(vars, vars);
    set_hessian();
    gradient.resize(vars);
    Jm.resize(chain_dof);
    gradient.setZero();
    lowerBound.resize(constr);
    lowerBound.setZero();
    upperBound.resize(constr);
    upperBound.setZero();
    Aobstacles.resize(obs_constr_num);
    bvalue.resize(obs_constr_num, std::numeric_limits<double>::max());
    linearMatrix.resize(constr, vars);
    for (int i = 0; i < chain_dof + 6; ++i) {
        linearMatrix.insert(i, i) = 1;
    }
    for (int i = 0; i < 6; ++i) {
        linearMatrix.insert(i + chain_dof + 6, chain_dof + i) = -1;
    }
    for (int i = 0; i < chain_dof; ++i) { // joint pos limits dumpers
        linearMatrix.insert(i + chain_dof + 12, i) = 1;
    }

    linearMatrix.insert(2*chain_dof + 12, 3) = 1.71 * dt;
    linearMatrix.insert(2*chain_dof + 12, 4) = -1.71 * dt;
    linearMatrix.insert(2*chain_dof + 12 + 1, 3) = 1.71 * dt;
    linearMatrix.insert(2*chain_dof + 12 + 1, 4) = -1.71 * dt;
    linearMatrix.insert(2*chain_dof + 12 + 1, 5) = -1.71 * dt;
    linearMatrix.insert(2*chain_dof + 12 + 2, 4) = dt;
    linearMatrix.insert(2*chain_dof + 12 + 2, 5) = dt;
    if (hitting_constraints) {
        linearMatrix.insert(2*chain_dof + 12 + 3, 4) = dt;
        linearMatrix.insert(2*chain_dof + 12 + 3, 5) = shou_m * dt;
        linearMatrix.insert(2*chain_dof + 12 + 4, 6) = -elb_m * dt;
        linearMatrix.insert(2*chain_dof + 12 + 4, 7) = dt;
        linearMatrix.insert(2*chain_dof + 12 + 5, 6) = elb_m * dt;
        linearMatrix.insert(2*chain_dof + 12 + 5, 7) = dt;
    }
    J0 = arm->GeoJacobian();
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < chain_dof; ++j)
        {
            linearMatrix.insert(i + chain_dof + 6, j) = J0(i, j);
        }
    }
    for (int i = 0; i < obs_constr_num; ++i) {
        for (int j = 0; j < chain_dof; ++j) {
            linearMatrix.insert(i + 2 * chain_dof + 12 + 3 + 3 * hitting_constraints, j) = 0.0;
        }
        lowerBound[i + 2 * chain_dof + 12 + 3 + 3 * hitting_constraints] = -std::numeric_limits<double>::max();
        Aobstacles[i].resize(chain_dof,0.0);
        upperBound[i + 2 * chain_dof + 12 + 3 + 3 * hitting_constraints] = std::numeric_limits<double>::max();

    }
    generateRobotColPoints(data);

    solver.data()->setNumberOfVariables(vars);
    solver.data()->setNumberOfConstraints(constr); // hiting_constraints*6
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);
    solver.settings()->setMaxIteration(30000);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);
    solver.settings()->setTimeLimit(0.9*dt);
    solver.settings()->setCheckTermination(10);
    solver.settings()->setPolish(true);
    solver.settings()->setRho(0.001);
    solver.settings()->setPrimalInfeasibilityTollerance(1e-4);
    solver.settings()->setDualInfeasibilityTollerance(1e-4);
    solver.settings()->setVerbosity(false);
    solver.initSolver();
    yDebug("solver prepared");
}
NeoQP::~NeoQP() = default;

/****************************************************************/
void NeoQP::init(const Vector &_xr, const Vector &_v0, const std::vector<Vector>& obstacles) //, const std::vector<Vector>& Alims, const std::vector<double>& blims) // TODO add obstacle dumpers
{
    yAssert(6 <= _xr.length());
    yAssert(v0.length() == _v0.length());
    Hessian.clear();
    v0= CTRL_DEG2RAD * _v0;
    q0=arm->getAng();
    H0=arm->getH();
    p0=H0.getCol(3).subVector(0,2);
    pr=_xr.subVector(0, 2);
    const Vector ang=_xr.subVector(3,6);
    const Matrix R = axis2dcm(ang).submatrix(0,2,0,2)*H0.submatrix(0,2,0,2).transposed();

    v_des.resize(6,0);
    v_des.setSubvector(0, (pr-p0) / dt);
    v_des.setSubvector(3, dcm2rpy(R) / dt);
    w_d = 0;
    for (int i = 0; i < 6; i++)
    {
        w_d += 10*abs(v_des[i])*dt;
    }

    J0=arm->GeoJacobian();
    for (int i = 0; i < chain_dof; i++) {
        Matrix mat = Matrix(6, chain_dof);
        for (int j = 0; j < chain_dof; ++j) {
            mat.setCol(j, arm->Hessian_ij(i, j));
        }
        Hessian.push_back(mat);
    }
    const double manip = sqrt(det(J0 * J0.transposed()));
    const Matrix b = luinv(J0*J0.transposed());
    Vector w(36,0.0);
    for (int j = 0; j < 6; j++)
    {
        w.setSubvector(j*6,b.getCol(j));
    }

    Jm.resize(chain_dof);
    for (int i = 0; i < chain_dof; ++i)
    {
        const Matrix c = J0 * Hessian[i].transposed();
        Vector v(36,0.0);
        for (int j = 0; j < 6; j++)
        {
            v.setSubvector(j*6,c.getCol(j));
        }
        Jm[i] = manip * dot(v,w);
    }
    for (int i = 0; i < obs_constr_num; i++)
    {
        Aobstacles[i].zero();
        bvalue[i] = std::numeric_limits<double>::max();
    }
    if (!obstacles.empty())
    {
        computeObstacles(obstacles);
        for (int j = 0; j < obs_constr_num; ++j) {
            upperBound[j + 2 * chain_dof + 12 + 3 + 3 * hitting_constraints] = bvalue[j];
        }
    }
    for (int i = 0; i < obs_constr_num; i++) {
        for (int j = 0; j < Aobstacles[i].size(); ++j) {
            linearMatrix.coeffRef(i + 2 * chain_dof + 12 + 3 + 3 * hitting_constraints, j) = Aobstacles[i][j];
        }
//        if (norm(Aobstacles[i]) > 1e-2) printf("dot %g and bval %g\n", dot(Aobstacles[i],_v0),bvalue[i]);
    }

    update_gradient();
    update_constraints();
    update_hessian();
}


/****************************************************************/
void NeoQP::update_bounds()
{
    for (int i = 0; i < chain_dof; i++)
    {
        lowerBound[i] = -vmax;
        upperBound[i] = vmax;
    }
    
    for (int i = 0; i < 3; ++i)
    {
        lowerBound[chain_dof+i] = -std::numeric_limits<double>::max(); // -std::numeric_limits<double>::max(); // normal(i) != 0? -10 : 0; //  normal(i)/dt/10 : 0; // normal(i)/dt/10;
        upperBound[chain_dof+i] = std::numeric_limits<double>::max(); // std::numeric_limits<double>::max(); //normal(i) != 0?  pos_error : 0; //  normal(i)/dt/10 : 0;
    }

    for (int i = 3; i < 6; ++i)
    {
        lowerBound[chain_dof+i] = -std::numeric_limits<double>::max();
        upperBound[chain_dof+i] =  std::numeric_limits<double>::max();
    }

    for (int i = 0; i < 6; ++i)
    {
        lowerBound[i+chain_dof+6] = v_des[i];
        upperBound[i+chain_dof+6] = v_des[i];
    }
    for (int i = 0; i < chain_dof; i++)
    {
        lowerBound[chain_dof+i+12] = -std::numeric_limits<double>::max();
        upperBound[chain_dof+i+12] = std::numeric_limits<double>::max();
        if (q0[i] - pos_lim(i, 0) <= 0.2)
        {
            lowerBound[chain_dof+i+12] = -w_n*(((q0[i] - pos_lim(i,0))  - 0.05) / (0.2 - 0.05));
        }
        else if (pos_lim(i, 1) - q0[i] <= 0.2)
        {
            upperBound[chain_dof+i+12] = w_n*(((pos_lim(i,1) - q0[i]) - 0.05) / (0.2 - 0.05));
        }
    }

    // shoulder's cables length
    lowerBound[6+2*chain_dof+6]=-347.00*CTRL_DEG2RAD-(1.71*(q0[3] - q0[4]));
    upperBound[6+2*chain_dof+6]=std::numeric_limits<double>::max();
    lowerBound[7+2*chain_dof+6]=-366.57*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    upperBound[7+2*chain_dof+6]=112.42*CTRL_DEG2RAD-(1.71*(q0[3]-q0[4]-q0[5]));
    lowerBound[8+2*chain_dof+6]=-66.60*CTRL_DEG2RAD-(q0[4]+q0[5]);
    upperBound[8+2*chain_dof+6]=213.30*CTRL_DEG2RAD-(q0[4]+q0[5]);
    if (hitting_constraints)
    {
        // avoid hitting torso
        lowerBound[9+2*chain_dof+6]=-shou_n - (q0[4] + shou_m*q0[5]);
        upperBound[9+2*chain_dof+6]=std::numeric_limits<double>::max();

        // avoid hitting forearm
        lowerBound[10+2*chain_dof+6]=-std::numeric_limits<double>::max();
        upperBound[10+2*chain_dof+6]=elb_n - (-elb_m*q0[6] + q0[7]);
        lowerBound[11+2*chain_dof+6]=-elb_n - (elb_m*q0[6] + q0[7]);
        upperBound[11+2*chain_dof+6]=std::numeric_limits<double>::max();
    }

    solver.updateBounds(lowerBound,upperBound);
}


/****************************************************************/
void NeoQP::update_gradient()
{
    gradient.setZero();

    for (int i=0; i < chain_dof; i++)
    {
        gradient[i] = 0.001 * -Jm[i];
    }
    solver.updateGradient(gradient);
}


/****************************************************************/
void NeoQP::set_hessian()
{
    for (int i = 0; i < chain_dof; ++i)
    {
        hessian.insert(i, i) = w_q;
    }

    for (int i = 0; i < 6; ++i)
    {
        hessian.insert(chain_dof+i,chain_dof+i) = w_d;
    }
}


void NeoQP::update_hessian()
{
    for (int i = 0; i < 6; ++i)
    {
        hessian.coeffRef(chain_dof+i,chain_dof+i) = w_d;
    }
    solver.updateHessianMatrix(hessian);
}


/****************************************************************/
void NeoQP::update_constraints()
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < chain_dof; ++j)
        {
            linearMatrix.coeffRef(i + chain_dof + 6, j) = J0(i, j);
        }
    }
    
    solver.updateLinearConstraintsMatrix(linearMatrix);
}

Vector NeoQP::get_resultInDegPerSecond()
{
    Eigen::VectorXd sol = solver.getSolution();
//    yDebug("Objective value is = %g", solver.workspace()->info->obj_val);
    Vector v(chain_dof,0.0);
    for (int i = 0; i < chain_dof; ++i)
    {
        v[i] = std::max(std::min(sol[i], upperBound[i]), lowerBound[i]);
    }
    return CTRL_RAD2DEG*v;
}

int NeoQP::optimize()
{
    update_bounds();
    Eigen::VectorXd primalVar(hessian.rows());
    primalVar.setZero();
    for (int i = 0; i < chain_dof; i++)
    {
        primalVar[i] = std::min(std::max(-vmax, v0[i]), vmax);
    }
    
    solver.setPrimalVariable(primalVar);
    solver.solve();
//    Eigen::VectorXd constr = linearMatrix * solver.getSolution();
//    for (int i = 0; i < chain_dof + 12 + 3 + hitting_constraints * 3 + chain_dof+10; i++)
//    {
//        printf("%g < %g < %g\n", lowerBound[i], constr[i], upperBound[i]);
//    }
//    printf("\n");
    std::ofstream f("test.txt");
    if (f.is_open())
    {
        f << "Lin matrix:\n" << linearMatrix << '\n';
        f << "Lowerbound:\n" <<  lowerBound << '\n';
        f << "Upperbound:\n" <<  upperBound << '\n';
        f << "Solution\n" << solver.getSolution() << "\n";
    }
    return static_cast<int>(solver.workspace()->info->status_val);
}
