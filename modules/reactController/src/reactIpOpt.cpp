/* 
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone <alessandro.roncone@iit.it>
 * website: www.robotcub.org
 * author website: http://alecive.github.io
 * 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
*/

#include <limits>
#include <sstream>
#include <cmath>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include "assert.h"

#include "reactIpOpt.h"

#define CAST_IPOPTAPP(x)             (static_cast<IpoptApplication*>(x))

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::skinDynLib;
using namespace Ipopt;


/*************optimization problem representation********************************************************/
class react_NLP : public TNLP
{
private:
    // Copy constructor: not implemented.
    react_NLP(const react_NLP&);
    // Assignment operator: not implemented.
    react_NLP &operator=(const react_NLP&);

protected:
    // The name of the class instance (fixed for now to react_NLP)
    string name;
    // The verbosity level (fixed for now to 0)
    int verbosity;
    // The chain that will undergo the task
    iKinChain &chain;
    // The dimensionality of the task ~ nr DOF in the chain ~ nr. primal variables (for now 7 for arm joints, 10 if torso is enabled)
    unsigned int dim;

    // The desired position to attain
    yarp::sig::Vector &xd;
    // The current position
    yarp::sig::Vector x0;

    // The delta T with which ipopt needs to solve the task - time in the task, not comp. time - delta x = delta T * J * q_dot 
    double &dT;

    // The maximum allowed speed at the joints
    double V_max;
    
    // The desired final joint velocities - the solution of this iteration 
    yarp::sig::Vector q_dot_d;
    // The initial joint velocities - solution from prev. step 
    yarp::sig::Vector q_dot_0;
    // The current joint velocities - for internal steps of ipopt, not carrying a particular meaning
    yarp::sig::Vector q_dot;

    // The current joint configuration
    yarp::sig::Vector q_t;

    // The cost function
    yarp::sig::Vector cost_func;
    // The gradient of the cost function
    yarp::sig::Vector grad_cost_func;
    // The jacobian associated with the cost function
    yarp::sig::Matrix J_cst;

    // Weights set to the joint limits
    yarp::sig::Vector W;
    // Derivative of said weights w.r.t q
    yarp::sig::Vector W_dot;
    // The minimum weight given to the joint limits bound function (1.0)
    double W_min;   
    // The gamma of the weight given to the joint limits bound function (W_min+W_gamma=W_max, default 3.0)
    double W_gamma; 
    // The guard ratio 
    double guardRatio;

    // The torso reduction rate at for the max velocities sent to the torso(default 3.0)
    double torsoReduction;  
    //for smooth changes in joint velocities
    double boundSmoothness;
    
    vector<collisionPoint_t> collisionPoints; //list of "avoidance vectors" from peripersonal space / safety margin
    /* every (potential) collision point will be converted into a set of joint velocity limits in the subchain   above the threat - 
     - the unaffected joints (more distal and those with opposite sign of Jacobian inverse) will be set to their maxima/minima */
    vector<yarp::sig::Vector> jointVelMinimaForSafetyMargin;
    vector<yarp::sig::Vector> jointVelMaximaForSafetyMargin;
    //joint position limits handled in a smooth way - see Pattacini et al. 2010 - iCub Cart. control - Fig. 8
    yarp::sig::Vector qGuard;
    yarp::sig::Vector qGuardMinInt, qGuardMinExt, qGuardMinCOG;
    yarp::sig::Vector qGuardMaxInt, qGuardMaxExt, qGuardMaxCOG;

    bool firstGo;

    /************************************************************************/
    virtual void computeQuantities(const Number *x)
    {
        printMessage(9,"[computeQuantities] START dim: %i \n", dim);
        yarp::sig::Vector new_q_dot(dim,0.0);

        for (Index i=0; i<(int)dim; i++)
        {
            new_q_dot[i]=x[i];
        }
        if (!(q_dot==new_q_dot) || firstGo)
        {
            firstGo=false;
            q_dot=new_q_dot;
            yarp::sig::Matrix J1=chain.GeoJacobian();
            submatrix(J1,J_cst,0,2,0,dim-1);

            q_t = chain.getAng();
            cost_func=xd -(x0+dT*J_cst*q_dot);
            grad_cost_func=2.0*cost_func*(-dT*J_cst);
        }
        printMessage(9,"[computeQuantities] OK x: %s\n",IPOPT_Number_toString(x,CTRL_RAD2DEG).c_str());
    }

    /************************************************************************/
    string IPOPT_Number_toString(const Number* x, const double multiplier=1.0)
    {
        std::ostringstream ss;
        for (Index i=0; i<(Index)dim; i++)
        {
            ss << x[i]*multiplier << " ";
        }
        return ss.str();
    }

    /************************************************************************/
    bool computeWeight(const yarp::sig::Vector &q)
    {
        for (unsigned int i=0; i<dim; i++)
        {
            bool print=false;
            if ((q[i]>=qGuardMinInt[i]) && (q[i]<=qGuardMaxInt[i]))
                W(i)=W_min;
            else if ((q[i]<=qGuardMinExt[i]) || (q[i]>=qGuardMaxExt[i]))
                W(i)=W_min+W_gamma;
            else if (q[i]<qGuardMinInt[i])
            {
                W(i)=0.5*W_gamma*(1.0+tanh(-6.0*(q[i]-qGuardMinCOG[i])/qGuard[i]))+W_min;
            }
            else
            {
                W(i)=0.5*W_gamma*(1.0+tanh( 6.0*(q[i]-qGuardMaxCOG[i])/qGuard[i]))+W_min;
            }

            if (print)
            {
                printf("weight vector: %s\n", W.toString(3,3).c_str());
            }
        }
        
        return true;
    }

    /************************************************************************/
    bool computeWeightDerivatives(const yarp::sig::Vector &q)
    {
        for (unsigned int i=0; i<dim; i++)
        {
            bool print=false;
            if ((q[i]>=qGuardMinInt[i]) && (q[i]<=qGuardMaxInt[i]))
                W_dot(i)=0.0;
            else if ((q[i]<=qGuardMinExt[i]) || (q[i]>=qGuardMaxExt[i]))
                W_dot(i)=0.0;
            else if (q[i]<qGuardMinInt[i])
            {
                W_dot(i)= (3*W_gamma*(pow(tanh((6*q[i] - 6*qGuardMinCOG[i])/qGuard[i]),2) - 1))/qGuard[i];
                // W_dot(i)=0.5*W_gamma*(1.0+tanh(-6.0*(q[i]-qGuardMinCOG[i])/qGuard[i]))+W_min;
            }
            else
            {
                W_dot(i)=-(3*W_gamma*(pow(tanh((6*q[i] - 6*qGuardMaxCOG[i])/qGuard[i]),2) - 1))/qGuard[i];
                // W_dot(i)=0.5*W_gamma*(1.0+tanh( 6.0*(q[i]-qGuardMaxCOG[i])/qGuard[i]))+W_min;
            }

            if (print)
            {
                printf("weight vector: %s\n", W_dot.toString(3,3).c_str());
            }
        }
        
        return true;
    }
    
    
    //creates a full transform as given by a DCM matrix at the pos and norm w.r.t. the original frame, from the pos and norm (one axis set arbitrarily)  
    bool computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR)
    {
        if (norm == zeros(3))
        {
            FoR=eye(4);
            return false;
        }
        
        // Set the proper orientation for the touching end-effector
        yarp::sig::Vector x(3,0.0), z(3,0.0), y(3,0.0);

        z = norm;
        if (z[0] == 0.0)
        {
            z[0] = 0.00000001;    // Avoid the division by 0
        }
        y[0] = -z[2]/z[0]; //y is in normal plane
        y[2] = 1; //this setting is arbitrary
        x = -1*(cross(z,y));

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
    

    /********to ensure joint pos limits but in a smooth way around the limits****************************************/
    void computeGuard()
    {
        for (unsigned int i=0; i<dim; i++)
        {
            qGuard[i]=0.25*guardRatio*(chain(i).getMax()-chain(i).getMin());

            qGuardMinExt[i]=chain(i).getMin()+qGuard[i];
            qGuardMinInt[i]=qGuardMinExt[i]  +qGuard[i];
            qGuardMinCOG[i]=0.5*(qGuardMinExt[i]+qGuardMinInt[i]);

            qGuardMaxExt[i]=chain(i).getMax()-qGuard[i];
            qGuardMaxInt[i]=qGuardMaxExt[i]  -qGuard[i];
            qGuardMaxCOG[i]=0.5*(qGuardMaxExt[i]+qGuardMaxInt[i]);
        }

        printMessage(4,"qGuard       %s\n",(CTRL_RAD2DEG*qGuard).toString(3,3).c_str());
        printMessage(4,"qGuardMinExt %s\n",(CTRL_RAD2DEG*qGuardMinExt).toString(3,3).c_str());
        printMessage(4,"qGuardMinCOG %s\n",(CTRL_RAD2DEG*qGuardMinCOG).toString(3,3).c_str());
        printMessage(4,"qGuardMinInt %s\n",(CTRL_RAD2DEG*qGuardMinInt).toString(3,3).c_str());
        printMessage(4,"qGuardMaxInt %s\n",(CTRL_RAD2DEG*qGuardMaxInt).toString(3,3).c_str());
        printMessage(4,"qGuardMaxCOG %s\n",(CTRL_RAD2DEG*qGuardMaxCOG).toString(3,3).c_str());
        printMessage(4,"qGuardMaxExt %s\n",(CTRL_RAD2DEG*qGuardMaxExt).toString(3,3).c_str());
    }
    
    /**** goes through the avoidanceVectors and for each of them adapts the joint vel limits in
     * the chain above the threat so as to assist in avoiding the collision - constraining the joints in going toward the threat */
    void setSafetyMarginJointVelLimits()
    {
        if (!collisionPoints.empty()){
            for(std::vector<collisionPoint_t>::const_iterator it = collisionPoints.begin(); it != collisionPoints.end(); ++it) {
                yarp::sig::Vector minima(dim,-V_max); //joint velocity minima
                yarp::sig::Vector maxima(dim,V_max); // joint velocity maxima
                iKinChain chain_local = chain; //makes a copy that will be used internally and some joints blocked
               
                if (verbosity >= 5){
                    printf("Full chain has %d DOF \n",chain.getDOF());
                    printf("chain.getH(): \n %s \n",chain.getH().toString(3,3).c_str());
                    int linkNrForCurrentSkinPartFrame = 0;
                    if (dim ==7){
                        printf("chain.getH() relevant for skinPart %s: \n %s \n",SkinPart_s[(*it).skin_part].c_str(),chain.getH(SkinPart_2_LinkNum[(*it).skin_part].linkNum).toString(3,3).c_str());
                    }
                    else if (dim == 10){
                         printf("chain.getH() relevant for skinPart %s: \n %s \n",SkinPart_s[(*it).skin_part].c_str(),chain.getH(SkinPart_2_LinkNum[(*it).skin_part].linkNum + 3).toString(3,3).c_str());
                    }
                    yarp::sig::Matrix JfullChain = chain.GeoJacobian(); //6 rows, n columns for every active DOF
                    printMessage(5,"GeoJacobian matrix for canonical end-effector (palm): \n %s \n",JfullChain.toString(3,3).c_str());
                }
                 
               // Block all the more distal joints after the joint 
                // if the skin part is a hand, no need to block any joints
                if (((*it).skin_part == SKIN_LEFT_FOREARM) ||  ((*it).skin_part == SKIN_RIGHT_FOREARM)){
                    if(dim == 10){
                        chain_local.blockLink(9); chain_local.blockLink(8);//wrist joints 
                        printMessage(4,"obstacle threatening skin part %s, blocking links 9 and 10 (wrist) (but indexes 8,9) on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                    }
                    else if(dim==7){
                        chain_local.blockLink(6); chain_local.blockLink(5);//wrist joints
                        printMessage(4,"obstacle threatening skin part %s, blocking links 6 and 7 (wrist) (but indexes 5,6) on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                   
                    }
                }
                else if (((*it).skin_part == SKIN_LEFT_UPPER_ARM) ||  ((*it).skin_part == SKIN_RIGHT_UPPER_ARM)){
                    if(dim == 10){
                        chain_local.blockLink(9); chain_local.blockLink(8);chain_local.blockLink(7);chain_local.blockLink(6); //wrist joints + elbow joints
                        printMessage(4,"obstacle threatening skin part %s, blocking links 7-10 (wrist+elbow; indexes 6-9) on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                    }
                    else if(dim==7){
                        chain_local.blockLink(6); chain_local.blockLink(5);chain_local.blockLink(4);chain_local.blockLink(3); //wrist joints + elbow joints
                        printMessage(4,"obstacle threatening skin part %s, blocking links 7-10 (wrist+elbow; indexes 6-9) on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                    }
                }
                // SetHN to move the end effector toward the point to be controlled - the average locus of collision threat from safety margin
                yarp::sig::Matrix HN = eye(4);
                computeFoR((*it).x,(*it).n,HN);
                printMessage(5,"HN matrix at collision point w.r.t. local frame: \n %s \n",HN.toString(3,3).c_str());
                //TODO Ugo - the end-effector needs to be correctly set w.r.t. the sub chain? But coordinates are in the corresponding frame for a given skin part, SkinPart_2_LinkNum[(*it).skin_part].linkNum) [+ 3 torso DOF]  
                chain_local.setHN(HN); //! this is probably wrong - setting the end-effector transform to the collision point w.r.t subchain
                if (verbosity >=5){
                    yarp::sig::Matrix H = chain_local.getH();
                    printf("H matrix at collision point w.r.t. root: \n %s \n",H.toString(3,3).c_str());
                }
                
                //printMessage(5,"Normal at collision point w.r.t. skin part frame: %s, norm %f.\n",(*it).n.toString(3,3).c_str(),yarp::math::norm((*it).n));
                //yarp::sig::Vector normalAtCollisionInRootFoR = chain_local.getH() * (*it).n;
                //normalAtCollisionInRootFoR.subVector(0,3); //take out the dummy element from homogenous transform
                //printMessage(5,"Normal at collision point w.r.t. Root: %s, norm %f.\n",normalAtCollisionInRootFoR.toString(3,3).c_str(),yarp::math::norm(normalAtCollisionInRootFoR));
        
                //for testing
                //yarp::sig::Vector normalAtCollisionInEndEffFrame(4,0.0);
                //normalAtCollisionInEndEffFrame(2) = 1.0; //z-axis ~ normal
                //yarp::sig::Vector normalAtCollisionInRootFoR_2 = chain_local.getHN() * normalAtCollisionInEndEffFrame;
                
                
                yarp::sig::Matrix J = chain_local.GeoJacobian(); //6 rows, n columns for every active DOF (excluding the blocked)
                printMessage(5,"GeoJacobian matrix of new end-effector (collision point), after possible blocking of joints: \n %s \n",J.toString(3,3).c_str());
                yarp::sig::Matrix pseudoInvJ = yarp::math::pinv(J); 
                printMessage(5,"Jacobian pseudoinverse matrix: \n %s \n",pseudoInvJ.toString(3,3).c_str());
                

                // Compute the q_dot - joint velocities that contribute to motion along the normal - "toward the "threat" 
                //yarp::sig::Vector qdotTowardThreat = correct submatrix of  pseudoInvJ * normal at collision point; //TODO Ugo
                // ~ Eq. 13 in Flacco et al. 2012
                
                //TODO Ugo take the q_dot and then use them to define the minima and maxima vectors - take into account just the sign like Flacco (Eq. 14) or 
                //magnitude of qdot or take into account avoidance vector magnitude (Eq. 12 in Flacco, in our case will come from PPS activation; 
                // be careful with which links are blocked - assign to correct joints
                
                //TODO Ugo add the newly found limits to jointVelMinimaForSafetyMargin / jointVelMaximaForSafetyMargin;
                
               // printf("setSafetyMarginJointVelLimits: end of \n");
            }
        }
    }
    
    
    /************************************************************************/
    int printMessage(const int l, const char *f, ...) const
    {
        if (verbosity>=l)
        {
            fprintf(stdout,"[%s] ",name.c_str());

            va_list ap;
            va_start(ap,f);
            int ret=vfprintf(stdout,f,ap);
            va_end(ap);
            return ret;
        }
        else
            return -1;
    }


public:
    /***** 8 pure virtual functions from TNLP class need to be implemented here **********/
    /************************************************************************/
    react_NLP(iKinChain &c, yarp::sig::Vector &_xd, yarp::sig::Vector &_q_dot_0,
             double &_dT, double &_vM, const std::vector<collisionPoint_t> & _collision_points, int _verbosity) : chain(c),
             q_dot_0(_q_dot_0), dT(_dT), xd(_xd), collisionPoints(_collision_points), verbosity(_verbosity), V_max(_vM)
    {
        name="react_NLP";

        // A time should always be positive
        if (dT<0.0)
        {
            dT=0.05;
        }

        x0.resize(3,0.0);
        yarp::sig::Matrix H=chain.getH();
        x0=H.subcol(0,3,3);

        dim=chain.getDOF(); //e.g. 7 for the arm joints, 10 if torso is included
        if (!((dim == 7) || (dim == 10)) ){ 
            yError("react_NLP(): unexpected nr DOF on the chain : %d\n",dim);
        }
        q_dot.resize(dim,0.0);
        q_dot_d.resize(dim,0.0);

        cost_func.resize(3,0.0); //cost function is defined in cartesian space - position of end-effector
        J_cst.resize(3,dim); J_cst.zero(); // The jacobian associated with the cost function

        W.resize(dim,0.0);
        W_dot.resize(dim,0.0);
        W_min=1.0;
        W_gamma=3.0;
        guardRatio=0.4;

        torsoReduction=3.0;
        boundSmoothness=10.0;

        qGuard.resize(dim,0.0);
        qGuardMinInt.resize(dim,0.0);
        qGuardMinExt.resize(dim,0.0);
        qGuardMaxInt.resize(dim,0.0);
        qGuardMaxExt.resize(dim,0.0);
        qGuardMinCOG.resize(dim,0.0);
        qGuardMaxCOG.resize(dim,0.0);

        computeGuard();

        firstGo=true;
    }

    /************************************************************************/
    yarp::sig::Vector get_q_dot_d() { return q_dot_d; }

    /************************************************************************/
    bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag,
                      IndexStyleEnum& index_style)
    {
        n=dim; // number of vars in the problem (dim(x)) ~ n DOF in chain in our case 
        m=dim+0; // nr constraints - dim(g(x))
        nnz_jac_g=dim; // the jacobian of g has dim non zero entries (the diagonal)

        // nnz_h_lag=(dim*(dim+1))>>1;
        nnz_h_lag=0; //number of nonzero entries in the Hessian
        index_style=TNLP::C_STYLE;
        printMessage(7,"[get_nlp_info]\tn: %i m: %i nnz_jac_g: %i\n",n,m,nnz_jac_g);
        
        return true;
    }
    
    /************************************************************************/
    bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                            Number* z_L, Number* z_U, Index m, bool init_lambda,
                            Number* lambda)
    {
        assert(init_x == true); //initial value for x
        assert(init_z == false); // false => no initial value for bound multipliers z_L, z_U
        assert(init_lambda == false); // false => no initial value for constraint multipliers

        for (Index i=0; i<n; i++){
            x[i]=q_dot_0[i]; //initializing the primal variables 
            //inside IPOPT, the variables optimized ("primal variables") are x (in our case they are q_dot - 
            // watch out that you don't confuse them with x - Cartesian pos)
        }
        printMessage(3,"[get_starting_pnt] x_0: %s\n",IPOPT_Number_toString(x,CTRL_RAD2DEG).c_str());

        return true;
    }

    /************************************************************************/
    bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l,
                         Number* g_u)
    {
        //x_l, x_u - limits for the primal variables - in our case joint velocities
        setSafetyMarginJointVelLimits(); //will go through the avoidanceVectors and set the jointVelMinimaForSafetyMargin / jointVelMaximaForSafetyMargin

        for (Index i=0; i<n; i++)
        {
            // The joints velocities will be constrained by the V_min / V_max constraints and 
            // and the previous state (that is our current initial state), in order to avoid abrupt changes
            x_l[i]=max(-V_max*CTRL_DEG2RAD,q_dot_0[i]-boundSmoothness*CTRL_DEG2RAD); //lower bound
            x_u[i]=min(+V_max*CTRL_DEG2RAD,q_dot_0[i]+boundSmoothness*CTRL_DEG2RAD); //upper bound
            
            //TODO Ugo integrate the additional limits from  jointVelMinimaForSafetyMargin / jointVelMaximaForSafetyMargin;
                
            
            if (n==10 && i<3) //special handling of torso joints - should be moving less
            {
                x_l[i]=max(-V_max*CTRL_DEG2RAD/torsoReduction,q_dot_0[i]-boundSmoothness*CTRL_DEG2RAD);
                x_u[i]=min(+V_max*CTRL_DEG2RAD/torsoReduction,q_dot_0[i]+boundSmoothness*CTRL_DEG2RAD);
            }

            // printf("-V_max*CTRL_DEG2RAD %g\tq_dot_0[i]-boundSmoothness*CTRL_DEG2RAD %g\n",
            //         -V_max*CTRL_DEG2RAD,    q_dot_0[i]-boundSmoothness*CTRL_DEG2RAD);
        }
        
        //limits for the g function - in our case joint position limits
        for (Index i=0; i<m; i++)
        {
            if (i<(Index)dim)
            {
                g_l[i]=chain(i).getMin(); //returns joint angle lower bound
                g_u[i]=chain(i).getMax();  //returns joint angle upper bound
            }
        }
        
        printMessage(3,"[get_bounds_info]   x_l: %s\n", IPOPT_Number_toString(x_l,CTRL_RAD2DEG).c_str());
        printMessage(3,"[get_bounds_info]   x_u: %s\n", IPOPT_Number_toString(x_u,CTRL_RAD2DEG).c_str());
        printMessage(4,"[get_bounds_info]   g_l: %s\n", IPOPT_Number_toString(g_l,CTRL_RAD2DEG).c_str());
        printMessage(4,"[get_bounds_info]   g_u: %s\n", IPOPT_Number_toString(g_u,CTRL_RAD2DEG).c_str());
        return true;
    }
    
    /*******Return the value of the objective function at the point x **************************************************/
    bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
    {
        computeQuantities(x); //computes the cost function (global variable)

        obj_value=norm2(cost_func);
        printMessage(7,"[eval_f] OK\t\tcost_func: %s\tobj_value %g\n",cost_func.toString().c_str(),obj_value);

        return true;
    }
    
    /************** Return the gradient of the objective function at the point x*********************************************************/
    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
    {
        computeQuantities(x);

        for (Index i=0; i<n; i++)
            grad_f[i]=grad_cost_func[i];
            
        printMessage(7,"[eval_grad_f] OK\n");
        return true;
    }
    
    /********** Return the value of the constraint function at the point x***********************************************************/
    bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
    {
        printMessage(9,"[eval_g]\t\tq(t): %s\n",(q_t*CTRL_RAD2DEG).toString(3,3).c_str());
        computeQuantities(x);
        yarp::sig::Vector q(dim,0.0);

        for (Index i=0; i<(Index)dim; i++)
        {
            q[i]=q_t(i) + dT * q_dot(i);
        }
        computeWeight(q);

        for (Index i=0; i<m; i++)
            if (i<(Index)dim)
                g[i]=W[i]*q[i];

        printMessage(7,"[eval_g] OK\t\tq(t+1): %s\n",IPOPT_Number_toString(g,CTRL_RAD2DEG).c_str());

        return true;
    }
    
    /************************************************************************/
    bool eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac,
                    Index* iRow, Index *jCol, Number* values)
    {
        // printMessage(9,"[eval_jac_g] START\tx: %i\n",IPOPT_Number_toString(x,CTRL_RAD2DEG).c_str());

        if (m>=n) // if there are at least the joint bounds as constraint
        {
            if (values==NULL)
            {
                Index idx=0;
                
                // Let's populate the diagonal matrix with dT
                for (Index i=0; i<m; i++)
                {
                    iRow[idx]=i;
                    jCol[idx]=i;
                    idx++;
                }
            }
            else
            {
                computeQuantities(x);

                yarp::sig::Vector q(dim,0.0);

                for (Index i=0; i<(Index)dim; i++)
                {
                    q[i]=q_t(i) + dT * q_dot(i);
                }
                computeWeight(q);
                computeWeightDerivatives(q);

                Index idx=0;
                
                // Let's populate the diagonal matrix with dT
                for (Index i=0; i<m; i++)
                {
                    values[idx]=dT*(W[i]+q[i]*W_dot[i]);
                    idx++;
                }
            }
        }

        printMessage(7,"[eval_jac_g] OK\n");
        return true;
    }
    
    /************************************************************************/
    // bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
    //             Index m, const Number* lambda, bool new_lambda,
    //             Index nele_hess, Index* iRow, Index* jCol, Number* values)
    // {
    //     // Empty for now
    //     computeQuantities(x);

    //     return true;
    // }

    /************************************************************************/
    void finalize_solution(SolverReturn status, Index n, const Number* x,
                           const Number* z_L, const Number* z_U, Index m,
                           const Number* g, const Number* lambda, Number obj_value,
                           const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
    {
        // Let's write the solution to the console
        printMessage(4,"[finalize_solution] Solution of the primal variables, x: %s\n",
                                        IPOPT_Number_toString(x,CTRL_RAD2DEG).c_str());
        printMessage(4,"[finalize_solution] Solution of the bound multipliers: z_L and z_U\n");
        printMessage(4,"[finalize_solution] z_L: %s\n",
                        IPOPT_Number_toString(z_L,CTRL_RAD2DEG).c_str());
        printMessage(4,"[finalize_solution] z_U: %s\n",
                        IPOPT_Number_toString(z_U,CTRL_RAD2DEG).c_str());
        printMessage(4,"[finalize_solution] q(t+1): %s\n",
                        IPOPT_Number_toString(g,CTRL_RAD2DEG).c_str());
        printMessage(4,"[finalize_solution] Objective value f(x*) = %e\n", obj_value);
        for (Index i=0; i<n; i++)
            q_dot_d[i]=x[i];
    }

    /************************************************************************/
    virtual ~react_NLP() { }  
};


/************************************************************************/
reactIpOpt::reactIpOpt(iKinChain &c, const double tol,
                       const unsigned int verbose) :
                       chain(c), verbosity(verbose)
{
    chain.setAllConstraints(false); // this is required since IpOpt initially relaxes constraints

    App=new IpoptApplication();

    CAST_IPOPTAPP(App)->Options()->SetNumericValue("tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("acceptable_tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("acceptable_iter",10);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("mu_strategy","adaptive");
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("print_level",verbose);

    // CAST_IPOPTAPP(App)->Options()->SetStringValue("jacobian_approximation","finite-difference-values");
    CAST_IPOPTAPP(App)->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test","none");
    // CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test","first-order");
    CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test_print_all","yes");
    // CAST_IPOPTAPP(App)->Options()->SetStringValue("print_timing_statistics","yes");
    // CAST_IPOPTAPP(App)->Options()->SetStringValue("print_options_documentation","no");
    // CAST_IPOPTAPP(App)->Options()->SetStringValue("skip_finalize_solution_call","yes");

    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
    CAST_IPOPTAPP(App)->Options()->SetStringValue("hessian_approximation","limited-memory");

    Ipopt::ApplicationReturnStatus status = CAST_IPOPTAPP(App)->Initialize();
    if (status != Ipopt::Solve_Succeeded)
        yError("Error during initialization!");
}


/************************************************************************/
void reactIpOpt::setTol(const double tol)
{
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("acceptable_tol",tol);
    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
double reactIpOpt::getTol() const
{
    double tol;
    CAST_IPOPTAPP(App)->Options()->GetNumericValue("tol",tol,"");
    return tol;
}


/************************************************************************/
void reactIpOpt::setVerbosity(const unsigned int verbose)
{
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("print_level",verbose);
    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
yarp::sig::Vector reactIpOpt::solve(yarp::sig::Vector &xd, yarp::sig::Vector q_dot_0,
                                    double &dt, double &vm, const std::vector<collisionPoint_t> &collision_points,
                                    int *exit_code)
{
    SmartPtr<react_NLP> nlp=new react_NLP(chain,xd,q_dot_0,dt,vm,collision_points,verbosity);
    
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("max_cpu_time",dt);
    ApplicationReturnStatus status=CAST_IPOPTAPP(App)->OptimizeTNLP(GetRawPtr(nlp));

    if (exit_code!=NULL)
        *exit_code=status;

    return nlp->get_q_dot_d();
}

/************************************************************************/
reactIpOpt::~reactIpOpt()
{
    delete CAST_IPOPTAPP(App);
}


