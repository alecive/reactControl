/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
 * Public License for more details
*/

#include <cmath>
#include <limits>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <deque>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iKin/iKinFwd.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/****************************************************************/
class ControllerNLP : public Ipopt::TNLP
{
    iKinChain &chain;
    bool hitting_constraints;
    bool orientation_control;

    Vector xr,pr;
    Matrix Hr,skew_nr,skew_sr,skew_ar;
    Matrix q_lim,v_lim;    
    Vector q0,v0,v,p0;
    Matrix H0,R0,He,J0_xyz,J0_ang,Derr_ang;
    Vector err_xyz,err_ang;
    Matrix bounds;
    double dt;

    double shou_m,shou_n;
    double elb_m,elb_n;

    Vector qGuard;
    Vector qGuardMinExt;
    Vector qGuardMinInt;
    Vector qGuardMinCOG;
    Vector qGuardMaxExt;
    Vector qGuardMaxInt;
    Vector qGuardMaxCOG;

    /****************************************************************/
    void computeSelfAvoidanceConstraints()
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
    void computeGuard()
    {
        double guardRatio=0.1;
        qGuard.resize(chain.getDOF());
        qGuardMinExt.resize(chain.getDOF());
        qGuardMinInt.resize(chain.getDOF());
        qGuardMinCOG.resize(chain.getDOF());
        qGuardMaxExt.resize(chain.getDOF());
        qGuardMaxInt.resize(chain.getDOF());
        qGuardMaxCOG.resize(chain.getDOF());

        for (size_t i=0; i<chain.getDOF(); i++)
        {
            qGuard[i]=0.25*guardRatio*(chain(i).getMax()-chain(i).getMin());

            qGuardMinExt[i]=chain(i).getMin()+qGuard[i];
            qGuardMinInt[i]=qGuardMinExt[i]+qGuard[i];
            qGuardMinCOG[i]=0.5*(qGuardMinExt[i]+qGuardMinInt[i]);

            qGuardMaxExt[i]=chain(i).getMax()-qGuard[i];
            qGuardMaxInt[i]=qGuardMaxExt[i]-qGuard[i];
            qGuardMaxCOG[i]=0.5*(qGuardMaxExt[i]+qGuardMaxInt[i]);
        }
    }

    /****************************************************************/
    void computeBounds()
    {
        for (size_t i=0; i<chain.getDOF(); i++)
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

        for (size_t i=0; i<chain.getDOF(); i++)
        {
            bounds(i,0)*=v_lim(i,0);
            bounds(i,1)*=v_lim(i,1);
        }
    }

    /****************************************************************/
    Matrix v2m(const Vector &x)
    {
        yAssert(x.length()>=6);
        Vector ang=x.subVector(3,5);
        double ang_mag=norm(ang);
        if (ang_mag>0.0)
            ang/=ang_mag;
        ang.push_back(ang_mag);
        Matrix H=axis2dcm(ang);
        H(0,3)=x[0];
        H(1,3)=x[1];
        H(2,3)=x[2];
        return H;
    }

    /****************************************************************/
    Matrix skew(const Vector &w)
    {
        yAssert(w.length()>=3);
        Matrix S(3,3);
        S(0,0)=S(1,1)=S(2,2)=0.0;
        S(1,0)= w[2]; S(0,1)=-S(1,0);
        S(2,0)=-w[1]; S(0,2)=-S(2,0);
        S(2,1)= w[0]; S(1,2)=-S(2,1);
        return S;
    }

public:
    /****************************************************************/
    ControllerNLP(iKinChain &chain_) : chain(chain_)
    {
        xr.resize(6,0.0);
        set_xr(xr);

        v0.resize(chain.getDOF(),0.0); v=v0;
        He=zeros(4,4); He(3,3)=1.0;

        q_lim.resize(chain.getDOF(),2);
        v_lim.resize(chain.getDOF(),2);        
        for (size_t r=0; r<chain.getDOF(); r++)
        {
            q_lim(r,0)=chain(r).getMin();
            q_lim(r,1)=chain(r).getMax();

            v_lim(r,1)=std::numeric_limits<double>::max();
            v_lim(r,0)=-v_lim(r,1);
        }
        bounds=v_lim;
        
        computeSelfAvoidanceConstraints();
        computeGuard();

        hitting_constraints=true;
        orientation_control=true;
        dt=0.01;
    }

    /****************************************************************/
    void set_xr(const Vector &xr)
    {
        yAssert(this->xr.length()==xr.length());
        this->xr=xr;

        Hr=v2m(xr);
        pr=xr.subVector(0,2);

        skew_nr=skew(Hr.getCol(0));
        skew_sr=skew(Hr.getCol(1));
        skew_ar=skew(Hr.getCol(2));
    }

    /****************************************************************/
    void set_v_lim(const Matrix &v_lim)
    {
        yAssert((this->v_lim.rows()==v_lim.rows()) &&
                (this->v_lim.cols()==v_lim.cols()));

        for (int r=0; r<v_lim.rows(); r++)
            yAssert(v_lim(r,0)<=v_lim(r,1));

        this->v_lim=CTRL_DEG2RAD*v_lim;
    }

    /****************************************************************/
    void set_hitting_constraints(const bool hitting_constraints)
    {
        this->hitting_constraints=hitting_constraints;
    }

    /****************************************************************/
    void set_orientation_control(const bool orientation_control)
    {
        this->orientation_control=orientation_control;
    }

    /****************************************************************/
    void set_dt(const double dt)
    {
        yAssert(dt>0.0);
        this->dt=dt;
    }

    /****************************************************************/
    void set_v0(const Vector &v0)
    {
        yAssert(this->v0.length()==v0.length());
        this->v0=CTRL_DEG2RAD*v0;
    }

    /****************************************************************/
    void init()
    {
        q0=chain.getAng();
        H0=chain.getH();
        R0=H0.submatrix(0,2,0,2);
        p0=H0.getCol(3).subVector(0,2);

        Matrix J0=chain.GeoJacobian();
        J0_xyz=J0.submatrix(0,2,0,chain.getDOF()-1);
        J0_ang=J0.submatrix(3,5,0,chain.getDOF()-1);

        computeBounds();
    }

    /****************************************************************/
    Vector get_result() const
    {
        return CTRL_RAD2DEG*v;
    }

    /****************************************************************/
    Property getParameters() const
    {
        Property parameters;
        parameters.put("dt",dt);
        return parameters;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=chain.getDOF();

        // reaching in position
        m=1; nnz_jac_g=n;

        if (hitting_constraints)
        {
            // shoulder's cables length
            m+=3; nnz_jac_g+=2+3+2;

            // avoid hitting torso
            m+=1; nnz_jac_g+=2;

            // avoid hitting forearm
            m+=2; nnz_jac_g+=2+2;
        }

        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
            x_l[i]=bounds(i,0);
            x_u[i]=bounds(i,1);            
        }

        // reaching in position
        g_l[0]=g_u[0]=0.0;

        if (hitting_constraints)
        {
            // shoulder's cables length
            g_l[1]=-347.00*CTRL_DEG2RAD;
            g_u[1]=std::numeric_limits<double>::max();
            g_l[2]=-366.57*CTRL_DEG2RAD;
            g_u[2]=112.42*CTRL_DEG2RAD;
            g_l[3]=-66.60*CTRL_DEG2RAD;
            g_u[3]=213.30*CTRL_DEG2RAD;

            // avoid hitting torso
            g_l[4]=shou_n;
            g_u[4]=std::numeric_limits<double>::max();

            // avoid hitting forearm
            g_l[5]=-std::numeric_limits<double>::max();
            g_u[5]=elb_n;
            g_l[6]=-elb_n;
            g_u[6]=std::numeric_limits<double>::max();
        }

        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=std::min(std::max(bounds(i,0),v0[i]),bounds(i,1));
        return true;
    }

    /************************************************************************/
    void computeQuantities(const Ipopt::Number *x, const bool new_x)
    {
        if (new_x)
        {
            for (size_t i=0; i<v.length(); i++)
                v[i]=x[i];

            He.setSubmatrix(R0+dt*(skew(J0_ang*v)*R0),0,0);
            Vector pe=p0+dt*(J0_xyz*v);            
            He(0,3)=pe[0];
            He(1,3)=pe[1];
            He(2,3)=pe[2];

            err_xyz=pr-pe;
            err_ang=dcm2axis(Hr*He.transposed());
            err_ang*=err_ang[3];
            err_ang.pop_back();

            Matrix L=-0.5*(skew_nr*skew(He.getCol(0))+
                           skew_sr*skew(He.getCol(1))+
                           skew_ar*skew(He.getCol(2)));
            Derr_ang=-dt*(L*J0_ang);
        }
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);
        obj_value=(orientation_control?norm2(err_ang):0.0);
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);
        for (Ipopt::Index i=0; i<n; i++)
            grad_f[i]=(orientation_control?2.0*dot(err_ang,Derr_ang.getCol(i)):0.0);
        return true; 
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        // reaching in position
        g[0]=norm2(err_xyz);

        if (hitting_constraints)
        {
            // shoulder's cables length
            g[1]=1.71*(q0[3+0]+dt*x[3+0]-(q0[3+1]+dt*x[3+1]));
            g[2]=1.71*(q0[3+0]+dt*x[3+0]-(q0[3+1]+dt*x[3+1])-(q0[3+2]+dt*x[3+2]));
            g[3]=q0[3+1]+dt*x[3+1]+q0[3+2]+dt*x[3+2];

            // avoid hitting torso
            g[4]=q0[3+1]+dt*x[3+1]-shou_m*(q0[3+2]+dt*x[3+2]);

            // avoid hitting forearm
            g[5]=-elb_m*(q0[3+3+0]+dt*x[3+3+0])+q0[3+3+1]+dt*x[3+3+1];
            g[6]=elb_m*(q0[3+3+0]+dt*x[3+3+0])+q0[3+3+1]+dt*x[3+3+1];
        }

        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            Ipopt::Index idx=0;

            // reaching in position
            for (Ipopt::Index i=0; i<n; i++)
            {
                iRow[i]=0; jCol[i]=i;
                idx++;
            }

            if (hitting_constraints)
            {
                // shoulder's cables length
                iRow[idx]=1; jCol[idx]=3+0; idx++;
                iRow[idx]=1; jCol[idx]=3+1; idx++;

                iRow[idx]=2; jCol[idx]=3+0; idx++;
                iRow[idx]=2; jCol[idx]=3+1; idx++;
                iRow[idx]=2; jCol[idx]=3+2; idx++;

                iRow[idx]=3; jCol[idx]=3+1; idx++;
                iRow[idx]=3; jCol[idx]=3+2; idx++;

                // avoid hitting torso
                iRow[idx]=4; jCol[idx]=3+1; idx++;
                iRow[idx]=4; jCol[idx]=3+2; idx++;

                // avoid hitting forearm
                iRow[idx]=5; jCol[idx]=3+3+0; idx++;
                iRow[idx]=5; jCol[idx]=3+3+1; idx++;

                iRow[idx]=6; jCol[idx]=3+3+0; idx++;
                iRow[idx]=6; jCol[idx]=3+3+1; idx++;
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Index idx=0;

            // reaching in position
            for (Ipopt::Index i=0; i<n; i++)
            {
                values[i]=-2.0*dt*dot(err_xyz,J0_xyz.getCol(i));
                idx++;
            }

            if (hitting_constraints)
            {
                // shoulder's cables length
                values[idx++]=1.71*dt;
                values[idx++]=-1.71*dt;

                values[idx++]=1.71*dt;
                values[idx++]=-1.71*dt;
                values[idx++]=-1.71*dt;

                values[idx++]=dt;
                values[idx++]=dt;

                // avoid hitting torso
                values[idx++]=dt;
                values[idx++]=-shou_m*dt;

                // avoid hitting forearm
                values[idx++]=-elb_m*dt;
                values[idx++]=dt;

                values[idx++]=elb_m*dt;
                values[idx++]=dt;
            }
        }

        return true;
    }

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq)
    {
        for (Ipopt::Index i=0; i<n; i++)
            v[i]=x[i];
    }
};


/****************************************************************/
class Motor
{
    Property parameters;
    Integrator I;
    double dt;

public:
    /****************************************************************/
    Motor(const Vector &q0, const Matrix &lim,
          const double dt_) :
          I(dt_,q0,lim), dt(dt_)
    {
        parameters.put("dt",dt);
    }

    /****************************************************************/
    Vector move(const Vector &v)
    {
        return I.integrate(v);
    }

    /****************************************************************/
    Vector getPosition() const
    {
        return I.get();
    }

    /****************************************************************/
    Property getParameters() const
    {
        return parameters;
    }
};


/****************************************************************/
class Obstacle
{
    Integrator I;

public:
    double radius;
    Vector v;

    /****************************************************************/
    Obstacle(const Vector &x0, const double r,
             const Vector &v0, const double dt) :
             I(dt,x0), radius(r), v(v0) { }

    /****************************************************************/
    Vector move()
    {
        return I.integrate(v);
    }

    /****************************************************************/
    Vector getPosition() const
    {
        return I.get();
    }

    /****************************************************************/
    string toString() const
    {
        ostringstream str;
        str<<I.get().toString(3,3)<<" "<<radius;
        return str.str();
    }
};


/****************************************************************/
class AvoidanceHandlerAbstract
{
protected:
    string type;
    iKinChain &chain;
    deque<iKinChain*> chainCtrlPoints;
    Property parameters;

public:
    /****************************************************************/
    AvoidanceHandlerAbstract(iKinLimb &limb) : chain(*limb.asChain())
    {
        iKinLimb *limb_;
        Matrix HN=eye(4,4);

        limb_=new iKinLimb(limb);
        iKinChain *c1=limb_->asChain();
        c1->rmLink(9); c1->rmLink(8); c1->rmLink(7);
        HN(2,3)=0.1373;
        c1->setHN(HN);

        limb_=new iKinLimb(limb);
        iKinChain *c2=limb_->asChain();
        c2->rmLink(9); c2->rmLink(8); c2->rmLink(7);
        HN(2,3)=0.1373/2.0;
        c2->setHN(HN);

        // the first item is the original chain
        // used to account for the end-effector
        chainCtrlPoints.push_back(&chain);
        chainCtrlPoints.push_back(c1);
        chainCtrlPoints.push_back(c2);

        type="none";
    }

    /****************************************************************/
    string getType() const
    {
        return type;
    }

    /****************************************************************/
    virtual Property getParameters() const
    {
        return parameters;
    }

    /****************************************************************/
    virtual void setParameters(const Property &parameters)
    {
        this->parameters=parameters;
    }

    /****************************************************************/
    void updateCtrlPoints()
    {
        // i>0: original chain is up-to-date
        for (size_t i=1; i<chainCtrlPoints.size(); i++)
            for (size_t j=0; j<chainCtrlPoints[i]->getDOF(); j++)
                chainCtrlPoints[i]->setAng(j,chain(j).getAng());
    }

    /****************************************************************/
    deque<Vector> getCtrlPointsPosition()
    {
        deque<Vector> ctrlPoints;
        for (size_t i=0; i<chainCtrlPoints.size(); i++)
            ctrlPoints.push_back(chainCtrlPoints[i]->EndEffPosition());

        return ctrlPoints;
    }

    /****************************************************************/
    virtual Matrix getVLIM(const Obstacle &obstacle, const Matrix &v_lim)
    {
        return v_lim;
    }

    /****************************************************************/
    virtual ~AvoidanceHandlerAbstract()
    {
        // i>0: don't dispose original chain
        for (size_t i=1; i<chainCtrlPoints.size(); i++)
            delete chainCtrlPoints[i];
    }
};


/****************************************************************/
class AvoidanceHandlerVisuo : public virtual AvoidanceHandlerAbstract
{
protected:
    double rho;
    double alpha;

public:
    /****************************************************************/
    AvoidanceHandlerVisuo(iKinLimb &limb) : AvoidanceHandlerAbstract(limb)
    {
        type="visuo";
        rho=0.4;
        alpha=6.0;

        parameters.unput("rho");
        parameters.put("rho",rho);

        parameters.unput("alpha");
        parameters.put("alpha",alpha);
    }

    /****************************************************************/
    void setParameters(const Property &parameters)
    {
        if (parameters.check("rho"))
        {
            rho=parameters.find("rho").asDouble();
            this->parameters.unput("rho");
            this->parameters.put("rho",rho);
        }

        if (parameters.check("alpha"))
        {
            alpha=parameters.find("alpha").asDouble();
            this->parameters.unput("alpha");
            this->parameters.put("alpha",alpha);
        }
    }

    /****************************************************************/
    Matrix getVLIM(const Obstacle &obstacle, const Matrix &v_lim)
    {
        Vector xo=obstacle.getPosition();
        deque<Vector> ctrlPoints=getCtrlPointsPosition();

        Matrix VLIM=v_lim;
        for (size_t i=0; i<ctrlPoints.size(); i++)
        {
            Vector dist=xo-ctrlPoints[i];
            double d=norm(dist);
            if (d>=obstacle.radius)
            {
                dist*=1.0-obstacle.radius/d;
                d=norm(dist);
            }
            else
            {
                dist=0.0;
                d=0.0;
            }
            
            double f=1.0/(1.0+exp((d*(2.0/rho)-1.0)*alpha));
            Matrix J=chainCtrlPoints[i]->GeoJacobian().submatrix(0,2,0,chainCtrlPoints[i]->getDOF()-1);
            Vector s=J.transposed()*dist;

            double red=1.0-f;
            for (size_t j=0; j<s.length(); j++)
            {
                if (s[j]>=0.0)
                {
                    double tmp=v_lim(j,1)*red;
                    VLIM(j,1)=std::min(VLIM(j,1),tmp);
                    VLIM(j,0)=std::min(VLIM(j,0),VLIM(j,1));
                }
                else
                {
                    double tmp=v_lim(j,0)*red;
                    VLIM(j,0)=std::max(VLIM(j,0),tmp);
                    VLIM(j,1)=std::max(VLIM(j,0),VLIM(j,1));
                }
            }
        }

        return VLIM;
    }
};


/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandlerAbstract
{
protected:
    double k;

public:
    /****************************************************************/
    AvoidanceHandlerTactile(iKinLimb &limb) : AvoidanceHandlerAbstract(limb)
    {
        type="tactile";

        // produce 30 deg/s repulsive
        // velocity for 3 mm of penetration
        k=30.0/0.003;

        parameters.unput("k");
        parameters.put("k",k);
    }

    /****************************************************************/
    void setParameters(const Property &parameters)
    {
        if (parameters.check("k"))
        {
            k=parameters.find("k").asDouble();
            this->parameters.unput("k");
            this->parameters.put("k",k);
        }
    }

    /****************************************************************/
    Matrix getVLIM(const Obstacle &obstacle, const Matrix &v_lim)
    {
        Vector xo=obstacle.getPosition();
        deque<Vector> ctrlPoints=getCtrlPointsPosition();

        Matrix VLIM=v_lim;
        for (size_t i=0; i<ctrlPoints.size(); i++)
        {
            Vector dist=xo-ctrlPoints[i];
            double d=norm(dist);
            if (d>=obstacle.radius)
                continue;
            
            double f=k*(obstacle.radius-d);
            Matrix J=chainCtrlPoints[i]->GeoJacobian().submatrix(0,2,0,chainCtrlPoints[i]->getDOF()-1);
            Vector s=(-f/d)*(J.transposed()*dist);
            
            for (size_t j=0; j<s.length(); j++)
            {
                if (s[j]>=0.0)
                {
                    s[j]=std::min(v_lim(j,1),s[j]);
                    VLIM(j,0)=std::max(VLIM(j,0),s[j]);
                    VLIM(j,1)=std::max(VLIM(j,0),VLIM(j,1));
                }
                else
                {
                    s[j]=std::max(v_lim(j,0),s[j]);
                    VLIM(j,1)=std::min(VLIM(j,1),s[j]);
                    VLIM(j,0)=std::min(VLIM(j,0),VLIM(j,1));
                }
            }
        }

        return VLIM;
    }
};


/****************************************************************/
class AvoidanceHandlerVisuoTactile : public AvoidanceHandlerVisuo,
                                     public AvoidanceHandlerTactile
{
public:
    /****************************************************************/
    AvoidanceHandlerVisuoTactile(iKinLimb &limb) : AvoidanceHandlerAbstract(limb),
                                                   AvoidanceHandlerVisuo(limb),
                                                   AvoidanceHandlerTactile(limb)
    {
        type="visuo-tactile";
    }

    /****************************************************************/
    void setParameters(const Property &parameters)
    {
        AvoidanceHandlerVisuo::setParameters(parameters);
        AvoidanceHandlerTactile::setParameters(parameters);
    }

    /****************************************************************/
    Matrix getVLIM(const Obstacle &obstacle, const Matrix &v_lim)
    {
        return AvoidanceHandlerTactile::getVLIM(obstacle,
               AvoidanceHandlerVisuo::getVLIM(obstacle,v_lim));
    }
};


/****************************************************************/
class ControllerModule : public RFModule
{
    PolyDriver drvTorso,drvArm;
    VectorOf<int> armJoints;

    iCubArm *arm;
    iKinChain *chain;
    
    AvoidanceHandlerAbstract *avhdl;
    minJerkTrajGen *target;
    Obstacle *obstacle;
    Motor *motor;

    double dt,T,t0;
    bool hitting_constraints;
    bool orientation_control;
    bool verbosity;
    Matrix lim,v_lim;

    Vector v;
    ofstream fout;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        dt=rf.check("dt",Value(0.02)).asDouble();
        T=rf.check("T",Value(1.0)).asDouble();
        hitting_constraints=rf.check("hitting-constraints",Value("on")).asString()=="on";
        orientation_control=rf.check("orientation-control",Value("on")).asString()=="on";
        verbosity=rf.check("verbosity",Value("off")).asString()=="on";
        string avoidance_type=rf.check("avoidance-type",Value("tactile")).asString();
        string robot=rf.check("robot",Value("icub")).asString();        

        arm=new iCubArm("left");
        chain=arm->asChain();

        chain->releaseLink(0);
        chain->releaseLink(1);
        chain->releaseLink(2);

        if (avoidance_type=="none")
            avhdl=new AvoidanceHandlerAbstract(*arm);
        else if (avoidance_type=="visuo")
            avhdl=new AvoidanceHandlerVisuo(*arm);
        else if (avoidance_type=="tactile")
            avhdl=new AvoidanceHandlerTactile(*arm);
        else if (avoidance_type=="visuo-tactile")
            avhdl=new AvoidanceHandlerVisuoTactile(*arm); 
        else
        {
            yError()<<"Unrecognized avoidance type! exiting ...";
            delete arm;
            return false;
        }

        Property option("(device remote_controlboard)");
        option.put("remote",("/"+robot+"/torso").c_str());
        option.put("local","/test-reactController/torso");
        if (!drvTorso.open(option))
        {
            yError()<<"Unable to open torso driver";
            delete arm;
            return false;
        }

        option.unput("remote"); option.unput("local");
        option.put("remote",("/"+robot+"/left_arm").c_str());
        option.put("local","/test-reactController/left_arm");
        if (!drvArm.open(option))
        {
            yError()<<"Unable to open arm driver";
            drvTorso.close();
            delete arm;
            return false;
        }

        IControlLimits *ilim_torso,*ilim_arm;
        drvTorso.view(ilim_torso);
        drvArm.view(ilim_arm);
        deque<IControlLimits*> ilim;
        ilim.push_back(ilim_torso);
        ilim.push_back(ilim_arm);
        arm->alignJointsBounds(ilim);

        VectorOf<int> modes;
        for (size_t i=0; i<7; i++)
        {
            armJoints.push_back(i);
            modes.push_back(VOCAB_CM_POSITION_DIRECT);
        }

        IControlMode2 *imod;

        drvTorso.view(imod);
        imod->setControlModes(modes.getFirst());

        drvArm.view(imod);
        imod->setControlModes(armJoints.size(),armJoints.getFirst(),modes.getFirst());

        Vector q0(chain->getDOF());
        IEncoders *ienc;
        int numEncs;
        Vector encs;
        size_t cnt; 

        drvTorso.view(ienc);
        ienc->getAxes(&numEncs);
        encs.resize(numEncs);
        while (!ienc->getEncoders(encs.data()))
        {
            yInfo()<<"waiting for torso feedback";
            Time::delay(0.1);
        }
        for (cnt=0; cnt<encs.length(); cnt++)
            q0[cnt]=encs[encs.length()-cnt-1];  // torso: swap readings
        
        drvArm.view(ienc);
        ienc->getAxes(&numEncs);
        encs.resize(numEncs);
        while (!ienc->getEncoders(encs.data()))
        {
            yInfo()<<"waiting for arm feedback";
            Time::delay(0.1);
        }
        for (size_t offs=cnt; cnt<q0.length(); cnt++)
            q0[cnt]=encs[cnt-offs];

        chain->setAng(CTRL_DEG2RAD*q0);

        lim.resize(chain->getDOF(),2);
        v_lim.resize(chain->getDOF(),2);
        for (size_t r=0; r<chain->getDOF(); r++)
        {
            lim(r,0)=CTRL_RAD2DEG*(*chain)(r).getMin();
            lim(r,1)=CTRL_RAD2DEG*(*chain)(r).getMax();
            v_lim(r,0)=-30.0;
            v_lim(r,1)=+30.0;
        }
        v_lim(1,0)=v_lim(1,1)=0.0;  // disable torso roll
        
        motor=new Motor(q0,lim,dt);

        Vector xee=chain->EndEffPose();
        xee[3]*=xee[6];
        xee[4]*=xee[6];
        xee[5]*=xee[6];
        xee.pop_back();
        target=new minJerkTrajGen(xee,dt,T);
        
        Vector xo(3);
        xo[0]=-0.30;
        xo[1]=-0.20;
        xo[2]=+0.4;
        Vector vo(3,0.0);
        vo[2]=-0.05;
        obstacle=new Obstacle(xo,0.05,vo,dt);

        v.resize(chain->getDOF(),0.0);
        fout.open("data.log");
        t0=Time::now();
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return dt;
    }

    /****************************************************************/
    bool updateModule()
    {
        double t=Time::now()-t0;

        Vector xd(6);
        double rt=0.05;
        xd[0]=-0.28;
        xd[1]=-0.20+rt*cos(2.0*M_PI*0.2*t);
        xd[2]=+0.05+rt*sin(2.0*M_PI*0.2*t);
        xd[3]=0.0;
        xd[4]=0.0;
        xd[5]=M_PI;

        target->computeNextValues(xd);
        Vector xr=target->getPos();
        Vector xo=obstacle->move();

        avhdl->updateCtrlPoints();
        Matrix VLIM=avhdl->getVLIM(*obstacle,v_lim);

        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",1e-3);
        app->Options()->SetNumericValue("constr_viol_tol",1e-6);
        app->Options()->SetIntegerValue("acceptable_iter",0);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
        app->Options()->SetNumericValue("max_cpu_time",0.75*dt);
        app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetStringValue("derivative_test",verbosity?"first-order":"none");
        app->Options()->SetIntegerValue("print_level",verbosity?5:0);
        app->Initialize();

        Ipopt::SmartPtr<ControllerNLP> nlp=new ControllerNLP(*chain);
        nlp->set_hitting_constraints(hitting_constraints);
        nlp->set_orientation_control(orientation_control);
        nlp->set_dt(dt);
        nlp->set_xr(xr);
        nlp->set_v_lim(VLIM);
        nlp->set_v0(v);
        nlp->init();

        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

        v=nlp->get_result();
        Vector refs=motor->move(v);

        IPositionDirect *idir;

        drvTorso.view(idir);
        Vector refs_swapped(3);
        refs_swapped[0]=refs[2];
        refs_swapped[1]=refs[1];
        refs_swapped[2]=refs[0];
        idir->setPositions(refs_swapped.data());

        drvArm.view(idir);
        idir->setPositions(armJoints.size(),armJoints.getFirst(),
                           refs.subVector(3,3+armJoints.size()-1).data());

        chain->setAng(CTRL_DEG2RAD*refs);
        Vector xee=chain->EndEffPose();
        Vector xee_pos=xee.subVector(0,2);
        Vector xee_ang=xee[6]*xee.subVector(3,5);

        yInfo()<<"       t [s] = "<<t;
        yInfo()<<"   v [deg/s] = ("<<v.toString(3,3)<<")";
        yInfo()<<"   e_pos [m] = "<<norm(xr.subVector(0,2)-xee_pos);
        yInfo()<<" e_ang [rad] = "<<norm(xr.subVector(3,5)-xee_ang);
        yInfo()<<"";

        ostringstream strCtrlPoints;
        deque<Vector> ctrlPoints=avhdl->getCtrlPointsPosition();
        for (size_t i=0; i<ctrlPoints.size(); i++)
            strCtrlPoints<<ctrlPoints[i].toString(3,3)<<" ";

        fout<<t<<" "<<
              xr.toString(3,3)<<" "<<
              obstacle->toString()<<" "<<
              v.toString(3,3)<<" "<<
              (CTRL_RAD2DEG*chain->getAng()).toString(3,3)<<" "<<
              strCtrlPoints.str()<<
              endl;

        return true;
    }

    /****************************************************************/
    bool close()
    {
        IPositionControl2 *ipos;

        drvTorso.view(ipos);
        ipos->stop();

        drvArm.view(ipos);
        ipos->stop(armJoints.size(),armJoints.getFirst());

        drvTorso.close();
        drvArm.close();

        delete obstacle;
        delete target;
        delete motor;
        delete avhdl;
        delete arm;

        fout.close();
        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP is not available";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    ControllerModule controller;
    return controller.runModule(rf);
}


