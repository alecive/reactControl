/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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

#include <csignal>
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
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iKin/iKinFwd.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/****************************************************************/
class ControllerNLP : public Ipopt::TNLP
{
    iKinChain &chain;

    Vector xr; //target position
    Vector x0; // current end-effector position
    Vector delta_x;
    Vector v0;
    Matrix v_lim; //velocity limits, #rows ~ #DOF, first column minima, 2nd column maxima; in radians 
    Matrix bounds;
    Matrix J0; //end-effector Jacobian for position (leaving orientation aside)
    Vector v;
    double dt;

    Vector qGuard;
    Vector qGuardMinExt;
    Vector qGuardMinInt;
    Vector qGuardMinCOG;
    Vector qGuardMaxExt;
    Vector qGuardMaxInt;
    Vector qGuardMaxCOG;

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
            double qi=chain(i).getAng();
            if ((qi>=qGuardMinInt[i]) && (qi<=qGuardMaxInt[i]))
                bounds(i,0)=bounds(i,1)=1.0;
            else if ((qi<=qGuardMinExt[i]) || (qi>=qGuardMaxExt[i]))
                bounds(i,0)=bounds(i,1)=0.0;
            else if (qi<qGuardMinInt[i])
            {
                bounds(i,0)=0.5*(1.0+tanh(+10.0*(qi-qGuardMinCOG[i])/qGuard[i]));
                bounds(i,1)=1.0;
            }
            else
            {
                bounds(i,0)=1.0;
                bounds(i,1)=0.5*(1.0+tanh(-10.0*(qi-qGuardMaxCOG[i])/qGuard[i]));
            }
        }
        
        for (size_t i=0; i<chain.getDOF(); i++)
        {
            bounds(i,0)*=v_lim(i,0);
            bounds(i,1)*=v_lim(i,1);
        }
    }

public:
    /****************************************************************/
    ControllerNLP(iKinChain &chain_) : chain(chain_)
    {
        xr.resize(3,0.0);
        v0.resize(chain.getDOF(),0.0);
        v=v0;

        v_lim.resize(chain.getDOF(),2);
        for (size_t r=0; r<chain.getDOF(); r++)
        {
            v_lim(r,1)=std::numeric_limits<double>::max();
            v_lim(r,0)=-v_lim(r,1);
        }
        bounds=v_lim;

        computeGuard();
        dt=0.0;
    }

    /****************************************************************/
    void set_xr(const Vector &xr)
    {
        yAssert(this->xr.length()==xr.length());
        this->xr=xr;
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
        x0=chain.EndEffPosition();
        J0=chain.GeoJacobian().submatrix(0,2,0,chain.getDOF()-1); //first 3 rows ~ delta position; 
        computeBounds();
    }

    /****************************************************************/
    Vector get_result() const
    {
        return v;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=chain.getDOF();
        m=nnz_jac_g=nnz_h_lag=0;
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
                v[i]=x[i]; //x are the primal variables optimized by Ipopt - in this case they are the joint velocities
            delta_x=xr-(x0+dt*(J0*v)); //difference between target position and new end-eff position
        }
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);
        obj_value=norm2(delta_x);
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);
        for (Ipopt::Index i=0; i<n; i++)
            grad_f[i]=-2.0*dt*dot(delta_x,J0.getCol(i));
        return true; 
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        return false;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return false;
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
            v[i]=CTRL_RAD2DEG*x[i];
    }
};


/****************************************************************/
class Motor
{
    Integrator I;
    Filter *F;

public:
    /****************************************************************/
    Motor(const Vector &q0, const Matrix &lim,
          const double tau, const double dt) :
          I(dt,q0,lim)
    {
        double c=2.0*tau/dt;
        Vector den(2,1.0);
        den[0]+=c; den[1]-=c;
        F=new Filter(Vector(2,1.0),den,q0);
    }

    /****************************************************************/
    Vector move(const Vector &v)
    {
        return F->filt(I.integrate(v));
    }

    /****************************************************************/
    Vector getPosition() const
    {
        return F->output();
    }

    /****************************************************************/
    ~Motor()
    {
        delete F;
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
        // i>0: original chain (one to the end-effector) is up-to-date
        for (size_t i=1; i<chainCtrlPoints.size(); i++)
            for (size_t j=0; j<chainCtrlPoints[i]->getDOF(); j++)
                chainCtrlPoints[i]->setAng(j,chain(j).getAng()); //updating the local chains with current config of the orig chain - 
                //only for joint values in the local (possibly shorter) chain
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

        // produce 50 deg/s repulsive
        // velocity for 1 mm of penetration
        k=50.0/0.001;

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
        Matrix VLIM=AvoidanceHandlerVisuo::getVLIM(obstacle,v_lim);
        VLIM=AvoidanceHandlerTactile::getVLIM(obstacle,VLIM);
        return VLIM;
    }
};


/****************************************************************/
namespace
{
    volatile std::sig_atomic_t gSignalStatus;
}


/****************************************************************/
void signal_handler(int signal)
{
    gSignalStatus=signal;
}


/****************************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("react-control");
    rf.setDefaultConfigFile("reactController-sim.ini");
    rf.configure(argc,argv);

    int verbosity = rf.check("verbosity",Value(0)).asInt();
    double sim_time=rf.check("sim-time",Value(10.0)).asDouble();
    double motor_tau=rf.check("motor-tau",Value(0.0)).asDouble(); //motor transfer function
    string avoidance_type=rf.check("avoidance-type",Value("tactile")).asString();   //none | visuo | tactile 
    string target_type=rf.check("target-type",Value("moving-circular")).asString(); // moving-circular | static

    yInfo("Starting with the following parameters: \n verbosity: %d \n sim-time: %f \n motor-tau: %f \n avoidance-type: %s \n target-type: %s \n",verbosity,sim_time,motor_tau,avoidance_type.c_str(),target_type.c_str());
    
    if (!yarp.checkNetwork())
    {
        yError("No Network!!!");
        return -1;
    }
    
    iCubArm arm("left");
    iKinChain &chain=*arm.asChain();
    chain.releaseLink(0); //releasing torso links that are blocked by default
    chain.releaseLink(1);
    chain.releaseLink(2);

    Vector q0(chain.getDOF(),0.0);
    q0[3]=-25.0; q0[4]=20.0; q0[6]=50.0; //setting shoulder position
    chain.setAng(CTRL_DEG2RAD*q0);

    Matrix lim(chain.getDOF(),2);
    Matrix v_lim(chain.getDOF(),2);
    for (size_t r=0; r<chain.getDOF(); r++)
    {
        lim(r,0)=CTRL_RAD2DEG*chain(r).getMin();
        lim(r,1)=CTRL_RAD2DEG*chain(r).getMax();
        v_lim(r,0)=-50.0;
        v_lim(r,1)=+50.0;
    }
    v_lim(1,0)=v_lim(1,1)=0.0;  // disable torso roll

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",1e-6);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",10000);
    app->Options()->SetNumericValue("max_cpu_time",0.05);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test","none");
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    Ipopt::SmartPtr<ControllerNLP> nlp=new ControllerNLP(chain);

    AvoidanceHandlerAbstract *avhdl;    
    if (avoidance_type=="none")
        avhdl=new AvoidanceHandlerAbstract(arm);
    else if (avoidance_type=="visuo")
        avhdl=new AvoidanceHandlerVisuo(arm);
    else if (avoidance_type=="tactile")
        avhdl=new AvoidanceHandlerTactile(arm);
    else if (avoidance_type=="visuo-tactile")
        avhdl=new AvoidanceHandlerVisuoTactile(arm); 
    else
    {
        yError()<<"unrecognized avoidance type! exiting ...";
        return 1;
    }
    yInfo()<<"Avoidance-Handler="<<avhdl->getType();
    yInfo()<<"Avoidance Parameters="<<avhdl->getParameters().toString();

    double dt=0.01;
    double T=1.0;

    nlp->set_dt(dt);
    Motor motor(q0,lim,motor_tau,dt);
    Vector v(chain.getDOF(),0.0);

    Vector xee=chain.EndEffPosition();
    minJerkTrajGen target(xee,dt,T);
    Vector xc(3); //center of target
    xc[0]=-0.35;
    xc[1]=0.0;
    xc[2]=0.1;
    double rt=1.0; //target will be moving along circular trajectory with this radius
    //if (target_type == "moving-circular") double rt=0.1; 
    if (target_type == "static")
        rt=0.0; //static target will be "moving" along a trajectory with 0 radius
    
    Vector xo(3);
    xo[0]=-0.3;
    xo[1]=0.0;
    xo[2]=0.4;
    Vector vo(3,0.0);
    vo[2]=-0.1;
    Obstacle obstacle(xo,0.07,vo,dt);

    ofstream fout_param; //log parameters that stay constant during the simulation, but are important for analysis - e.g. joint limits 
    ofstream fout; //to log data every iteration
    
    fout_param.open("param.log");
    fout.open("data.log");
    
    fout_param<<chain.getDOF()<<" ";
    for (size_t i=0; i<chain.getDOF(); i++)
    {
        fout_param<<CTRL_RAD2DEG*chain(i).getMin()<<" ";
        fout_param<<CTRL_RAD2DEG*chain(i).getMax()<<" ";
    }
    for (size_t i=0; i<chain.getDOF(); i++)
    {
        fout_param<<v_lim(i,0)<<" ";
        fout_param<<v_lim(i,1)<<" ";
    }
    
            
    std::signal(SIGINT,signal_handler);
    for (double t=0.0; t<sim_time; t+=dt)
    {
        Vector xd=xc; //target moving along circular trajectory
        xd[1]+=rt*cos(2.0*M_PI*0.3*t);
        xd[2]+=rt*sin(2.0*M_PI*0.3*t);

        target.computeNextValues(xd);
        Vector xr=target.getPos(); //target for end-effector

        xo=obstacle.move();

        avhdl->updateCtrlPoints();
        Matrix VLIM=avhdl->getVLIM(obstacle,v_lim);

        nlp->set_xr(xr);
        nlp->set_v_lim(VLIM);
        nlp->set_v0(v);
        nlp->init();
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
        v=nlp->get_result(); //updating the joint velocities with the output from optimizer

        xee=chain.EndEffPosition(CTRL_DEG2RAD*motor.move(v));

        if (verbosity>0){ //this is probably not the right way to do it
            yInfo()<<"        t [s] = "<<t;
            yInfo()<<"    v [deg/s] = ("<<v.toString(3,3)<<")";
            yInfo()<<" |xr-xee| [m] = "<<norm(xr-xee);
            yInfo()<<"";
        }

        ostringstream strCtrlPoints;
        deque<Vector> ctrlPoints=avhdl->getCtrlPointsPosition();
        for (size_t i=0; i<ctrlPoints.size(); i++)
            strCtrlPoints<<ctrlPoints[i].toString(3,3)<<" ";

        fout<<t<<" "<<
              xr.toString(3,3)<<" "<<
              obstacle.toString()<<" "<<
              v.toString(3,3)<<" "<<
              (CTRL_RAD2DEG*chain.getAng()).toString(3,3)<<" "<<
              strCtrlPoints.str()<<
              endl;
              //in columns on the output for 10 DOF case: 1:time, 2:4 target, 5:8 obstacle, 9:18 joint velocities, 19:28 joint pos, 29:end - control points     
        if (gSignalStatus==SIGINT)
        {
            yWarning("SIGINT detected: exiting ...");
            break;
        }
    }

    fout.close();
    delete avhdl;

    return 0;
}


