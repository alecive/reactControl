/* 
 * Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>, Matej Hoffmann <matej.hoffmann@iit.it> 
 * website: www.icub.org
 * 
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


#include "avoidanceHandler.h"

using namespace std;

using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::iKin;
using namespace iCub::skinDynLib;

AvoidanceHandlerAbstract::AvoidanceHandlerAbstract(const iCub::iKin::iKinChain &_chain, const std::vector<collisionPoint_t> &_collisionPoints,const unsigned int _verbosity):
                                                   chain(_chain), collisionPoints(_collisionPoints), verbosity(_verbosity), type("none") {}
    
/****************************************************************/
string AvoidanceHandlerAbstract::getType() const
{
    return type;
}

/****************************************************************/
Property AvoidanceHandlerAbstract::getParameters() const
{
    return parameters;
}

/****************************************************************/
void AvoidanceHandlerAbstract::setParameters(const Property &params)
{
    parameters=params;
}


/****************************************************************/
deque<Vector> AvoidanceHandlerAbstract::getCtrlPointsPosition()
{
    deque<Vector> ctrlPoints;
    for (auto & ctrlPointChain : ctrlPointChains)
        ctrlPoints.push_back(ctrlPointChain.EndEffPosition());
    return ctrlPoints;
}

/****************************************************************/
Matrix AvoidanceHandlerAbstract::getVLIM(const Matrix &v_lim)
{
    return v_lim;
}

/****************************************************************/
AvoidanceHandlerAbstract::~AvoidanceHandlerAbstract()
{
    ctrlPointChains.clear();
}


int AvoidanceHandlerAbstract::printMessage(const unsigned int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",type.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    else
        return -1;
}


//creates a full transform as given by a DCM matrix at the pos and norm w.r.t. the original frame, from the pos and norm (one axis set arbitrarily)  
bool AvoidanceHandlerAbstract::computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR)
{
    if (norm == zeros(3))
    {
        FoR=eye(4);
        return false;
    }
        
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
    


/****************************************************************/
AvoidanceHandlerTactile::AvoidanceHandlerTactile(const iCub::iKin::iKinChain &_chain,const std::vector<collisionPoint_t> &_collisionPoints,const unsigned int _verbosity): AvoidanceHandlerAbstract(_chain,_collisionPoints,_verbosity)
{
    type="tactile";

    avoidingSpeed = 1;  // produce collisionPoint.magnitude * avoidingSpeed rad/s repulsive speed
        
    parameters.unput("avoidingSpeed");
    parameters.put("avoidingSpeed",avoidingSpeed);
}

/****************************************************************/
void AvoidanceHandlerTactile::setParameters(const Property &parameters)
{
    if (parameters.check("avoidingVelocity"))
    {
        avoidingSpeed=parameters.find("avoidingSpeed").asDouble();
        this->parameters.unput("avoidingSpeed");
        this->parameters.put("avoidingSpeed",avoidingSpeed);
    }
}

/****************************************************************/
Matrix AvoidanceHandlerTactile::getVLIM(const Matrix &v_lim)
{
    printMessage(2,"AvoidanceHandlerTactile::getVLIM\n");
    Matrix VLIM=v_lim;
    int dim = static_cast<int>(chain.getDOF());
    int i = 0;
    int dim_offset = dim-7;  // 3 if dim == 10; 0 if dim == 7
    ctrlPointChains.clear();
    for(const auto & colPoint : collisionPoints) {
        iKinChain customChain= chain; //instantiates a new chain, copying from the old (full) one
        if (verbosity >= 5){
            printf("Full chain has %d DOF \n",dim);
            printf("chain.getH() (end-effector): \n %s \n",chain.getH().toString(3,3).c_str());
            printf("SkinPart %s, linkNum %d, chain.getH() (skin part frame): \n %s \n", SkinPart_s[colPoint.skin_part].c_str(), SkinPart_2_LinkNum[colPoint.skin_part].linkNum + dim_offset , chain.getH(SkinPart_2_LinkNum[colPoint.skin_part].linkNum + dim_offset).toString(3, 3).c_str());

            if (dim ==7){
                printf("SkinPart %s, linkNum %d, chain.getH() (skin part frame): \n %s \n", SkinPart_s[colPoint.skin_part].c_str(), SkinPart_2_LinkNum[colPoint.skin_part].linkNum, chain.getH(SkinPart_2_LinkNum[colPoint.skin_part].linkNum).toString(3, 3).c_str());
            }
            else if (dim == 10){
                printf("SkinPart %s, linkNum %d + 3, chain.getH() (skin part frame): \n %s \n", SkinPart_s[colPoint.skin_part].c_str(), SkinPart_2_LinkNum[colPoint.skin_part].linkNum, chain.getH(SkinPart_2_LinkNum[colPoint.skin_part].linkNum + 3).toString(3, 3).c_str());
            }
        }

        // Remove all the more distal links after the collision point
        // if the skin part is a hand, no need to remove any links from the chain
        if ((colPoint.skin_part == SKIN_LEFT_FOREARM) || (colPoint.skin_part == SKIN_RIGHT_FOREARM)){
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset);
            // we keep link 4(+3) from elbow to wrist - it is getH(4(+3)) that is the FoR at the wrist in which forearm skin is expressed; and we want to keep the elbow joint part of the game
            printMessage(2,"obstacle threatening skin part %s, blocking links 5(+3) and 6(+3) on subchain for avoidance\n",SkinPart_s[colPoint.skin_part].c_str());
        }
        else if ((colPoint.skin_part == SKIN_LEFT_UPPER_ARM) || (colPoint.skin_part == SKIN_RIGHT_UPPER_ARM)){
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset);customChain.rmLink(4+dim_offset);customChain.rmLink(3+dim_offset);
            printMessage(2,"obstacle threatening skin part %s, blocking links 3(+3)-6(+3) on subchain for avoidance\n",SkinPart_s[colPoint.skin_part].c_str());
        }

        // SetHN to move the end effector toward the point to be controlled - the average locus of collision threat from safety margin
        yarp::sig::Matrix HN = eye(4);
        computeFoR(colPoint.x, colPoint.n, HN);
        printMessage(5,"HN matrix at collision point w.r.t. local frame: \n %s \n",HN.toString(3,3).c_str());
        customChain.setHN(HN); //setting the end-effector transform to the collision point w.r.t subchain
        if (verbosity >=5){
            yarp::sig::Matrix H = customChain.getH();
            printf("H matrix at collision point w.r.t. root: \n %s \n",H.toString(3,3).c_str());
        }

        printMessage(2,"Chain with control point - index %d (last index %d), nDOF: %d.\n",i,collisionPoints.size()-1,customChain.getDOF());
        Matrix J=customChain.GeoJacobian().submatrix(0,2,0,customChain.getDOF()-1); //first 3 rows ~ dPosition/dJoints
        Vector normal = customChain.getH().getCol(2).subVector(0,2); //get the end-effector frame of the standard or custom chain (control point derived from skin), takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the 4 in the homogenous transf. format
        Vector s=(J.transposed()*normal) * avoidingSpeed * colPoint.magnitude; //project movement along the normal into joint velocity space and scale by default avoidingSpeed and magnitude of skin (or PPS) activation
        if (verbosity>=2){
            printf("J for positions at control point:\n %s \nJ.transposed:\n %s \nNormal at control point: (%s), norm: %f \n",J.toString(3,3).c_str(),J.transposed().toString(3,3).c_str(), normal.toString(3,3).c_str(),norm(normal));
            printf("s = (J.transposed()*normal) * avoidingSpeed * collisionPoints[i].magnitude \n (%s)T = (%s)T * %f * %f\n",s.toString(3,3).c_str(),(J.transposed()*normal).toString(3,3).c_str(),avoidingSpeed,colPoint.magnitude);
        }
        s = s * -1.0; //we reverse the direction to obtain joint velocities that bring about avoidance
        printMessage(2,"s * (-1) -> joint contributions toward avoidance: \n (%s) \n",s.toString(3,3).c_str());

        for (size_t j=0; j<s.length(); j++)
        {
            printMessage(2,"        Joint: %d, s[j]: %f, limits before: Min: %f, Max: %f\n",j,s[j],VLIM(j,0),VLIM(j,1));
            if (s[j]>=0.0) //joint contributes to avoidance, we will set the min velocity accordingly
            {
                s[j]=std::min(v_lim(j,1),s[j]); //make sure new min vel is <= max vel
                VLIM(j,0)=std::max(VLIM(j,0),s[j]); // set min vel to max of s[j] and current limit ~ avoiding action
                VLIM(j,1)=std::max(VLIM(j,0),VLIM(j,1)); //make sure current max is at least equal to current min
                printMessage(2,"            s>=0 clause, joint contributes to avoidance, adjusting Min; limits after: Min: %f, Max: %f\n",VLIM(j,0),VLIM(j,1));
            }
            else //joint acts to bring control point toward obstacle - we will shape the max vel
            {
                s[j]=std::max(v_lim(j,0),s[j]);
                VLIM(j,1)=std::min(VLIM(j,1),s[j]);
                VLIM(j,0)=std::min(VLIM(j,0),VLIM(j,1));
                printMessage(2,"            s<0 clause, joint contributes to approach, adjusting Max; limits after: Min: %f, Max: %f\n",VLIM(j,0),VLIM(j,1));
            }
        }
        ctrlPointChains.push_back(customChain);
        i++;
    }
    return VLIM;
}







 

