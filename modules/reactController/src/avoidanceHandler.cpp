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

AvoidanceHandlerAbstract::AvoidanceHandlerAbstract(iKinLimb &limb, const std::vector<collisionPoint_t> &_collisionPoints) : chain(*limb.asChain()), collisionPoints(_collisionPoints)
{
        if (!collisionPoints.empty()){
            for(std::vector<collisionPoint_t>::const_iterator it = collisionPoints.begin(); it != collisionPoints.end(); ++it) {
                iKinLimb *limb_; 
                limb_=new iKinLimb(limb);
                iKinChain *c=limb_->asChain();
                   
                if (verbosity >= 5){
                    printf("Full chain has %d DOF \n",chain.getDOF());
                    printf("chain.getH(): \n %s \n",chain.getH().toString(3,3).c_str());
                    int linkNrForCurrentSkinPartFrame = 0;
                    if (dim ==7){
                        printf("SkinPart %s, linkNum %d, chain.getH(): \n %s \n",SkinPart_s[(*it).skin_part].c_str(),SkinPart_2_LinkNum[(*it).skin_part].linkNum,chain.getH(SkinPart_2_LinkNum[(*it).skin_part].linkNum).toString(3,3).c_str());
                    }
                    else if (dim == 10){
                         printf("SkinPart %s, linkNum %d + 3, chain.getH(): \n %s \n",SkinPart_s[(*it).skin_part].c_str(),SkinPart_2_LinkNum[(*it).skin_part].linkNum,chain.getH(SkinPart_2_LinkNum[(*it).skin_part].linkNum + 3).toString(3,3).c_str());
                    }
                    yarp::sig::Matrix JfullChain = chain.GeoJacobian(); //6 rows, n columns for every active DOF
                    printMessage(5,"GeoJacobian matrix for canonical end-effector (palm): \n %s \n",JfullChain.toString(3,3).c_str());
                }
                 
                // Remove all the more distal links after the collision point 
                // if the skin part is a hand, no need to remove any links from the chain
                if (((*it).skin_part == SKIN_LEFT_FOREARM) ||  ((*it).skin_part == SKIN_RIGHT_FOREARM)){
                    if(dim == 10){
                        c->rmLink(9); c->rmLink(8); 
                        // we keep link 7 from elbow to wrist - it is getH(7) that is the FoR at the wrist in which forearm skin is expressed; and we want to keep the elbow joint part of the game
                        //TODO check with Ugo - he was doing    c1->rmLink(9); c1->rmLink(8); c1->rmLink(7); 
                        printMessage(4,"obstacle threatening skin part %s, blocking links 8 and 9 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                    }
                    else if(dim==7){
                        c->rmLink(6); c->rmLink(5); 
                        // we keep link 4 from elbow to wrist - it is getH(4) that is the FoR at the wrist in which forearm skin is expressed; and we want to keep the elbow joint part of the game
                        chain_local.blockLink(6); chain_local.blockLink(5);//wrist joints
                        printMessage(4,"obstacle threatening skin part %s, blocking links 5 and 6 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                   
                    }
                }
                else if (((*it).skin_part == SKIN_LEFT_UPPER_ARM) ||  ((*it).skin_part == SKIN_RIGHT_UPPER_ARM)){
                    if(dim == 10){
                        c->rmLink(9); c->rmLink(8);c->rmLink(7);c->rmLink(6); 
                        printMessage(4,"obstacle threatening skin part %s, blocking links 6-9 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                    }
                    else if(dim==7){
                        c->rmLink(6); c->rmLink(5);c->rmLink(4);c->rmLink(3); 
                        printMessage(4,"obstacle threatening skin part %s, blocking links 3-6 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                    }
                }
              
              
                // SetHN to move the end effector toward the point to be controlled - the average locus of collision threat from safety margin
                yarp::sig::Matrix HN = eye(4);
                computeFoR((*it).x,(*it).n,HN);
                printMessage(5,"HN matrix at collision point w.r.t. local frame: \n %s \n",HN.toString(3,3).c_str());
                chain_local.setHN(HN); //setting the end-effector transform to the collision point w.r.t subchain
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
                   
                chainCtrlPoints.push_back(c);
            }
        }
        type="none";
}
    
    
/****************************************************************/
AvoidanceHandlerAbstract::string getType() const
{
    return type;
}

/****************************************************************/
AvoidanceHandlerAbstract::virtual Property getParameters() const
{
    return parameters;
}

/****************************************************************/
virtual void AvoidanceHandlerAbstract::setParameters(const Property &parameters)
{
    this->parameters=parameters;
}

/****************************************************************/
void AvoidanceHandlerAbstract::updateCtrlPoints()
{
    // i>0: original chain (one to the end-effector) is up-to-date
    for (size_t i=1; i<chainCtrlPoints.size(); i++)
        for (size_t j=0; j<chainCtrlPoints[i]->getDOF(); j++)
            chainCtrlPoints[i]->setAng(j,chain(j).getAng()); //updating the local chains with current config of the orig chain - 
            //only for joint values in the local (possibly shorter) chain
}

/****************************************************************/
deque<Vector> AvoidanceHandlerAbstract::getCtrlPointsPosition()
{
    deque<Vector> ctrlPoints;
    for (size_t i=0; i<chainCtrlPoints.size(); i++)
        ctrlPoints.push_back(chainCtrlPoints[i]->EndEffPosition());
    return ctrlPoints;
}

/****************************************************************/
virtual Matrix AvoidanceHandlerAbstract::getVLIM(const Obstacle &obstacle, const Matrix &v_lim)
{
    return v_lim;
}

/****************************************************************/
virtual AvoidanceHandlerAbstract::~AvoidanceHandlerAbstract()
{
    // i>0: don't dispose original chain
    for (size_t i=1; i<chainCtrlPoints.size(); i++)
        delete chainCtrlPoints[i];
}


/****************************************************************/
AvoidanceHandlerTactile::AvoidanceHandlerTactile(iKinLimb &limb,const std::vector<collisionPoint_t> &_collisionPoints): AvoidanceHandlerAbstract(limb,_collisionPoints)
{
    type="tactile";

    avoidingSpeed = 50;
    // produce collisionPoint.magnitude * avoidingVelocity deg/s repulsive velocity  
        
    parameters.unput("avoidingVelocity");
    parameters.put("avoidingVelocity",avoidingVelocity);
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
        
        Matrix VLIM=v_lim;
        for (size_t i=0; i<chainCtrlPoints[i]->size(); i++)
        {
            Matrix J=chainCtrlPoints[i]->GeoJacobian().submatrix(0,2,0,chainCtrlPoints[i]->getDOF()-1);
            Vector normal = chainCtrlPoints[i]->getHN().getCol(2).subVector(0,2); //get the end-effector frame of the standard or custom chain (control point derived from skin), takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the 4 in the homogenous transf. format
            Vector s=(J.transposed()*normal) * avoidingSpeed * collisionPoints[i].magnitude; //project movement along the normal into joint velocity space and scale by default avoidingSpeed and magnitude of skin (or PPS) activation
            s = s * -1.0; //we reverse the direction to obtain joint velocities that bring about avoidance
            for (size_t j=0; j<s.length(); j++)
            {
                if (s[j]>=0.0) //joint contributes to avoidance, we will set the min velocity accordingly 
                {
                    s[j]=std::min(v_lim(j,1),s[j]); //make sure min vel is <= max vel
                    VLIM(j,0)=std::max(VLIM(j,0),s[j]); // set min vel to the s[j] ~ avoiding action
                    VLIM(j,1)=std::max(VLIM(j,0),VLIM(j,1)); //range check
                }
                else //joint acts to bring control point toward obstacle - we will shape the max vel
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

