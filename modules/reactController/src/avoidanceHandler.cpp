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

AvoidanceHandlerAbstract::AvoidanceHandlerAbstract(const iCub::iKin::iKinChain &_chain, const std::vector<collisionPoint_t> &_collisionPoints,
                                                   bool _useSelfColPoints, const unsigned int _verbosity):
                                                   chain(_chain), collisionPoints(_collisionPoints), verbosity(_verbosity), type("none")
{
    selfColPoints = {};
    if (_useSelfColPoints)
    {
        // front chest, back, face, back of head, ears (2x), hip (3x), front chest low band
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
                    Vector pos(4);
                    selfColPoints.push_back(pos);
                    selfColPoints.back() = {posx[l][j], posy[l][i], posz[l][j], 1}; //homogenous coordinates
                }
            }
        }

        selfControlPoints.resize(2);
        selfControlPoints[0].push_back({-0.0049, 0.0012,  0.0});
        selfControlPoints[0].push_back({-0.0059, 0.0162,  0.0});
        selfControlPoints[0].push_back({-0.0199, 0.0122,  0.0});
        selfControlPoints[0].push_back({-0.0049, -0.0143,  0.0});
        selfControlPoints[0].push_back({-0.0304, 0.0122,  0.0});
        selfControlPoints[1].push_back({-0.0323, -0.0642, 0.0178});
        selfControlPoints[1].push_back({-0.0310, -0.0794, 0.0278});
        selfControlPoints[1].push_back({-0.0335, -0.0964, 0.0192});
        selfControlPoints[1].push_back({0.0286, -0.0641, 0.0179});
        selfControlPoints[1].push_back({0.0258, -0.0474, 0.0251});
        selfControlPoints[1].push_back({0.0157, -0.0460, 0.0401});
        selfControlPoints[1].push_back({0.0273, -0.0793, 0.0278});
        selfControlPoints[1].push_back({0.0296, -0.0964, 0.0192});
        selfControlPoints[1].push_back({0.0075, -0.0952, 0.0463});
        selfControlPoints[1].push_back({0.0167, -0.0789, 0.0425});
        selfControlPoints[1].push_back({0.0075, -0.0624, 0.0450});
        selfControlPoints[1].push_back({-0.0112, -0.0624, 0.0450});
        selfControlPoints[1].push_back({-0.0204, -0.0789, 0.0425});
        selfControlPoints[1].push_back({-0.0112, -0.0952, 0.0463});
        selfControlPoints[1].push_back({-0.0194, -0.0460, 0.0400});
        selfControlPoints[1].push_back({-0.0293, -0.0475, 0.0248});
        selfControlPoints[1].push_back({-0.0327, -0.0645, -0.0101});
        selfControlPoints[1].push_back({-0.0299, -0.0461, -0.0152});
        selfControlPoints[1].push_back({0.0290, -0.0643, -0.0101});
        selfControlPoints[1].push_back({0.0264, -0.0460, -0.0154});
        selfControlPoints[1].push_back({0.0074, -0.0258, -0.0302});
        selfControlPoints[1].push_back({0.0159, -0.0429, -0.0300});
        selfControlPoints[1].push_back({-0.0196, -0.0430, -0.0299});
        selfControlPoints[1].push_back({-0.0112, -0.0259, -0.0301});
    }
}


/****************************************************************/
deque<Vector> AvoidanceHandlerAbstract::getCtrlPointsPosition()
{
    deque<Vector> ctrlPoints;
    for (auto & ctrlPointChain : ctrlPointChains)
    {
        ctrlPoints.push_back(ctrlPointChain.EndEffPosition());
    }
    return ctrlPoints;
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

void AvoidanceHandlerAbstract::checkSelfCollisions()
{
    std::vector<Matrix> transforms;
    transforms.push_back(yarp::math::SE3inv(chain.getH(SkinPart_2_LinkNum[SKIN_LEFT_HAND].linkNum + 3))*chain.getH(2));
    transforms.push_back(yarp::math::SE3inv(chain.getH(SkinPart_2_LinkNum[SKIN_LEFT_FOREARM].linkNum + 3))*chain.getH(2));
    int index = 0;
    for (int k = 0; k < 2; ++k)
    {
        for (const auto &colPoint: selfColPoints)
        {
            Vector pos = transforms[k] * colPoint;
            int nearest = -1;
            double neardist = 10000;
            for (int i = 0; i < selfControlPoints[k].size(); i++)
            {
                double n = yarp::math::norm2(pos.subVector(0, 2) - selfControlPoints[k][i]);
                if (n < neardist)
                {
                    nearest = i;
                    neardist = n;
                }
            }
            if (neardist < 0.0025)  // distance lower than 0.05 m
            {
                totalColPoints.emplace_back();
                collisionPoint_t cp{(k == 0)? SKIN_LEFT_HAND : SKIN_LEFT_FOREARM, (1 - neardist * 100)};
                cp.x = selfControlPoints[k][nearest];
                Vector n = pos.subVector(0, 2) - selfControlPoints[k][nearest];
                cp.n = n / yarp::math::norm(n);
                totalColPoints.push_back(cp);
                printf("colPoint %d with pos = %s\n", index, cp.x.toString().c_str());
            }
            index++;
        }
    }
}


/****************************************************************/
AvoidanceHandlerTactile::AvoidanceHandlerTactile(const iCub::iKin::iKinChain &_chain,const std::vector<collisionPoint_t> &_collisionPoints,
                                                 bool _useSelfColPoints, const unsigned int _verbosity):
                                                 AvoidanceHandlerAbstract(_chain,_collisionPoints, _useSelfColPoints, _verbosity)
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
        avoidingSpeed=parameters.find("avoidingSpeed").asFloat64();
        this->parameters.unput("avoidingSpeed");
        this->parameters.put("avoidingSpeed",avoidingSpeed);
    }
}

/****************************************************************/
Matrix AvoidanceHandlerTactile::getVLIM(const Matrix &v_lim, Vector& weighted_normal)
{
    printMessage(2,"AvoidanceHandlerTactile::getVLIM\n");
    Matrix VLIM=v_lim;
    int dim = static_cast<int>(chain.getDOF());
    int i = 0;
    int dim_offset = dim-7;  // 3 if dim == 10; 0 if dim == 7
    ctrlPointChains.clear();
    totalColPoints = collisionPoints;
    checkSelfCollisions();
    for(const auto & colPoint : totalColPoints)
    {
        iKinChain customChain= chain; //instantiates a new chain, copying from the old (full) one
        if (verbosity >= 5)
        {
            printf("Full chain has %d DOF \n",dim);
            printf("chain.getH() (end-effector): \n %s \n",chain.getH().toString(3,3).c_str());
            printf("SkinPart %s, linkNum %d, chain.getH() (skin part frame): \n %s \n", SkinPart_s[colPoint.skin_part].c_str(), SkinPart_2_LinkNum[colPoint.skin_part].linkNum + dim_offset , chain.getH(SkinPart_2_LinkNum[colPoint.skin_part].linkNum + dim_offset).toString(3, 3).c_str());
        }

        // Remove all the more distal links after the collision point
        // if the skin part is a hand, no need to remove any links from the chain
        if ((colPoint.skin_part == SKIN_LEFT_FOREARM) || (colPoint.skin_part == SKIN_RIGHT_FOREARM))
        {
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset);
            // we keep link 4(+3) from elbow to wrist - it is getH(4(+3)) that is the FoR at the wrist in which forearm skin is expressed; and we want to keep the elbow joint part of the game
            printMessage(2,"obstacle threatening skin part %s, blocking links 5(+3) and 6(+3) on subchain for avoidance\n",SkinPart_s[colPoint.skin_part].c_str());
        }
        else if ((colPoint.skin_part == SKIN_LEFT_UPPER_ARM) || (colPoint.skin_part == SKIN_RIGHT_UPPER_ARM))
        {
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset);
            customChain.rmLink(4+dim_offset); customChain.rmLink(3+dim_offset);
            printMessage(2,"obstacle threatening skin part %s, blocking links 3(+3)-6(+3) on subchain for avoidance\n",SkinPart_s[colPoint.skin_part].c_str());
        }
        else if (colPoint.skin_part == SKIN_FRONT_TORSO)
        {
            customChain.rmLink(6+dim_offset); customChain.rmLink(5+dim_offset); customChain.rmLink(4+dim_offset);
            customChain.rmLink(3+dim_offset); customChain.rmLink(2+dim_offset); customChain.rmLink(1+dim_offset);
            customChain.rmLink(0+dim_offset);
            printMessage(2,"obstacle threatening skin part %s, blocking links 0(+3)-6(+3) on subchain for avoidance\n",SkinPart_s[colPoint.skin_part].c_str());
        }

        // SetHN to move the end effector toward the point to be controlled - the average locus of collision threat from safety margin
        yarp::sig::Matrix HN = eye(4);
        computeFoR(colPoint.x, colPoint.n, HN);
        printMessage(5,"HN matrix at collision point w.r.t. local frame: \n %s \n",HN.toString(3,3).c_str());
        customChain.setHN(HN); //setting the end-effector transform to the collision point w.r.t subchain
        if (verbosity >=5)
        {
            yarp::sig::Matrix H = customChain.getH();
            printf("H matrix at collision point w.r.t. root: \n %s \n",H.toString(3,3).c_str());
        }

        printMessage(2,"Chain with control point - index %d (last index %d), nDOF: %d.\n",i,collisionPoints.size()-1,customChain.getDOF());
        Matrix J=customChain.GeoJacobian().submatrix(0,2,0,customChain.getDOF()-1); //first 3 rows ~ dPosition/dJoints
        Vector normal = customChain.getH().getCol(2).subVector(0,2); //get the end-effector frame of the standard or custom chain (control point derived from skin), takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the 4 in the homogenous transf. format
        weighted_normal += (-colPoint.magnitude*normal);
        Vector s=(J.transposed()*normal) * avoidingSpeed * colPoint.magnitude; //project movement along the normal into joint velocity space and scale by default avoidingSpeed and magnitude of skin (or PPS) activation
        if (verbosity>=2)
        {
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







 

