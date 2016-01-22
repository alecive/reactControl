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


#ifndef __AVOIDANCEHANDLER_H__
#define __AVOIDANCEHANDLER_H__

#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <vector>

struct collisionPoint_t{
        iCub::skinDynLib::SkinPart skin_part;
        yarp::sig::Vector x; //position (x,y,z) in the FoR of the respective skin part
        yarp::sig::Vector n; //direction of normal vector at that point - derived from taxel normals, pointing out of the skin
        double magnitude; // ~ activation level from probabilistic representation in pps - likelihood of collision
};

/****************************************************************/
class AvoidanceHandlerAbstract
{
protected:
    string type;
    iKinChain &chain;
    std::vector<collisionPoint_t> &collisionPoints;
    deque<iKinChain*> chainCtrlPoints;
    Property parameters;

public:
    AvoidanceHandlerAbstract(iKinLimb &limb, const std::vector<collisionPoint_t> &_collisionPoints);
    
    string getType() const;

    virtual Property getParameters() const;
    
    virtual void setParameters(const Property &parameters);
    
    void updateCtrlPoints();
    
    deque<Vector> getCtrlPointsPosition();
    
    virtual Matrix getVLIM(const Matrix &v_lim);
    
    virtual ~AvoidanceHandlerAbstract();
};



/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandlerAbstract
{
protected:
    double avoidingVelocity;

public:
    AvoidanceHandlerTactile(iKinLimb &limb,const std::vector<collisionPoint_t> &_collisionPoints);
    void setParameters(const Property &parameters);
    Matrix getVLIM(const Matrix &v_lim);
   
};


#endif