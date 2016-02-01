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

#include <vector>
#include <deque>

#include <stdarg.h>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/common.h>


struct collisionPoint_t{
        iCub::skinDynLib::SkinPart skin_part;
        yarp::sig::Vector x; //position (x,y,z) in the FoR of the respective skin part
        yarp::sig::Vector n; //direction of normal vector at that point - derived from taxel normals, pointing out of the skin
        double magnitude; // ~ activation level from probabilistic representation in pps - likelihood of collision
};

/****************************************************************/
class AvoidanceHandlerAbstract
{


public:
    AvoidanceHandlerAbstract(const iCub::iKin::iKinChain &_chain, const std::vector<collisionPoint_t> &_collisionPoints, const unsigned int _verbosity=0);
    
    std::string getType() const;

    virtual yarp::os::Property getParameters() const;
    
    virtual void setParameters(const yarp::os::Property &parameters);
    
    std::deque<yarp::sig::Vector> getCtrlPointsPosition();
    
    virtual yarp::sig::Matrix getVLIM(const yarp::sig::Matrix &v_lim);
    
    virtual ~AvoidanceHandlerAbstract();
    
 protected:
    unsigned int verbosity;
    std::string type;
    iCub::iKin::iKinChain chain;
    const std::vector<collisionPoint_t> &collisionPoints;
    std::deque<iCub::iKin::iKinChain> ctrlPointChains;
    yarp::os::Property parameters;
    
    bool computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR);
    
    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;
    
};



/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandlerAbstract
{

public:
    AvoidanceHandlerTactile(const iCub::iKin::iKinChain &_chain,const std::vector<collisionPoint_t> &_collisionPoints,const unsigned int _verbosity=0);
    void setParameters(const yarp::os::Property &parameters);
    yarp::sig::Matrix getVLIM(const yarp::sig::Matrix &v_lim);

protected:
    double avoidingSpeed;

    
};


#endif