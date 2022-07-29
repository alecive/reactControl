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

#include <iCub/iKin/iKinFwd.h>

#include "common.h"



/****************************************************************/
class AvoidanceHandlerAbstract
{


public:
    AvoidanceHandlerAbstract(iCub::iKin::iKinChain &_chain, const std::vector<collisionPoint_t> &_colPoints,
                             iCub::iKin::iKinChain* _secondChain, bool _useSelfColPoints, const std::string& _part,
                             yarp::sig::Vector* data, unsigned int _verbosity=0);
    
    std::string getType() const { return type; }

    virtual yarp::os::Property getParameters() const { return parameters; }
    
    virtual void setParameters(const yarp::os::Property &params) { parameters = params; }
    
    std::deque<yarp::sig::Vector> getCtrlPointsPosition();
    
    virtual yarp::sig::Matrix getVLIM(const yarp::sig::Matrix &v_lim, bool&  /*velLimited*/) { return v_lim; }
    
    virtual ~AvoidanceHandlerAbstract() { ctrlPointChains.clear(); }

    const std::vector<yarp::sig::Vector>& getSelfColPointsTorso() { return selfColPoints[3]; }

    void checkSelfCollisions();

protected:
    unsigned int verbosity;
    std::string type;
    std::string part;
    iCub::iKin::iKinChain& chain;
    iCub::iKin::iKinChain* secondChain;
    const std::vector<collisionPoint_t> &collisionPoints;
    std::deque<iCub::iKin::iKinChain> ctrlPointChains;
    std::vector<collisionPoint_t> totalColPoints;
    yarp::os::Property parameters;
    std::vector<std::vector<yarp::sig::Vector>> selfColPoints;
    std::vector<std::vector<yarp::sig::Vector>> selfControlPoints;

    static bool computeFoR(const yarp::sig::Vector &pos, const yarp::sig::Vector &norm, yarp::sig::Matrix &FoR);
    
    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(unsigned int l, const char *f, ...) const;

};



/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandlerAbstract
{

public:
    AvoidanceHandlerTactile(iCub::iKin::iKinChain &_chain, const std::vector<collisionPoint_t> &_colPoints,
                            iCub::iKin::iKinChain* _secondChain, bool _useSelfColPoints, const std::string& _part,
                            yarp::sig::Vector* data, unsigned int _verbosity=0);
    void setParameters(const yarp::os::Property &params) override;
    yarp::sig::Matrix getVLIM(const yarp::sig::Matrix &v_lim, bool& velLimited) override;

protected:
    double avoidingSpeed;


};


#endif