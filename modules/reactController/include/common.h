//
// Created by Jakub Rozlivek on 7/28/21.
//

#ifndef COMMON_H
#define COMMON_H

#include <sstream>
#include <iCub/skinDynLib/common.h>
#include <cstdarg>

struct collisionPoint_t{
    iCub::skinDynLib::SkinPart skin_part;
    yarp::sig::Vector x; //position (x,y,z) in the FoR of the respective skin part
    yarp::sig::Vector n; //direction of normal vector at that point - derived from taxel normals, pointing out of the skin
    double magnitude; // ~ activation level from probabilistic representation in pps - likelihood of collision

    collisionPoint_t(): magnitude(1), skin_part(iCub::skinDynLib::SKIN_PART_UNKNOWN)
    {
        x.resize(3);
        n.resize(3);
    }
    collisionPoint_t(iCub::skinDynLib::SkinPart _skinPart, std::vector<double> pos, double mag = 1): skin_part(_skinPart), magnitude(mag)
    {
        x.resize(3);
        x = {pos[0], pos[1], pos[2]};
        n.resize(3);
    }

    explicit collisionPoint_t(iCub::skinDynLib::SkinPart _skinPart, double mag=1): skin_part(_skinPart), magnitude(mag)
    {
        x.resize(3);
        n.resize(3);
    }

};


class ControlPoint
{
public:
    std::string type; //e.g. "elbow"
    yarp::sig::Vector x_desired; //desired Cartesian position (x,y,z) in Root FoR
    yarp::sig::Vector p0; //position of the control point depending on current state of chain
    yarp::sig::Matrix J0_xyz; //Jacobian for position depending on current state of chain

    ControlPoint()
    {
        x_desired.resize(3); x_desired.zero();
        x_desired(0)=-0.2; //just to have it iCub Root FoR friendly
        p0.resize(3); p0.zero();
        p0(0) = -0.1;
        //for J0_xyz we don't know the size yet - depending on the control point
    }

    std::string toString() const
    {
        std::stringstream sstm;
        sstm<< "ControlPoint, type: "<<type<<", x_desired: ("<<x_desired.toString(3,3)<<"), p0: ("<<p0.toString(3,3)<<"), J0_xyz: "<<std::endl<<
            J0_xyz.toString(3,3)<<std::endl;

        return sstm.str();
    }
};


#endif //COMMON_H
