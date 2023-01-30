//
// Created by Jakub Rozlivek on 7/28/21.
//

#ifndef COMMON_H
#define COMMON_H
#define M2MM 1000
#define DURATION 1

#include <sstream>
#include <iCub/skinDynLib/common.h>
#include <cstdarg>

enum {
    TACTILE_OBS,
    VISUAL_OBS,
    PROX_OBS,
    SELFCOL_OBS
};


struct collisionPoint_t{
    iCub::skinDynLib::SkinPart skin_part;
    yarp::sig::Vector x; //position (x,y,z) in the FoR of the respective skin part
    yarp::sig::Vector n; //direction of normal vector at that point - derived from taxel normals, pointing out of the skin
    double magnitude; // ~ activation level from probabilistic representation in pps - likelihood of collision
    double duration;  // how long will be collision point activated when no update come
    int type;

    explicit collisionPoint_t(int typ): magnitude(1), skin_part(iCub::skinDynLib::SKIN_PART_UNKNOWN), duration(DURATION), type(typ)
    {
        x.resize(3);
        n.resize(3);
    }
    collisionPoint_t(iCub::skinDynLib::SkinPart _skinPart, std::vector<double> pos, int typ, double mag = 1):
            skin_part(_skinPart), magnitude(mag), duration(DURATION), type(typ)
    {
        x.resize(3);
        x = {pos[0], pos[1], pos[2]};
        n.resize(3);
    }

    explicit collisionPoint_t(iCub::skinDynLib::SkinPart _skinPart, int typ,  double mag=1):
            skin_part(_skinPart), magnitude(mag), duration(DURATION), type(typ)
    {
        x.resize(3);
        n.resize(3);
    }

    void reset(double mag=1)
    {
        magnitude = mag;
        duration = DURATION;
    }

};

#endif //COMMON_H
