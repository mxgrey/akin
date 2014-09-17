#ifndef HUBOKIN_DRCHUBO_H
#define HUBOKIN_DRCHUBO_H

#include "Hubo2Plus.h"

namespace HuboKin {

class DrcHubo : public Hubo2Plus
{
public:

    enum {
        LEFT_PEG = RIGHT_FOOT+1,
        RIGHT_PEG
    };

    DrcHubo(const std::string& urdf_file,
            const std::string& urdf_package_directory,
            akin::Frame& referenceFrame = akin::Frame::World());

    DrcHubo(const std::string& urdf_string,
            akin::Frame& referenceFrame = akin::Frame::World());

protected:

    void _setup_pegs();

};

} // namespace HuboKin

#endif // HUBOKIN_DRCHUBO_H
