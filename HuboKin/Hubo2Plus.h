#ifndef HUBOKIN_HUBO2PLUS_H
#define HUBOKIN_HUBO2PLUS_H

#include "akin/Robot.h"

namespace HuboKin {

class Hubo2Plus : public akin::Robot
{
public:

    Hubo2Plus(const std::string& urdf_file,
              const std::string& urdf_package_directory,
              akin::Frame& referenceFrame = akin::Frame::World());

    Hubo2Plus(const std::string& urdf_string,
              akin::Frame& referenceFrame = akin::Frame::World());

protected:

    void _setup_manipulators();

};

} // namespace HuboKin

#endif // HUBOKIN_HUBO2PLUS_H
