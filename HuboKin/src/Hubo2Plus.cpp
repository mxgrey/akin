
#include "../Hubo2Plus.h"
#include "akinUtils/urdfParsing.h"

using namespace akin;
using namespace HuboKin;

Hubo2Plus::Hubo2Plus(const std::string &urdf_file,
                     const std::string &urdf_package_directory,
                     Frame& referenceFrame) :
    Robot(referenceFrame)
{
    akinUtils::loadURDF(*this, urdf_file, urdf_package_directory);
    _setup_manipulators();
}

Hubo2Plus::Hubo2Plus(const std::string &urdf_string, Frame &referenceFrame) :
    Robot(referenceFrame)
{
    akinUtils::loadURDFstring(*this, urdf_string);
    _setup_manipulators();
}

void Hubo2Plus::_setup_manipulators()
{

}


