
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
    addManipulator(joint("LWR").childLink(), "leftHandManip",
                   link("leftPalm").respectToRef());
    addManipulator(joint("RWR").childLink(), "rightHandManip",
                   link("rightPalm").respectToRef());
    addManipulator(joint("LAR").childLink(), "leftFootManip",
                   link("leftFoot").respectToRef());
    addManipulator(joint("RAR").childLink(), "rightFootManip",
                   link("rightFoot").respectToRef());

    joint(DOF_POS_Z).value(-link("leftFoot").withRespectTo(refFrame()).translation()[2]);
}


