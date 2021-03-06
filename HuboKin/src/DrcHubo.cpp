
#include "../DrcHubo.h"
#include "akin/AnalyticalIKBase.h"

using namespace akin;
using namespace HuboKin;

DrcHubo::DrcHubo(const std::string &urdf_file,
                 const std::string &urdf_package_directory,
                 Frame &referenceFrame) :
    Hubo2Plus(urdf_file, urdf_package_directory, referenceFrame)
{
    _setup_pegs();
}

DrcHubo::DrcHubo(const std::string &urdf_string, Frame &referenceFrame) :
    Hubo2Plus(urdf_string, referenceFrame)
{
    _setup_pegs();
}

void DrcHubo::_setup_pegs()
{
    Transform leftTf = link("leftPalm").withRespectTo(joint("LWR").parentLink());
    leftTf.translate(Vec3(0,0.21,0));
    addManipulator(joint("LWR").parentLink(), "leftPeg", leftTf);
    Transform rightTf = link("rightPalm").withRespectTo(joint("RWR").parentLink());
    rightTf.translate(Vec3(0,0.21,0));
    addManipulator(joint("RWR").parentLink(), "rightPeg", rightTf);

    // Left Peg Constraints
    manip(LEFT_PEG).setConstraint(Manipulator::LINKAGE,
                new ManipConstraint<6>(manip(LEFT_PEG),
                            Robot::Explorer::getDofIds(joint("LSP"),joint("LWP"))));

    // Right Peg Constraints
    manip(RIGHT_PEG).setConstraint(Manipulator::LINKAGE,
                new ManipConstraint<6>(manip(RIGHT_PEG),
                            Robot::Explorer::getDofIds(joint("RSP"),joint("RWP"))));
}
