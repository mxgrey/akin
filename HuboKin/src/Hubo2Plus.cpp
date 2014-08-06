
#include "../Hubo2Plus.h"
#include "akin/AnalyticalIKBase.h"
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

    // Left Hand Constraints
    manip(MANIP_L_HAND).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<7>(manip(MANIP_L_HAND),
                        Robot::Explorer::getIdPath(joint("LSP"),joint("LWR"))));

    // Right Hand Constraints
    manip(MANIP_R_HAND).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<7>(manip(MANIP_R_HAND),
                        Robot::Explorer::getIdPath(joint("RSP"),joint("RWR"))));

    // Left Foot Constraints
    manip(MANIP_L_FOOT).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<6>(manip(MANIP_L_FOOT),
                        Robot::Explorer::getIdPath(joint("LHY"),joint("LAR"))));

    AnalyticalBallSocketHip<6>* fc = new AnalyticalBallSocketHip<6>(manip(MANIP_L_FOOT),
                        Robot::Explorer::getIdPath(joint("LHY"),joint("LAR")));
    fc->hipRotation.rotate(Rotation(90*DEG, Vec3(0,0,1)));
    fc->footRotation.rotate(Rotation(-90*DEG, Vec3(0,1,0)));
    manip(MANIP_L_FOOT).setConstraint(Manipulator::ANALYTICAL, fc);

    // Right Foot Constraints
    manip(MANIP_R_FOOT).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<6>(manip(MANIP_R_FOOT),
                        Robot::Explorer::getIdPath(joint("RHY"),joint("RAR"))));

    fc = new AnalyticalBallSocketHip<6>(manip(MANIP_R_FOOT),
                        Robot::Explorer::getIdPath(joint("RHY"),joint("RAR")));
    fc->hipRotation.rotate(Rotation(90*DEG, Vec3(0,0,1)));
    fc->hipRotation.rotate(Rotation(180*DEG, Vec3(1,0,0)));
    fc->footRotation.rotate(Rotation(90*DEG, Vec3(0,1,0)));
    manip(MANIP_R_FOOT).setConstraint(Manipulator::ANALYTICAL, fc);
}


