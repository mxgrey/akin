
#include "../Hubo2Plus.h"
#include "../HuboLegIK.h"
#include "urdfAkin/urdfParsing.h"

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
    dof("LKP").min(0);
    dof("RKP").min(0);
    
    addManipulator(joint("LWR").childLink(), "leftHandManip",
                   link("leftPalm").respectToRef());
    addManipulator(joint("RWR").childLink(), "rightHandManip",
                   link("rightPalm").respectToRef());
    addManipulator(joint("LAR").childLink(), "leftFootManip",
                   link("leftFoot").respectToRef());
    addManipulator(joint("RAR").childLink(), "rightFootManip",
                   link("rightFoot").respectToRef());

    dof(DOF_POS_Z).position(-link("leftFoot").withRespectTo(refFrame()).translation()[2]);


    // TODO: Move these to the DrcHubo clas, since Hubo2Plus has 6 DOF arms!
    // Left Hand Constraints
    std::vector<size_t> ids = Robot::Explorer::getDofIds(joint("LSP"),joint("LWR"));
    for(size_t i=0; i<ids.size(); ++i)
        std::cout << dof(ids[i]).name() << std::endl;

    std::cout << "\n";
    std::vector<size_t> joints = Robot::Explorer::getIdPath(joint("LSP"),joint("LWR"));
    for(size_t i=0; i<joints.size(); ++i)
        std::cout << joint(joints[i]).name() << std::endl;

    manip(LEFT_HAND).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<7>(manip(LEFT_HAND),
                        Robot::Explorer::getDofIds(joint("LSP"),joint("LWR"))));

    // Right Hand Constraints
    manip(RIGHT_HAND).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<7>(manip(RIGHT_HAND),
                        Robot::Explorer::getDofIds(joint("RSP"),joint("RWR"))));

    // Left Foot Constraints
    manip(LEFT_FOOT).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<6>(manip(LEFT_FOOT),
                        Robot::Explorer::getDofIds(joint("LHY"),joint("LAR"))));

    manip(LEFT_FOOT).setConstraint(Manipulator::ANALYTICAL, 
            new HuboLegIK<6>(manip(LEFT_FOOT),
                        Robot::Explorer::getDofIds(joint("LHY"),joint("LAR"))));
    
    manip(LEFT_FOOT).setConstraint(Manipulator::SUPPORT, 
            new HuboLegIK<6>(manip(LEFT_FOOT),
                        Robot::Explorer::getDofIds(joint("LHY"),joint("LAR"))));
    

    // Right Foot Constraints
    manip(RIGHT_FOOT).setConstraint(Manipulator::LINKAGE,
            new ManipConstraint<6>(manip(RIGHT_FOOT),
                        Robot::Explorer::getDofIds(joint("RHY"),joint("RAR"))));

    manip(RIGHT_FOOT).setConstraint(Manipulator::ANALYTICAL, 
            new HuboLegIK<6>(manip(RIGHT_FOOT),
                        Robot::Explorer::getDofIds(joint("RHY"),joint("RAR"))));
    
    manip(RIGHT_FOOT).setConstraint(Manipulator::SUPPORT, 
            new HuboLegIK<6>(manip(RIGHT_FOOT),
                        Robot::Explorer::getDofIds(joint("RHY"),joint("RAR"))));
    
    for(size_t i=0; i<2; ++i)
    {
        size_t m = i==0? LEFT_FOOT : RIGHT_FOOT;
        std::string name = i==0? "lfoot" : "rfoot";
        
        std::vector<KinTranslation>& sg = manip(m).supportGeometry;
        sg.push_back(KinTranslation(Translation( 0.02, 0.02,0), manip(m), name+"_FL_support"));
        sg.push_back(KinTranslation(Translation(-0.02, 0.02,0), manip(m), name+"_BL_support"));
        sg.push_back(KinTranslation(Translation(-0.02,-0.02,0), manip(m), name+"_BR_support"));
        sg.push_back(KinTranslation(Translation( 0.02,-0.02,0), manip(m), name+"_FR_support"));
    }
}


