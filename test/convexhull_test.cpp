
#include "HuboKin/DrcHubo.h"
#include "akin/RobotConstraint.h"

using namespace akin;
using namespace HuboKin;

int main(int , char* [])
{
    // TODO: Make a visualization for support polygons
    
    DrcHubo hubo("../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                 "../../../resources/drchubo");
    
    size_t left = DrcHubo::LEFT_FOOT;
    size_t right = DrcHubo::RIGHT_FOOT;
    
    hubo.manip(left).mode = Manipulator::SUPPORT;
    hubo.manip(right).mode = Manipulator::SUPPORT;
    
//    std::vector<size_t> joints = hubo.manip(right).constraint().getJoints();
//    Eigen::VectorXd config = hubo.getConfig(joints);
//    Transform tf = hubo.manip(right).respectToWorld();
//    tf.pretranslate(Translation(0.05, -0.05, 0.1));
//    tf.rotate(Rotation(-45*DEG, Vec3(0,0,1)));
//    hubo.manip(right).ik(config, tf);
//    hubo.setConfig(joints, config);
    
    for(size_t i=0; i<hubo.manip(left).supportGeometry.size(); ++i)
        std::cout << hubo.manip(left).supportGeometry[i].respectToWorld().transpose() << std::endl;
    
    for(size_t i=0; i<hubo.manip(right).supportGeometry.size(); ++i)
        std::cout << hubo.manip(right).supportGeometry[i].respectToWorld().transpose() << std::endl;
    
    std::cout << "\nHull:" << std::endl;
    const std::vector<Eigen::Vector2d>& hull = hubo.getSupportPolygon();
    for(size_t i=0; i<hull.size(); ++i)
        std::cout << hull[i].transpose() << std::endl;
    
    std::cout << "\nCenter: " << hubo.getSupportCenter().transpose() << std::endl;
    
    
    hubo.dof(DOF_POS_X).value(1);
    std::cout << "\nMoved forward" << std::endl;
    
    const std::vector<Eigen::Vector2d>& newHull = hubo.getSupportPolygon();
    for(size_t i=0; i<newHull.size(); ++i)
        std::cout << newHull[i].transpose() << std::endl;
    
    hubo.dof(DOF_POS_Y).value(1);
    std::cout << "\nMoved left" << std::endl;
    for(size_t i=0; i<hubo.getSupportPolygon().size(); ++i)
        std::cout << hubo.getSupportPolygon()[i].transpose() << std::endl;
    
    return 0;
}
