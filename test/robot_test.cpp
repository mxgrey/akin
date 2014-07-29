#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/MatrixTransform>

#include "../osgAkin/AkinCallback.h"
#include "../osgAkin/Axes.h"

#include "../akin/Robot.h"
#include "../akinUtils/urdfParsing.h"

#include "../akin/RobotConstraint.h"
#include "../akin/Solver.h"

using namespace akin;
using namespace std;
using namespace osgAkin;

class CustomNode : public AkinNode
{
public:

    inline CustomNode() : manip(NULL), solver(NULL), time(0) { }

    void setManipulator(Manipulator& new_manip)
    {
        solver->setMandatoryConstraint(&new_manip.constraint(Manipulator::LINKAGE));
        manip = &new_manip;
        config = getRobot(0).getConfig(manip->constraint(Manipulator::LINKAGE).getJoints());
    }

    size_t addRobot(Robot &new_robot)
    {
        size_t r = AkinNode::addRobot(new_robot);

        if(solver == NULL)
        {
            solver = new RobotSolverX(new_robot);
            solver->max_steps = 10;
        }

        return r;
    }

    virtual void update()
    {
        time += 0.01;
    
        Robot& robot = getRobot(0);
        robot.joint("NK2").value( 45*M_PI/180 * sin(time) );

        if(solver != NULL && manip != NULL)
        {
            tf = manip->constraint(Manipulator::LINKAGE).target.respectToRef();
            tf.translate( Vec3(0.5,-0.5,0) * 0.2*sin(time)*0.01 );
            manip->constraint(Manipulator::LINKAGE).target.respectToRef(tf);
            solver->solve(config);
        }
    
        AkinNode::update();
    }

protected:

    Transform tf;
    Eigen::VectorXd config;
    Manipulator* manip;
    RobotSolverX* solver;
    double time;
};

Robot& build_manual_robot()
{
    Robot* rb_ptr = new Robot;
    Robot& robot = *rb_ptr;
    
    robot.name("test_bot");
    
//    if(!robot.createRootLink("root_link"))
//    {
//        cout << "Wtf failed to create root link" << endl;
//        return robot;
//    }
    cout << "Size of links: " << robot.numLinks() << endl;
    
    int newID = robot.createJointLinkPair(0, "first_link", "first_joint", Transform(Translation(1,0,0)),
                                          Axis(1,0,0), Joint::REVOLUTE, -M_PI, M_PI);
    
    cout << "created first pair" << endl;
    
    cout << robot.link(0) << endl;
    cout << robot.link(1) << endl;
    
    if(newID <= 0)
    {
        cout << "Wtf invalid creation attempt" << endl;
        return robot;
    }
    
    newID = robot.createJointLinkPair(newID, "second_link", "second_joint", Transform(Translation(0,0,1)),
                                      Axis(0,0,1), Joint::PRISMATIC, -10, 10);
    
    robot.joint(0).value(180*M_PI/180);
    robot.joint(1).value(2);
    
    cout << robot.link(2) << endl;
    
    return robot;
}

Robot& build_urdf_robot()
{
    Robot* rb_ptr = new Robot(akin::Frame::World(), verbosity::DEBUG);
    akinUtils::loadURDF(*rb_ptr, "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                        "../../../resources/drchubo");
    Robot& robot = *rb_ptr;

//    for(size_t i=0; i<robot.numLinks(); ++i)
//        if(robot.link(i).peekVisual(0).type == Geometry::MESH_FILE)
//            std::cout << "Mesh file for Link '" << robot.link(i).name()
//                  << "': " << robot.link(i).peekVisual(0).mesh_filename << std::endl;
    
    return robot;
}

void display_robot(Robot& displaying_robot)
{
    osg::Group* root = new osg::Group;
    CustomNode* akinNode = new CustomNode;
    root->addChild(akinNode);

    akinNode->addRobot(displaying_robot);
    akinNode->addRootFrame(akin::Frame::World());
    
    
    
    Robot& r = displaying_robot;

    r.joint(DOF_POS_Z).value(-r.link("leftFoot").respectToWorld().translation()[2]);
    
////    r.joint(DOF_ROT_Z).value(90*M_PI/180);
////    r.joint(DOF_ROT_Y).value(90*M_PI/180);
////    r.joint(DOF_ROT_X).value(90*M_PI/180);
////    r.joint(DOF_POS_X).value(1);
////    r.joint(DOF_POS_Y).value(0.5);
////    r.joint(DOF_POS_Z).value(1);
//    r.joint(DOF_ROT_X).value(90*DEG);
////    r.joint(DOF_ROT_Y).value(45*DEG);
    
    
//    r.joint("LSP").value(90*DEG);
//    r.joint("LSY").value(-90*DEG);
////    r.joint("LEP").value(90*DEG);
    
    
//    Eigen::Matrix<double, 6, 14> J;
//    std::vector<std::string> joints;
//    joints.push_back("LSP");
//    joints.push_back("LSR");
//    joints.push_back("LSY");
//    joints.push_back("LEP");
//    joints.push_back("LWY");
//    joints.push_back("LWP");
//    joints.push_back("LWR");
//    joints.push_back("DOF_POS_X");
//    joints.push_back("DOF_POS_Y");
//    joints.push_back("DOF_POS_Z");
//    joints.push_back("DOF_ROT_X");
//    joints.push_back("DOF_ROT_Y");
//    joints.push_back("DOF_ROT_Z");
//    joints.push_back("RSP");
    
//    KinTranslation manip(Translation(),r.link("leftPalm"));
//    std::cout << "Robot:" << std::endl;
//    for(size_t i=0; i<joints.size(); ++i)
//    {
//        J.block<6,1>(0, i) = r.joint(joints[i]).Jacobian(manip, r.frame());
//    }
    
//    std::cout << "wrt Robot:\n";
//    std::cout << J << std::endl;
    
//    std::cout << "World:" << std::endl;
//    for(size_t i=0; i<joints.size(); ++i)
//    {
//        J.block<6,1>(0, i) = r.joint(joints[i]).Jacobian(manip, Frame::World());
//    }
    
//    std::cout << "\nwrt World:\n";
//    std::cout << J << std::endl;
    
    
//    std::vector<std::string> joint_names;
//    joint_names.push_back("LSP");
//    joint_names.push_back("LSR");
//    joint_names.push_back("LSY");
//    joint_names.push_back("LEP");
//    joint_names.push_back("LWY");
//    joint_names.push_back("LWP");
//    joint_names.push_back("LWR");
    
//    std::vector<size_t> joints;
//    for(size_t i=0; i<joint_names.size(); ++i)
//        joints.push_back(r.joint(joint_names[i]).id());
    
    std::vector<size_t> joints = Robot::Explorer::getIdPath(r.joint("LSP"),r.joint("LWR"));
//    std::vector<size_t> joints = Robot::Explorer::getIdPath(r.joint("LWR"),r.joint("LSP"));
    for(size_t i=0; i<joints.size(); ++i)
        std::cout << r.joint(joints[i]).name() << std::endl;
    
    int m = r.addManipulator(r.link("leftPalm"), "leftHandManip");
    if(m < 0)
        std::cout << "something went wrong: " << m << std::endl;
    else
    {
//        r.manip(m).setConstraint(new ManipConstraint<7>(r.manip(m),joints));
        r.manip(m).setConstraint(Manipulator::LINKAGE, new ManipConstraintX(7, r.manip(m), joints));
    }
//    ManipConstraintX* mptr = new ManipConstraintX(7, r.manip(m), joints);
//    delete mptr;

    r.joint("LEP").value(-90*DEG);
    r.manip(m).constraint(Manipulator::LINKAGE).target.respectToRef(r.manip(m).respectToWorld());
    Transform tf = r.manip(m).constraint(Manipulator::LINKAGE).target.respectToRef();
    tf.translate(Vec3(0,0.2,0.2));
    r.manip(m).constraint(Manipulator::LINKAGE).target.respectToRef(tf);

    akinNode->setManipulator(r.manip(m));

    Eigen::VectorXd config = r.getConfig(joints);
    RobotSolverX solver(r);
    solver.setMandatoryConstraint(&r.manip(m).constraint(Manipulator::LINKAGE));
    if(solver.solve(config))
    {
        std::cout << "Solved!" << std::endl;

        std::cout << config.transpose() << std::endl;
    }
    else
    {
        std::cout << "Failed!" << std::endl;

        std::cout << config.transpose() << std::endl;
        return;
    }

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3f,0.3f,0.3f,1.0f));
    viewer.setSceneData(root);
    viewer.run();
}


int main(int , char* [])
{
//    Robot& built_robot = build_manual_robot();
    
    Robot& built_robot = build_urdf_robot();

    new Frame(akin::Transform(Translation(0,0,0),
                              Rotation(90*M_PI/180, akin::Axis(0, 1, 0))),
                              built_robot.joint("NK2").childLink(),
                              "camera_frame");

    
    display_robot(built_robot);
}
