#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/MatrixTransform>

#include "osgAkin/AkinCallback.h"
#include "osgAkin/Axes.h"

#include "../akin/Robot.h"
#include "urdfAkin/urdfParsing.h"

#include "../akin/RobotConstraint.h"
#include "../akin/AnalyticalIKBase.h"
#include "../akin/Solver.h"

using namespace akin;
using namespace std;
using namespace osgAkin;

class CustomNode : public AkinNode
{
public:

    inline CustomNode() : mode(Manipulator::FREE), manip(NULL), solver(NULL), time(0) { }

    Manipulator::Mode mode;

    void setManipulator(Manipulator& new_manip)
    {
        solver->setMandatoryConstraint(new_manip.constraint(mode));
        manip = &new_manip;
        config = getRobot(0).getConfig(manip->constraint(mode)->getDofs());
    }

    size_t addRobot(Robot &new_robot)
    {
        size_t r = AkinNode::addRobot(new_robot);

        if(solver == NULL)
        {
            solver = new RobotSolverX(new_robot);
//            solver->max_steps = 10;
            solver->max_steps = 1;
            solver->step_size = 0.1;
        }

        return r;
    }

    virtual void update()
    {
        time += 0.01;
    
        Robot& robot = getRobot(0);
        robot.dof("NK2").position( 45*M_PI/180 * sin(time) );

        if( manip != NULL )
        {
            tf = manip->constraint(mode)->target.respectToRef();
            tf.pretranslate( Vec3(0.5,0.5,0) * 0.2*sin(time)*0.01 );
            tf.rotate(Rotation(-90*DEG*sin(time)/2.0*0.01, Axis(1,0,0)));
            manip->ik(config, tf, manip->constraint(mode)->target.refFrame());
        }

//        if(solver != NULL && manip != NULL)
//        {
//            solver->solve(config);
////            tf = manip->constraint(mode).target.respectToRef();
////            tf.pretranslate( Vec3(0.5,0.5,0) * 0.2*sin(time)*0.01 );
////            tf.rotate(Rotation(-90*DEG*sin(time)/2.0*0.01, Axis(1,0,0)));
////            tf.rotate(Rotation(90*DEG*sin(time)/2.0*0.01, Axis(0,1,0)));
////            manip->constraint(mode).target = tf;
////            solver->solve(config);

//        }
    
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
    
    int newID = robot.createJointLinkPair(robot.link(0), "first_link",
                                          ProtectedJointProperties("first_joint", Joint::REVOLUTE,
                                          Transform(Translation(1,0,0), Rotation()), Axis(1,0,0)),
                                          DofProperties(-M_PI,M_PI));
    
    cout << "created first pair" << endl;
    
    cout << robot.link(0) << endl;
    cout << robot.link(1) << endl;
    
    if(newID <= 0)
    {
        cout << "Wtf invalid creation attempt" << endl;
        return robot;
    }
    
    newID = robot.createJointLinkPair(robot.link(newID), "second_link",
                                      ProtectedJointProperties("second_joint", Joint::PRISMATIC,
                                      Transform(Translation(0,0,1), Rotation()), Axis(0,0,1)),
                                      DofProperties(-10,10));
    
    robot.joint(0).dof(0).position(180*M_PI/180);
    robot.joint(0).dof(0).position(2);
    
    cout << robot.link(2) << endl;
    
    return robot;
}

Robot& build_urdf_robot()
{
    Robot* rb_ptr = new Robot(akin::Frame::World(), verbosity::DEBUG);
    urdfAkin::loadURDF(*rb_ptr, "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                        "../../../resources/drchubo");
    Robot& robot = *rb_ptr;
    
    return robot;
}

void display_robot(Robot& displaying_robot)
{
    osg::Group* root = new osg::Group;
    CustomNode* akinNode = new CustomNode;
    root->addChild(akinNode);

    akinNode->addRobot(displaying_robot);
    akinNode->addRootFrame(akin::Frame::World());
    
    Frame w(Frame::World(),"w");
    
    Robot& r = displaying_robot;

    r.dof(DOF_POS_Z).position(-r.link("leftFoot").respectToWorld().translation()[2]);

    
    int m = r.addManipulator(r.joint("LWR").childLink(), "leftHandManip", 
                             r.link("leftPalm").respectToRef());
    Manipulator::Mode mode = Manipulator::LINKAGE;
    akinNode->mode = mode;
    if(m < 0)
        std::cout << "something went wrong: " << m << std::endl;
    else
    {
//        r.manip(m).setConstraint(new ManipConstraint<7>(r.manip(m),joints));
//        r.manip(m).setConstraint(Manipulator::FULLBODY, new ManipConstraintX(7, r.manip(m), joints));
    }
//    ManipConstraintX* mptr = new ManipConstraintX(7, r.manip(m), joints);
//    delete mptr;

    r.joint("LEP").dof(0).position(-90*DEG);
    w.respectToRef(r.manip(m).respectToWorld());
    r.manip(m).constraint(mode)->target.changeRefFrame(w);
    r.manip(m).constraint(mode)->target = r.manip(m).withRespectTo(w);
    Transform tf = r.manip(m).constraint(mode)->target.respectToRef();
//    tf.translate(Vec3(0.2,0,0.2));
    tf.translate(Vec3(0,0.05,0.1));
//    tf.rotate(Rotation(-90*DEG, Vec3(1,0,0)));
//    tf.rotate(Rotation(1*DEG,Vec3(0,0,1)));
    r.manip(m).constraint(mode)->target = tf;

    akinNode->setManipulator(r.manip(m));

    std::vector<size_t> joints = r.manip(m).constraint(mode)->getDofs();
    Eigen::VectorXd config = r.getConfig(joints);
    RobotSolverX solver(r);
    solver.setMandatoryConstraint(r.manip(m).constraint(mode));
    if(solver.solve(config))
    {
        std::cout << "Solved!" << std::endl;

//        std::cout << config.transpose() << std::endl;
    }
    else
    {
        std::cout << "Failed!" << std::endl;

        std::cout << config.transpose() << std::endl;
//        return;
    }
    
    
//    AnalyticalIKTemplate<7> anal(r.manip(m), joints);
//    anal.target.changeRefFrame(w);
//    anal.target = r.manip(m).withRespectTo(w);

////    anal.min_limits[0] = -INFINITY; anal.max_limits[0] = INFINITY;
////    anal.min_limits[0] = -0.1; anal.min_limits[0] = 0.1;
////    anal.min_limits[1] = -INFINITY; anal.max_limits[1] = INFINITY;
//    anal.min_limits[1] = -0.1; anal.max_limits[1] = 0.1;
//    //    anal.min_limits[2] = -INFINITY; anal.max_limits[2] = INFINITY;
//    anal.min_limits[2] = -0.1; anal.max_limits[2] = 0.1;

//    anal.min_limits[3] = -10*DEG; anal.max_limits[3] = 10*DEG;

//    anal.target.rotate(Rotation(45*DEG, Vec3(1,0,0)));
//    anal.target.translate(Translation(0.0,0.0,-0.4));

//    const KinTransform& goalTf = anal.getGoalTransform(config);
//    std::cout << "Manip\n" << r.manip(m).respectToWorld() << std::endl;
//    std::cout << "Target\n" << anal.target.respectToWorld() << std::endl;
//    std::cout << "Goal\n" << goalTf.respectToWorld() << std::endl;
//    std::cout << "Manip\n" << r.manip(m).withRespectTo(w) << std::endl;
//    std::cout << "Target\n" << anal.target.withRespectTo(w) << std::endl;
//    std::cout << "Goal\n" << goalTf.withRespectTo(w) << std::endl;
//    Eigen::AngleAxisd aaM(r.manip(m).withRespectTo(w).rotation());
//    std::cout << "Mrot " << aaM.angle()/DEG << " <" << aaM.axis().transpose() << ">" << std::endl;
//    Eigen::AngleAxisd aaT(anal.target.withRespectTo(w).rotation());
//    std::cout << "Trot " << aaT.angle()/DEG << " <" << aaT.axis().transpose() << ">" << std::endl;
//    Eigen::AngleAxisd aaG(goalTf.withRespectTo(w).rotation());
//    std::cout << "Grot " << aaG.angle()/DEG << " <" << aaG.axis().transpose() << ">" << std::endl;
////    Eigen::AngleAxisd aa((r.manip(m).respectToWorld()*goalTf.respectToWorld().inverse()).rotation());
////    std::cout << "Diff " << aa.angle()/DEG << " <" << aa.axis().transpose() << ">" << std::endl;



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
