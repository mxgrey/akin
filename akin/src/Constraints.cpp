
#include "akin/RobotConstraint.h"
#include "sstream"

using namespace akin;
using namespace std;

ConstraintBase::Validity::Validity() :
    valid(false),
    stuck(false),
    near_edge(false)
{
    
}

ConstraintBase::Validity ConstraintBase::Validity::Valid()
{
    Validity v;
    v.valid = true;
    return v;
}

ConstraintBase::Validity ConstraintBase::Validity::Stuck()
{
    Validity v;
    v.stuck = true;
    return v;
}

ConstraintBase::Validity ConstraintBase::Validity::Invalid()
{
    Validity v;
    return v;
}

bool operator==(const ConstraintBase::Validity& v1, const ConstraintBase::Validity& v2)
{
    if( (v1.valid == v2.valid) 
            && (v1.stuck == v2.stuck) 
            && (v1.near_edge == v2.near_edge) )
        return true;
    
    return false;
}

std::string ConstraintBase::Validity::toString() const
{
    stringstream str;
    if(valid)
        str << "VALID";
    else
        str << "INVALID";
    
    if(stuck)
        str << "|STUCK";
    
    if(near_edge)
        str << "|NEAR EDGE";
    
    return str.str();
}

std::ostream& operator<<(std::ostream& oStrStream, const ConstraintBase::Validity& v)
{
    oStrStream << v.toString();
    return oStrStream;
}

ConstraintBase::Validity NullConstraintBase::getGradientX(Eigen::VectorXd &gradient, 
                                                          const Eigen::VectorXd &configuration)
{
    gradient.resize(configuration.size());
    gradient.setZero();
    return Validity::Valid();
}

ConstraintBase::Validity NullConstraintBase::getValidityX(const Eigen::VectorXd &)
{
    return Validity::Valid();
}

double NullConstraintBase::getErrorNormX(const Eigen::VectorXd &, bool) 
{
    return 0;
}

int NullConstraintBase::getConfigurationSize()
{
    return 0;
}

/////// Robot Constraints

RobotConstraintBase::RobotConstraintBase() :
    _robot(NULL)
{
    
}

RobotConstraintBase::RobotConstraintBase(Robot& robot, const std::vector<size_t> &joints) :
    _robot(&robot),
    _joints(joints)
{
    
}

bool RobotConstraintBase::changeRobot(Robot *newRobot, bool reconfigure)
{
    _robot = newRobot;
    
    if(reconfigure)
        return _reconfigure();
    return false;
}

bool RobotConstraintBase::changeJoints(const std::vector<size_t> &newJoints, bool reconfigure)
{
    _joints = newJoints;
    
    if(reconfigure)
        return _reconfigure();
    return false;
}

bool RobotConstraintBase::changeSetup(Robot* newRobot, const std::vector<size_t>& newJoints) 
{
    changeRobot(newRobot, false);
    return changeJoints(newJoints);
}

ManipConstraintBase::ManipConstraintBase() :
    manip(NULL),
    target(Frame::World(), "manip_target")
{
    
}

ManipConstraintBase::ManipConstraintBase(Manipulator& manipulator) :
    manip(&manipulator),
    target(Frame::World(), manipulator.name()+"_target")
{
    target.respectToRef(manip->respectToWorld());
}

bool ManipConstraintBase::checkSetup() const
{
    if(_robot==NULL)
    {
        std::cout << "Constraint has not been assigned a robot yet!" << std::endl;
        return false;
    }
    
    if(!_robot->verb.Assert(manip != NULL, verbosity::ASSERT_CRITICAL,
                                  "Constraint has not been set up yet!") )
        return false;
    
    return true;
}

bool ManipConstraintBase::_reconfigure() {
    if(this->_robot==NULL)
        return false;
    
    this->_robot->verb.Assert((int)this->_joints.size()==getConfigurationSize(), 
                              verbosity::ASSERT_CRITICAL,
                        "ManipConstraint templated for "
                              +std::to_string(getConfigurationSize())
                        +" DoF was given an index array of size "
                        +std::to_string(this->_joints.size())+"!");
    
    if(manip==NULL)
        return false;
    
    _dependency.clear();
    for(int i=0; i<getConfigurationSize(); ++i)
    {
        const Joint& j = this->_robot->const_joint(this->_joints[i]);
        if(manip->descendsFrom(j.const_childLink()))
            _dependency.push_back(true);
        else
            _dependency.push_back(false);
    }
    
    return true;
}


