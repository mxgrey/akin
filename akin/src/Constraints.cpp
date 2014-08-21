
#include "akin/RobotConstraint.h"
#include "../AnalyticalIKBase.h"
#include "sstream"

using namespace akin;
using namespace std;

ConstraintBase::~ConstraintBase() { }

ConstraintBase::ConstraintBase() : _isNull(false) { }

bool ConstraintBase::isNull() const { return _isNull; }

Validity::Validity() :
    valid(false),
    stuck(false),
    near_edge(false)
{
    
}

Validity Validity::Valid()
{
    Validity v;
    v.valid = true;
    return v;
}

Validity Validity::Stuck()
{
    Validity v;
    v.stuck = true;
    return v;
}

Validity Validity::Invalid()
{
    Validity v;
    return v;
}

bool operator==(const Validity& v1, const Validity& v2)
{
    if( (v1.valid == v2.valid) 
            && (v1.stuck == v2.stuck) 
            && (v1.near_edge == v2.near_edge) )
        return true;
    
    return false;
}

Validity& Validity::operator&=(const Validity& otherV)
{
    this->valid &= otherV.valid;
    this->stuck &= otherV.stuck;
    this->near_edge &= otherV.near_edge;

    return *this;
}

Validity& Validity::operator |=(const Validity& otherV)
{
    this->valid |= otherV.valid;
    this->stuck |= otherV.stuck;
    this->near_edge |= otherV.near_edge;

    return *this;
}

std::string Validity::toString() const
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

std::ostream& operator<<(std::ostream& oStrStream, const Validity& v)
{
    oStrStream << v.toString();
    return oStrStream;
}



NullConstraintBase::~NullConstraintBase() { }

NullConstraintBase::NullConstraintBase() { _isNull = true; }

Validity NullConstraintBase::computeGradient(const Eigen::VectorXd &) { return Validity::Valid(); }

double NullConstraintBase::getGradientComponent(size_t) const { return 0; }

Validity NullConstraintBase::getGradientX(Eigen::VectorXd &gradient,
                                                          const Eigen::VectorXd &configuration)
{
    gradient.resize(configuration.size());
    gradient.setZero();
    return Validity::Valid();
}

Validity NullConstraintBase::getValidityX(const Eigen::VectorXd &)
{
    return Validity::Valid();
}

double NullConstraintBase::getErrorNormX(const Eigen::VectorXd &, bool) 
{
    return 0;
}

int NullConstraintBase::getConfigurationDimension()
{
    return 0;
}

JacobianConstraintBase::~JacobianConstraintBase() { }


NullJacobianConstraint::~NullJacobianConstraint() { }

void NullJacobianConstraint::computeJacobian(const Eigen::VectorXd &) { }

double NullJacobianConstraint::getJacobianComponent(size_t, size_t) const { return 0; }

bool NullJacobianConstraint::computeJPinvJ(const Eigen::VectorXd &, bool) { return false; }

double NullJacobianConstraint::getJPinvJComponent(size_t, size_t) const { return 0; }

void NullJacobianConstraint::computeError(const Eigen::VectorXd &) { }

double NullJacobianConstraint::getErrorComponent(size_t) { return 0; }

int NullJacobianConstraint::getErrorDimension() { return 0; }

/////// Robot Constraints

RobotConstraintBase::~RobotConstraintBase() { }

RobotConstraintBase::RobotConstraintBase() :
    _configured(false),
    _robot(NULL)
{
    
}

RobotConstraintBase::RobotConstraintBase(Robot& robot, const std::vector<size_t>& joints) :
    _configured(false),
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

const std::vector<size_t>& RobotConstraintBase::getJoints() const
{
    return _joints;
}

Robot* RobotConstraintBase::getRobot()
{
    return _robot;
}

bool RobotConstraintBase::checkSetup() const 
{
    if(_robot==NULL)
    {
        std::cout << "Constraint has not been assigned a robot yet!" << std::endl;
        return false;
    }
    
    if(!_configured)
    {
        std::cout << "Constraint has not been configured yet!" << std::endl;
    }
    
    return true;
}

bool RobotConstraintBase::_reconfigure()
{
    if(this->_robot==NULL)
    {
        _configured = false;
        return false;
    }
    
    if(!this->_robot->verb.Assert((int)this->_joints.size()==getConfigurationDimension(),
                              verbosity::ASSERT_CRITICAL,
                        "ManipConstraint templated for "
                              +std::to_string(getConfigurationDimension())
                        +" DoF was given an index array of size "
                        +std::to_string(this->_joints.size())+"!"))
    {
        _configured = false;
        return false;
    }
    
    _configured = true;
    return true;
}

ManipConstraintBase::~ManipConstraintBase() { }

ManipConstraintBase::ManipConstraintBase() :
    target(Frame::World(), "manip_target"),
    _manip(NULL)
{
    
}

ManipConstraintBase::ManipConstraintBase(Manipulator& manipulator) :
    target(Frame::World(), manipulator.name()+"_target"),
    _manip(&manipulator)
{
    target = _manip->respectToWorld();
}

Manipulator* ManipConstraintBase::manip()
{
    return _manip;
}

bool ManipConstraintBase::changeManipulator(Manipulator *manip)
{
    if(!_robot->owns(*manip))
        return false;

    _manip = manip;
    return _reconfigure();
}

bool ManipConstraintBase::checkSetup() const
{
    if(!RobotConstraintBase::checkSetup())
        return false;
    
    if(!_robot->verb.Assert(_manip != NULL, verbosity::ASSERT_CRITICAL,
                                  "Constraint has not been set up yet!") )
        return false;
    
    return true;
}

bool ManipConstraintBase::_reconfigure() {
    
    if(!RobotConstraintBase::_reconfigure())
        return false;
    
    if(_manip==NULL)
    {
        _configured = false;
        return false;
    }
    
    _dependency.clear();
    for(int i=0; i<getConfigurationDimension(); ++i)
    {
        const Joint& j = this->_robot->const_joint(this->_joints[i]);
        if(_manip->descendsFrom(j.const_childLink()))
            _dependency.push_back(true);
        else
            _dependency.push_back(false);
    }
    
    _configured = true;
    return true;
}

NullManipConstraint::~NullManipConstraint() { }

NullManipConstraint::NullManipConstraint() { }

NullManipConstraint::NullManipConstraint(Robot &robot)
{
    _robot = &robot;
}

int NullManipConstraint::getErrorDimension() { return 6; }

///////////// CenterOfMassConstraintBase

CenterOfMassConstraintBase::CenterOfMassConstraintBase() :
    useRobotSupportPolygon(true),
    min_height(-INFINITY),
    max_height( INFINITY)
{

}

///////////// Analytical IK

AnalyticalIKBase::~AnalyticalIKBase() { }

AnalyticalIKBase::AnalyticalIKBase() : options(0), ignore_joint_limits(false) { }

void NullAnalyticalIK::getSolutionsX(const Eigen::VectorXd& ,
                                     std::vector<Eigen::VectorXd>& solutions,
                                     std::vector<bool>& valid )
{
    solutions.clear();
    valid.clear();
}

Validity NullAnalyticalIK::getBestSolutionX(Eigen::VectorXd &, const Eigen::VectorXd& )
{
    return Validity::Stuck();
}
