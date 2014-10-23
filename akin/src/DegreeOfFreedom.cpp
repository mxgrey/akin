
#include "../Robot.h"

using namespace akin;

DofProperties::DofProperties(double minimumValue, double maximumValue,
                             double maxSpeed, double maxAcceleration,
                             double maxTorque) :
    _value(0),
    _minValue(minimumValue),
    _maxValue(maximumValue),
    _maxSpeed(maxSpeed),
    _maxAcceleration(maxAcceleration),
    _maxEffort(maxTorque)
{

}

DegreeOfFreedom::DegreeOfFreedom(Joint* parentJoint, const std::string& name,
                                 const DofProperties& properties) :
    DofProperties(properties),
    verb(_parent->verb),
    _name(name),
    _parent(parentJoint),
    _robot(parentJoint->_myRobot)

{

}


bool DegreeOfFreedom::value(double newDofValue)
{
    bool inBounds = true;

    if(newDofValue != newDofValue)
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set value for DOF '"
                    +name()+"' to NaN!");
        return false;
    }

    if(newDofValue < _minValue)
    {
        if(_robot->enforceJointLimits())
            newDofValue = _minValue;
        inBounds = false;
    }

    if(newDofValue > _maxValue)
    {
        if(_robot->enforceJointLimits())
            newDofValue = _maxValue;
        inBounds = false;
    }

    if(newDofValue == _value)
        return inBounds;

    _value = newDofValue;
    _parent->notifyPosUpdate();

    return inBounds;
}

double DegreeOfFreedom::value() const { return _value; }

bool DegreeOfFreedom::velocity(double newDofVelocity)
{
    bool inBounds = true;

    if(newDofVelocity != newDofVelocity)
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set velocity for DOF '"
                    +name()+"' to NaN!");
        return false;
    }

    if(fabs(newDofVelocity) > _maxSpeed)
    {
        if(_robot->enforceJointLimits())
            newDofVelocity = newDofVelocity>0? _maxSpeed : -_maxSpeed;
        inBounds = false;
    }

    if(newDofVelocity==_velocity)
        return inBounds;

    _velocity = newDofVelocity;
    _parent->notifyVelUpdate();

    return inBounds;
}

double DegreeOfFreedom::velocity() const { return _velocity; }


bool DegreeOfFreedom::acceleration(double newDofAcceleration)
{
    if(_robot->getDynamicsMode()==FORWARD)
    {
        verb.Assert(false, verbosity::ASSERT_CASUAL, "Warning: attempting to set acceleration of "
                    "DOF '"+name()+"' when the robot is in FORWARD dynamics mode! "
                    "Torque should be set instead!");
        return false;
    }

    bool inBounds = true;

    if(newDofAcceleration != newDofAcceleration)
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set acceleration for DOF '"
                    +name()+"' to NaN!");
        return false;
    }

    if(fabs(newDofAcceleration) > _maxAcceleration)
    {
        if(_robot->enforceJointLimits())
            newDofAcceleration = newDofAcceleration>0? _maxAcceleration : -_maxAcceleration;
        inBounds = false;
    }

    if(newDofAcceleration==_acceleration)
        return inBounds;

    _acceleration = newDofAcceleration;
    _parent->notifyAccUpdate();

    return inBounds;
}

double DegreeOfFreedom::acceleration() const
{
    if(_robot->getDynamicsMode()==INVERSE)
        return _acceleration;

    if(_parent->childLink().needsDynUpdate())
        _parent->childLink()._computeABA_pass3();

    return _acceleration;
}

bool DegreeOfFreedom::effort(double newEffort)
{
    if(_robot->getDynamicsMode()==INVERSE)
    {
        verb.Assert(false, verbosity::ASSERT_CASUAL, "Warning: Attempting to set torque of "
                    "DOF '"+name()+"' when the robot is in INVERSE dynamics mode! "
                    "Acceleration should be set instead!");
        return false;
    }

    bool inBounds = true;

    if(newEffort != newEffort)
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set torque for DOF '"
                    +name()+"' to NaN!");
        return false;
    }

    if(fabs(newEffort) > _maxEffort)
    {
        if(_robot->enforceJointLimits())
            newEffort = newEffort>0? _maxEffort : -_maxEffort;
        inBounds = false;
    }

    if(newEffort == _effort)
        return inBounds;

    _effort = newEffort;
    _parent->notifyDynUpdate();

    return inBounds;
}

double DegreeOfFreedom::effort() const
{
    // TODO: Compute torque if we're in INVERSE dynamics mode
    return _effort;
}

void DegreeOfFreedom::_computeTransformedJointAxis(Vec3 &z_i, const Frame &refFrame) const
{
    // TODO: Does this accomplish anything? Shouldn't the joint axis be invariant to
    // the rotation of its frame?
//    z_i = _parent->_reversed?
//                Vec3(-_parent->childLink().respectToRef().rotation()*_parent*_axis) :
//                Vec3( _parent->childLink().respectToRef().rotation()*_parent*_axis);

//    if(refFrame.isWorld())
//        z_i = _parent->childLink().respectToWorld().rotation()*z_i;
//    else
//        z_i = refFrame.respectToWorld().rotation().transpose()*
//              _parent->childLink().respectToWorld().rotation()*z_i;

    if(_parent->_reversed)
        z_i = -z_i;

    if(refFrame.isWorld())
        z_i = _parent->upstreamLink().respectToWorld().rotation()*z_i;
    else
        z_i = refFrame.respectToWorld().rotation().transpose()*
              _parent->upstreamLink().respectToWorld().rotation()*z_i;
}

Vec3 DegreeOfFreedom::_computePosJacobian(const Vec3& z_i, const KinTranslation& point,
                                          const Frame& refFrame) const
{
    if(_parent->type()==Joint::REVOLUTE)
    {
        return z_i.cross( point.withRespectTo(refFrame) -
                          _parent->childLink().withRespectTo(refFrame).translation() );
    }
    else if(_parent->type()==Joint::PRISMATIC)
    {
        return z_i;
    }
    else if(_parent->type()==Joint::FIXED)
    {
        return Vec3::Zero();
    }
    else if(_parent->type()==Joint::FLOATING)
    {
        Vec3 v = _parent->upstreamLink().withRespectTo(refFrame).rotation().matrix().col(_localID);
        if(_localID<3)
            return v;
        else
            return v.cross( point.withRespectTo(refFrame) -
                            _parent->childLink().withRespectTo(refFrame).translation() );
    }

    return Vec3::Zero();
}

Vec3 DegreeOfFreedom::_computeRotJacobian(const Vec3& z_i, const akin::Frame& refFrame) const
{
    if(_parent->type()==Joint::REVOLUTE)
        return z_i;
    else if(_parent->type()==Joint::PRISMATIC)
        return Vec3::Zero();
    else if(_parent->type()==Joint::FIXED)
        return Vec3::Zero();
    else if(_parent->type()==Joint::FLOATING)
    {
        if(3 <= _localID && _localID < 6)
            return _parent->upstreamLink().withRespectTo(refFrame).rotation().matrix()
                    .col(_localID);
        else
            return Vec3::Zero();
    }

    return Vec3::Zero();
}

Vec3 DegreeOfFreedom::Jacobian_rotOnly(const KinTranslation &point, const Frame &refFrame,
                                       bool checkDependence) const
{
    if(checkDependence)
    {
        if(!point.descendsFrom(_parent->childLink()))
            return Vec3::Zero();
    }

    Vec3 z_i;
    // TODO: Replace this if statement with a function that
    // checks whether joint axis is used by this joint type
    if(_parent->type()==Joint::REVOLUTE || _parent->type()==Joint::PRISMATIC)
        _computeTransformedJointAxis(z_i, refFrame);

    return _computeRotJacobian(z_i, refFrame);
}

Vec3 DegreeOfFreedom::Jacobian_posOnly(const KinTranslation &point, const Frame &refFrame,
                                       bool checkDependence) const
{
    if(checkDependence)
    {
        if(!point.descendsFrom(_parent->childLink()))
            return Vec3::Zero();
    }

    Vec3 z_i;
    // TODO: Replace this if statement with a function that
    // checks whether joint axis is used by this joint type
    if(_parent->type()==Joint::REVOLUTE || _parent->type()==Joint::PRISMATIC)
        _computeTransformedJointAxis(z_i, refFrame);

    return _computePosJacobian(z_i, point, refFrame);
}

Screw DegreeOfFreedom::Jacobian(const KinTranslation &point, const Frame &refFrame,
                                bool checkDependence) const
{
    if(checkDependence)
    {
        if(!point.descendsFrom(_parent->childLink()))
            return Screw::Zero();
    }

    Vec3 z_i;
    // TODO: Replace this if statement with a function that
    // checks whether joint axis is used by this joint type
    if(_parent->type()==Joint::REVOLUTE || _parent->type()==Joint::PRISMATIC)
        _computeTransformedJointAxis(z_i, refFrame);

    return Screw(_computePosJacobian(z_i,point,refFrame), _computeRotJacobian(z_i,refFrame));
}

const std::string& DegreeOfFreedom::name() const { return _name; }

bool DegreeOfFreedom::name(const std::string &newName)
{
    if( verb.Assert(!_robot->checkForDofName(newName),
                    verbosity::ASSERT_CRITICAL,
                    "You requested to change DOF named '"+name()+"' to '"
                    +newName+"', but robot '"+_robot->name()+"' already has "
                    "a joint with that name!"))
        return false;

    StringMap::iterator n = _robot->_dofNameToIndex.find(name());
    size_t index = n->second;
    _robot->_dofNameToIndex.erase(n);
    _robot->_dofNameToIndex[newName] = index;

    _name = newName;

    return true;
}

Joint& DegreeOfFreedom::joint() { return *_parent; }
const Joint& DegreeOfFreedom::joint() const { return *_parent; }

Robot& DegreeOfFreedom::robot() { return *_robot; }
const Robot& DegreeOfFreedom::robot() const { return *_robot; }

size_t DegreeOfFreedom::id() const { return _id; }
size_t DegreeOfFreedom::localID() const { return _localID; }
