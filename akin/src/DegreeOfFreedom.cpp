
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
    verb(parentJoint->verb),
    _name(name),
    _parent(parentJoint),
    _robot(parentJoint->_robot)
{

}


bool DegreeOfFreedom::position(double newDofPosition)
{
    bool inBounds = true;

    if(newDofPosition != newDofPosition)
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set value for DOF '"
                    +name()+"' to NaN!");
        return false;
    }

    if(newDofPosition < _minValue)
    {
        if(_robot->enforceJointLimits())
            newDofPosition = _minValue;
        inBounds = false;
    }

    if(newDofPosition > _maxValue)
    {
        if(_robot->enforceJointLimits())
            newDofPosition = _maxValue;
        inBounds = false;
    }

    if(newDofPosition == _value)
        return inBounds;

    _value = newDofPosition;
    _parent->notifyPosUpdate();

    return inBounds;
}

double DegreeOfFreedom::position() const { return _value; }

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

bool DegreeOfFreedom::effort(double newDofEffort)
{
    if(_robot->getDynamicsMode()==INVERSE)
    {
        verb.Assert(false, verbosity::ASSERT_CASUAL, "Warning: Attempting to set torque of "
                    "DOF '"+name()+"' when the robot is in INVERSE dynamics mode! "
                    "Acceleration should be set instead!");
        return false;
    }

    bool inBounds = true;

    if(newDofEffort != newDofEffort)
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set torque for DOF '"
                    +name()+"' to NaN!");
        return false;
    }

    if(fabs(newDofEffort) > _maxEffort)
    {
        if(_robot->enforceJointLimits())
            newDofEffort = newDofEffort>0? _maxEffort : -_maxEffort;
        inBounds = false;
    }

    if(newDofEffort == _effort)
        return inBounds;

    _effort = newDofEffort;
    _parent->notifyDynUpdate();

    return inBounds;
}

double DegreeOfFreedom::effort() const
{
    // TODO: Compute torque if we're in inverse dynamics mode
    return _effort;
}

bool DegreeOfFreedom::property(property_t p, double newValue)
{
    switch(p)
    {
        case POSITION:      return position(newValue);
        case VELOCITY:      return velocity(newValue);
        case ACCELERATION:  return acceleration(newValue);
        case EFFORT:        return effort(newValue);
        default:
            verb.Assert(false, verbosity::ASSERT_CASUAL,
                        "Called property(~,~) on DegreeOfFreedom named '"+name()+"' with invalid "
                        "property_t: "+property_to_string(p));
            return false;
    }
}

double DegreeOfFreedom::property(property_t p) const
{
    switch(p)
    {
        case POSITION:      return position();
        case VELOCITY:      return velocity();
        case ACCELERATION:  return acceleration();
        case EFFORT:        return effort();
        default:
            verb.Assert(false, verbosity::ASSERT_CASUAL,
                        "Called property(~) on DegreeOfFreedom named '"+name()+"' with invalid "
                        "property_t: "+property_to_string(p));
            return 0;
    }
}

void DegreeOfFreedom::limits(double newMinValue, double newMaxValue)
{
    verb.Assert(newMinValue <= newMaxValue, verbosity::ASSERT_CASUAL,
                "You have entered the following DOF limits: ["
                +std::to_string(newMinValue)+","+std::to_string(newMaxValue)
                +"], but the min is larger than the max!");

    _minValue = newMinValue;
    _maxValue = newMaxValue;

    position(position());
}

void DegreeOfFreedom::limits(const std::pair<double, double> &newLimits)
{
    limits(newLimits.first, newLimits.second);
}

std::pair<double,double> DegreeOfFreedom::limits() const
{
    return std::pair<double,double>(_minValue,_maxValue);
}

bool DegreeOfFreedom::min(double newMinValue)
{
    bool inBounds = true;
    if(newMinValue > _maxValue)
    {
        newMinValue = _maxValue;
        inBounds = false;
    }

    position(position());

    return inBounds;
}

double DegreeOfFreedom::min() const { return _minValue; }

bool DegreeOfFreedom::max(double newMaxValue)
{
    bool inBounds = true;
    if(newMaxValue < _minValue)
    {
        newMaxValue = _minValue;
        inBounds = false;
    }

    position(position());

    return inBounds;
}

double DegreeOfFreedom::max() const { return _maxValue; }

void DegreeOfFreedom::maxSpeed(double newMaxSpeed)
{
    _maxSpeed = newMaxSpeed;
    velocity(velocity());
}

double DegreeOfFreedom::maxSpeed() const { return _maxSpeed; }

void DegreeOfFreedom::maxAcceleration(double newMaxAcceleration)
{
    _maxAcceleration = newMaxAcceleration;
    acceleration(acceleration());
}

bool DegreeOfFreedom::withinLimits() const
{
    return withinLimits(position());
}

bool DegreeOfFreedom::withinLimits(double someValue) const
{
    if( min() <= someValue && someValue <= max() )
        return true;

    return false;
}

void DegreeOfFreedom::_computeTransformedJointAxis(Vec3 &z_i, const Frame &refFrame) const
{
//    z_i = _parent->_reversed?
//                Vec3(-_parent->childLink().respectToRef().rotation()*_parent->_axis) :
//                Vec3( _parent->childLink().respectToRef().rotation()*_parent->_axis);
    z_i = _parent->_reversed?
                Vec3(-_parent->_baseTransform.rotation()*_parent->_axis) :
                Vec3( _parent->_baseTransform.rotation()*_parent->_axis);

    if(refFrame.isWorld())
        z_i = _parent->childLink().respectToWorld().rotation()*z_i;
    else
        z_i = refFrame.respectToWorld().rotation().transpose()*
              _parent->childLink().respectToWorld().rotation()*z_i;
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
        if(_localID<3)
            return _parent->upstreamLink().withRespectTo(refFrame).rotation().matrix().col(_localID);
        else
        {
            const Vec3& v = _parent->upstreamLink().withRespectTo(refFrame).
                    rotation().matrix().col(_localID-3);
            return v.cross( point.withRespectTo(refFrame) -
                            _parent->childLink().withRespectTo(refFrame).translation() );
        }
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
