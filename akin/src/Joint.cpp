
#include "akin/Robot.h"

using namespace akin;
using namespace std;

std::string PublicJointProperties::type_to_string(akin::Joint::Type myJointType)
{
    switch(myJointType)
    {
        case FIXED:             return "FIXED";
        case REVOLUTE:          return "REVOLUTE";
        case PRISMATIC:         return "PRISMATIC";
        case CUSTOM:            return "CUSTOM";
        case JOINT_TYPE_SIZE:   return "INVALID (JOINT_TYPE_SIZE)";
    }
    
    return "UNKNOWN JOINT TYPE ("+to_string((int)myJointType)+")";
}

bool Joint::value(double newJointValue)
{
    bool inBounds = true;
    
    if(newJointValue != newJointValue)
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set value for joint '"
                    +name()+"' to NaN!");
        return false;
    }
    
    if(newJointValue < _min)
    {
        if(_myRobot->enforcingJointLimits())
            newJointValue = _min;
        inBounds = false;
    }
    
    if(newJointValue > _max)
    {
        if(_myRobot->enforcingJointLimits())
            newJointValue = _max;
        inBounds = false;
    }
    
    if(newJointValue == _value)
        return true;
    
    _value = newJointValue;
    
    _computeRefTransform();
    
    return inBounds;
}

double Joint::value() const { return _value; }

void Joint::_computeTransformedJointAxis(Vec3 &z_i, const akin::Frame& refFrame) const
{
    z_i = _reversed ?
            Vec3(-const_childLink().respectToRef().rotation()*_axis) :
            Vec3(const_childLink().respectToRef().rotation()*_axis);
    
    // Put z_i into the reference frame
    if(refFrame.isWorld())
        z_i = const_childLink().respectToWorld().rotation()*z_i;
    else
        z_i = refFrame.respectToWorld().rotation().transpose()
              *const_childLink().respectToWorld().rotation()*z_i;
}

Vec3 Joint::_computePosJacobian(const Vec3 &z_i, const KinTranslation &point, 
                                const Frame &refFrame) const
{
    if(type()==REVOLUTE)
    {
        return z_i.cross( point.withRespectTo(refFrame)
                          - const_childLink().withRespectTo(refFrame).translation() );
    }
    else if(type()==PRISMATIC)
        return z_i;
    
    return Vec3::Zero();
}

Vec3 Joint::_computeRotJacobian(const Vec3 &z_i) const
{
    if(type()==REVOLUTE)
        return z_i;
    else if(type()==PRISMATIC)
        return Vec3::Zero();
    
    return Vec3::Zero();
}

Vec3 Joint::Jacobian_rotOnly(const KinTranslation &point, const Frame &refFrame,
                         bool checkDependence) const
{
    if(checkDependence)
    {
        if(!point.descendsFrom(const_childLink()))
            return Vec3::Zero();
    }
    
    Vec3 z_i;
    _computeTransformedJointAxis(z_i, refFrame);
    
    return _computeRotJacobian(z_i);
}

Vec3 Joint::Jacobian_posOnly(const KinTranslation &point, const Frame &refFrame, 
                      bool checkDependence) const
{
    if(checkDependence)
    {
        if(!point.descendsFrom(const_childLink()))
            return Vec3::Zero();
    }
    
    Vec3 z_i;
    _computeTransformedJointAxis(z_i, refFrame);
    
    return _computePosJacobian(z_i, point, refFrame);
}

Screw Joint::Jacobian(const KinTranslation& point, const Frame &refFrame,
                      bool checkDependence) const
{
    if(checkDependence)
    {
        if(!point.descendsFrom(const_childLink()))
            return Screw::Zero();
    }
    
    Vec3 z_i;
    _computeTransformedJointAxis(z_i, refFrame);
    
    return Screw(_computePosJacobian(z_i,point,refFrame), _computeRotJacobian(z_i));
}

void Joint::_computeRefTransform()
{
    // Handle different joint types
    Transform respectToRef = _baseTransform;
    if(REVOLUTE == _myType)
    {
        respectToRef = respectToRef * Transform(Translation(0,0,0),
                                                Rotation(_value, _axis));
    }
    else if(PRISMATIC == _myType)
    {
        respectToRef = respectToRef * Transform(_value*_axis);
    }
    
    // Handle if the kinematic direction is reversed
    if(_reversed)
    {
        _downstreamLink->respectToRef(respectToRef.inverse());
    }
    else
    {
        _downstreamLink->respectToRef(respectToRef);
    }
}

ProtectedJointProperties::ProtectedJointProperties() { }

ProtectedJointProperties::ProtectedJointProperties(
        size_t jointID, const string &jointName, const Transform &mBaseTransform,
        const Axis &mJointAxis, PublicJointProperties::Type mType,
        double minimumValue, double maximumValue) :
    _baseTransform(mBaseTransform),
    _axis(mJointAxis),
    _value(0),
    _min(minimumValue),
    _max(maximumValue),
    _myType(mType),
    _id(jointID),
    _name(jointName),
    _isDummy(false)
{

}

Joint::Joint(Robot *mRobot, size_t jointID, const string &jointName,
             Link *mParentLink, Link *mChildLink,
             const Transform &mBaseTransform,
             const Axis &mJointAxis, akin::Joint::Type mType,
             double mininumValue, double maximumValue) :
    ProtectedJointProperties(jointID, jointName, mBaseTransform, mJointAxis, mType,
                             mininumValue, maximumValue),
    verb(mRobot->verb),
    _parentLink(mParentLink),
    _childLink(mChildLink),
    _reversed(false),
    _upstreamLink(mParentLink),
    _downstreamLink(mChildLink),
    _myRobot(mRobot)
{
    _computeRefTransform();
    
//    std::cout << "Joint '"+jointName+"' wants to connect '"+mChildLink->name()
//                 +"' to '"+mParentLink->name()+"' but '"+mChildLink->name()
//                 +"' is in the reference frame of '"+mChildLink->refFrame().name()
//                 +"'!" << std::endl;
    
    if( !mChildLink->isDummy() )
        verb.Assert(mParentLink == &mChildLink->refFrame(), verbosity::ASSERT_CRITICAL,
                "Joint '"+jointName+"' wants to connect '"+mChildLink->name()
                +"' to '"+mParentLink->name()+"' but '"+mChildLink->name()
                +"' is in the reference frame of '"+mChildLink->refFrame().name()
                +"'!", " A joint must always connect a link to its parent frame!");
}

Joint::~Joint()
{
    
}

Joint& Joint::operator=(const Joint& otherJoint)
{
    (PublicJointProperties&)(*this) = (PublicJointProperties&)(otherJoint);
    (ProtectedJointProperties&)(*this) = (ProtectedJointProperties&)(otherJoint);

    return *this;
}

bool Joint::min(double newMinValue)
{
    bool inBounds = true;
    if(newMinValue > _max)
    {
        newMinValue = _max;
        inBounds = false;
    }
    
    value(value());
    
    return inBounds;
}

double Joint::min() const { return _min; }

bool Joint::max(double newMaxValue)
{
    bool inBounds = true;
    if(newMaxValue < _min)
    {
        newMaxValue = _min;
        inBounds = false;
    }
    
    value(value());
    
    return inBounds;
}

double Joint::max() const { return _max; }

bool Joint::withinLimits() const
{
    return withinLimits(value());
}

bool Joint::withinLimits(double someValue) const
{
    if( min() <= someValue && someValue <= max() )
        return true;
    
    return false;
}

Joint::Type Joint::type() const { return _myType; }
void Joint::type(akin::Joint::Type newType) { _myType = newType; _computeRefTransform(); }

void Joint::_changeParentLink(Link *newParent)
{
    _upstreamLink = newParent;
    _parentLink = newParent;
}

void Joint::axis(Vec3 newAxis) { _axis = Axis(newAxis); _computeRefTransform(); }
const Vec3& Joint::axis() const { return _axis; }

void Joint::baseTransform(const Transform &newBaseTf)
{
    _baseTransform = newBaseTf; _computeRefTransform();
}

const Transform& Joint::baseTransform() const { return _baseTransform; }

size_t Joint::id() const { return _id; }

std::string Joint::name() const { return _name; }

//Robot& Joint::robot() const { return *_myRobot; }

bool Joint::name(const string &new_name)
{
    if( verb.Assert(!_myRobot->checkForJointName(new_name),
                         verbosity::ASSERT_CRITICAL,
                         "You requested to change joint named '"+name()+"' to '"
                         +new_name+"', but robot '"+_myRobot->name()+"' already has "
                         +"a joint with that name!"))
        return false;
    
    StringMap::iterator n = _myRobot->_jointNameToIndex.find(name());
    size_t index = n->second;
    _myRobot->_jointNameToIndex.erase(n);
    _myRobot->_jointNameToIndex[new_name] = index;
    
    _name = new_name;
    
    return true;
}

Link& Joint::parentLink() { return *_parentLink; }
const Link& Joint::const_parentLink() const { return const_cast<Joint*>(this)->parentLink(); }

Link& Joint::childLink() { return *_childLink; }
const Link& Joint::const_childLink() const { return const_cast<Joint*>(this)->childLink(); }

Joint& Joint::parentJoint() { return _parentLink->parentJoint(); }
const Joint& Joint::const_parentJoint() const { return const_cast<Joint*>(this)->parentJoint(); }

Joint& Joint::childJoint(size_t num) { return _childLink->childJoint(num); }
const Joint& Joint::const_childJoint(size_t num) const
    { return const_cast<Joint*>(this)->childJoint(num); }

size_t Joint::numChildJoints() const { return _childLink->numChildJoints(); }

Link& Joint::upstreamLink() { return *_upstreamLink; }
const Link& Joint::const_upstreamLink() const { return const_cast<Joint*>(this)->upstreamLink(); }

Link& Joint::downstreamLink() { return *_downstreamLink; }
const Link& Joint::const_downstreamLink() const
    { return const_cast<Joint*>(this)->downstreamLink(); }

Joint& Joint::upstreamJoint() { return _upstreamLink->upstreamJoint(); }
const Joint& Joint::const_upstreamJoint() const 
    { return const_cast<Joint*>(this)->upstreamJoint(); }

Joint& Joint::downstreamJoint(size_t num) { return _downstreamLink->downstreamJoint(num); }
const Joint& Joint::const_downstreamJoint(size_t num) const
    { return const_cast<Joint*>(this)->downstreamJoint(num); }

size_t Joint::numDownstreamJoints() const { return _downstreamLink->numDownstreamJoints(); }

bool Joint::belongsTo(const Robot &someRobot) const { return &someRobot == _myRobot; }
Robot& Joint::robot() { return *_myRobot; }
const Robot& Joint::const_robot() const { return const_cast<Joint*>(this)->robot(); }

bool Joint::isDummy() const { return _isDummy; }

bool Joint::isReversed() const { return _reversed; }

std::ostream& operator<<(std::ostream& oStrStream, akin::Joint::Type type)
{
    oStrStream << Joint::type_to_string(type);
    return oStrStream;
}

std::ostream& operator<<(std::ostream& oStrStream, const akin::Joint& someJoint)
{
    oStrStream << "Joint named '" << someJoint.name() << "' with ID " << someJoint.id()
               << " connects Parent Link '" << someJoint.const_parentLink().name() << "' to Child '" 
               << someJoint.const_childLink().name() << "' for robot '" 
               << someJoint.const_robot().name() << "'\n";
    if(someJoint.isReversed())
        oStrStream << "[Parent/Child are currently kinematically reversed]\n";
    oStrStream << "Axis: <" << someJoint.axis().transpose() << "> (" << someJoint.type() 
               << ") with Base Transform:\n" << someJoint.baseTransform() << "\n";
    oStrStream << "Current value: " << someJoint.value() << " (min " 
               << someJoint.min() << " | max " << someJoint.max() << ")";
    if(!someJoint.withinLimits())
        oStrStream << " [Currently outside its limits!]";
    oStrStream << "\n";
    
    return oStrStream;
}

