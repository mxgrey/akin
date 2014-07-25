
#include "akin/Robot.h"

using namespace akin;
using namespace std;

std::string Joint::type_to_string(Type myJointType)
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

Screw Joint::Jacobian(const KinTranslation &point, const Frame &refFrame,
                      bool checkDescension) const
{
    if(checkDescension)
    {
        if(!point.descendsFrom(const_childLink()))
            return Screw::Zero();
    }
    
//    Translation z_i = _reversed ?
//                          (-const_childLink().respectToRef().rotation()*_axis) :
//                           (const_childLink().respectToRef().rotation()*_axis);
    
//    Axis z_i = const_childLink().respectToRef().rotation()*_axis;
//    z_i = const_childLink().respectToRef().rotation()*Translation::Zero();
//    Translation z_i = const_childLink().respectToRef()*Translation::Zero();
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
//        cout << "(reversed) Setting respectToRef of " << _downstreamLink->name() << " to\n"
//             << respectToRef.inverse().matrix() << endl;
        _downstreamLink->respectToRef(respectToRef.inverse());
    }
    else
    {
//        cout << "Setting respectToRef of " << _downstreamLink->name() << " to\n"
//             << respectToRef.matrix() << endl;
        _downstreamLink->respectToRef(respectToRef);
    }
}

Joint::Joint(Robot *mRobot, size_t jointID, const string &jointName,
             Link *mParentLink, Link *mChildLink,
             const Transform &mBaseTransform,
             const Axis &mJointAxis, Type mType,
             double mininumValue, double maximumValue) :
    verb(mRobot->verb),
    _id(jointID),
    _name(jointName),
    _parentLink(mParentLink),
    _childLink(mChildLink),
    _reversed(false),
    _upstreamLink(mParentLink),
    _downstreamLink(mChildLink),
    _baseTransform(mBaseTransform),
    _axis(mJointAxis),
    _value(0),
    _min(mininumValue),
    _max(maximumValue),
    _myType(mType),
    _isDummy(false),
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
void Joint::type(Type newType) { _myType = newType; _computeRefTransform(); }

void Joint::_changeParentLink(Link *newParent)
{
    _upstreamLink = newParent;
    _parentLink = newParent;
}

void Joint::axis(Axis newAxis) { _axis = newAxis; _computeRefTransform(); }
const Axis& Joint::axis() const { return _axis; }

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

