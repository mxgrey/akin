
#include "akin/Link.h"

using namespace akin;
using namespace std;

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

bool Joint::withinLimits()
{
    if( min() <= value() && value() <= max() )
        return true;
    
    return false;
}

bool Joint::withinLimits(double someValue)
{
    if( min() <= someValue && someValue <= max() )
        return true;
    
    return false;
}

void Joint::_changeParentLink(Link *newParent)
{
    _upstreamLink = newParent;
    _parentLink = newParent;
}

//Robot& Joint::robot() const { return *_myRobot; }

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

Joint& Joint::parentJoint()
{
    return _parentLink->parentJoint();
}

Joint& Joint::childJoint(size_t num)
{
    return _childLink->childJoint(num);
}

size_t Joint::numChildJoints()
{
    return _childLink->numChildJoints();
}

Joint& Joint::upstreamJoint()
{
    return _upstreamLink->upstreamJoint();
}

Joint& Joint::downstreamJoint(size_t num)
{
    return _downstreamLink->downstreamJoint(num);
}

size_t Joint::numDownstreamJoints()
{
    return _downstreamLink->numDownstreamJoints();
}
