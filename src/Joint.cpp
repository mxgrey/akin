
#include "Link.h"

using namespace akin;
using namespace std;

bool Joint::value(double newJointValue)
{
    bool inBounds = true;
    if(newJointValue < _min)
    {
        newJointValue = _min;
        inBounds = false;
    }
    
    if(newJointValue > _max)
    {
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
    if(!_reversed)
    {
        _downstreamLink->respectToRef(respectToRef.inverse());
    }
    else
    {
        _downstreamLink->respectToRef(respectToRef);
    }
}

Joint::Joint(Robot *mRobot, size_t jointID, const string &jointName,
             Link *mParentLink, Link *mChildLink,
             const Transform &mBaseTransform,
             const Axis &mJointAxis, Type mType,
             double mininumValue, double maximumValue,
             verbosity::verbosity_level_t report_level) :
    _myRobot(mRobot),
    _id(jointID),
    _name(jointName),
    _parentLink(mParentLink),
    _childLink(mChildLink),
    _baseTransform(mBaseTransform),
    _axis(mJointAxis),
    _min(mininumValue),
    _max(maximumValue),
    _myType(mType),
    _reversed(false)
{
    value(0);
    verb.level = report_level;
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
