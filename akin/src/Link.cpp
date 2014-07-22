
#include "akin/Link.h"

using namespace akin;
using namespace std;

Link::Link(Robot *mRobot, Frame &referenceFrame, string linkName, size_t mID, bool root) :
    Body(referenceFrame, linkName),
    _id(mID),
    _isAnchor(root),
    _isRoot(root),
    _isDummy(false),
    _myRobot(mRobot)
{
    _needsUpdate = true;
    
    if(_isRoot)
        _parentJoint = _myRobot->_dummyJoint;
}

Link::~Link()
{
    
}

const string& Link::name() const
{
    return KinObject::name();
}

bool Link::name(const string& newName)
{
    if( verb.Assert(!_myRobot->checkForLinkName(newName),
                    verbosity::ASSERT_CRITICAL,
                    "You requested to change link named '"+name()+"' to '"
                    +newName+"', but robot '"+_myRobot->name()+"' already has "
                    "a link with that name!"))
        return false;
    
    StringMap::iterator n = _myRobot->_linkNameToIndex.find(name());
    size_t index = n->second;
    _myRobot->_linkNameToIndex.erase(n);
    _myRobot->_linkNameToIndex[newName] = index;
    
    return KinObject::name(newName);
}

size_t Link::id() const
{
    return _id;
}

void Link::setAsAnchor()
{
    _myRobot->anchorLink(*this);
}

bool Link::belongsTo(const Robot &someRobot) const
{
    if(_myRobot == &someRobot)
        return true;

    return false;
}

void Link::_addChildJoint(Joint *newChild)
{
    _childJoints.push_back(newChild);
    _downstreamJoints.push_back(newChild);
}

void Link::_removeChildJoint(Joint *oldChild)
{
    for(size_t i=0; i<_childJoints.size(); ++i)
    {
        if(_childJoints[i] == oldChild)
        {
            _childJoints.erase(_childJoints.begin()+i);
            break;
        }
    }
    
    for(size_t i=0; i<_downstreamJoints.size(); ++i)
    {
        if(_downstreamJoints[i] == oldChild)
        {
            _downstreamJoints.erase(_downstreamJoints.begin()+i);
            break;
        }
    }
}

void Link::_setParentJoint(Joint *newParent)
{
    _parentJoint = newParent;
    _upstreamJoint = newParent;
}

Link& Link::parentLink()
{
    if( !verb.Assert(!_isRoot, verbosity::ASSERT_CASUAL,
                     "You are requesting the parent link of the root link for "
                     "the robot '"+_myRobot->name()+"'!") )
        return *_myRobot->_dummyLink;
    
    return _parentJoint->parentLink();
}

Joint& Link::parentJoint()
{
    if( !verb.Assert(!_isRoot, verbosity::ASSERT_CASUAL,
                     "You are requesting the parent joint of the root link for "
                     "the robot '"+_myRobot->name()+"'!") )
        return *_myRobot->_dummyJoint;
    
    return *_parentJoint;
}

Link& Link::childLink(size_t num)
{
    if( !verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a child link index which is out of bounds "
                      "for link '"+name()+"' in the robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyLink;
    
    return childJoint(num).childLink();
}

Joint& Link::childJoint(size_t num)
{
    if( !verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a child joint index which is out of bounds "
                      "for link '"+name()+"' in robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyJoint;
    
    return *_childJoints[num];
}

Link& Link::upstreamLink()
{
    if( !verb.Assert(!_isAnchor, verbosity::ASSERT_CASUAL,
                     "You are requesting the upstream link of the anchor -- currently '"
                     +name()+"' -- for the robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyLink;
    
    return _upstreamJoint->upstreamLink();
}

Joint& Link::upstreamJoint()
{
    if( !verb.Assert(!_isAnchor, verbosity::ASSERT_CASUAL,
                     "You are requesting the upstream joint of the anchor -- currently '"
                     +name()+"' -- for the robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyJoint;
    
    return *_upstreamJoint;
}

Link& Link::downstreamLink(size_t num)
{
    if( !verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a downstream link index which is out of bounds "
                      "for link '"+name()+"' in robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyLink;
    
    return downstreamJoint(num).downstreamLink();
}

Joint& Link::downstreamJoint(size_t num)
{
    if( !verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a downstream joint index which is out of bounds "
                      "for link '"+name()+"' in robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyJoint;
    
    return *_downstreamJoints[num];
}

Robot& Link::robot()
{
    return *_myRobot;
}
