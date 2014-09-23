
#include "akin/Robot.h"

using namespace akin;
using namespace std;

Link::Link(Robot *mRobot, Frame &referenceFrame, string linkName, size_t mID, bool root) :
    Body(referenceFrame, linkName),
    _id(mID),
    _isRoot(root),
    _isDummy(false),
    _parentJoint(NULL),
    _upstreamJoint(NULL),
    _myRobot(mRobot)
{
    _isLink = true;
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
    if( _myRobot->verb.Assert(!_myRobot->checkForLinkName(newName),
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

bool Link::isAnchor() const { return (this == &_myRobot->anchorLink()); }

void Link::setAsAnchor()
{
    // TODO
//    _myRobot->anchorLink(*this);
}

bool Link::isRoot() const { return _isRoot; }

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
    return parentJoint().parentLink();
}

const Link& Link::parentLink() const
{
    return const_cast<Link*>(this)->parentLink();
}

Joint& Link::parentJoint()
{
    if( !_myRobot->verb.Assert(_parentJoint != NULL, verbosity::ASSERT_CRITICAL,
                     "Link named '"+name()+"' has a NULL Parent Joint!"))
        return *_myRobot->_dummyJoint;
    
    return *_parentJoint;
}

const Joint& Link::parentJoint() const
{
    return const_cast<Link*>(this)->parentJoint();
}

Link& Link::childLink(size_t num)
{
    if( !_myRobot->verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a child link index which is out of bounds "
                      "for link '"+name()+"' in the robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyLink;
    
    return childJoint(num).childLink();
}

const Link& Link::childLink(size_t num) const
{
    return const_cast<Link*>(this)->childLink(num);
}

Joint& Link::childJoint(size_t num)
{
    if( !_myRobot->verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a child joint index which is out of bounds "
                      "for link '"+name()+"' in robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyJoint;
    
    return *_childJoints[num];
}

const Joint& Link::childJoint(size_t num) const
{
    return const_cast<Link*>(this)->childJoint(num);
}

size_t Link::numChildJoints() const { return _childJoints.size(); }
size_t Link::numChildLinks() const { return _childJoints.size(); }

Joint& Link::upstreamJoint()
{
    if( !_myRobot->verb.Assert(_upstreamJoint != NULL, verbosity::ASSERT_CRITICAL,
                     "Link named '"+name()+"' has a NULL upstream joint!"))
        return *_myRobot->_dummyJoint;
    
    return *_upstreamJoint;
}

const Joint& Link::upstreamJoint() const
{
    return const_cast<Link*>(this)->upstreamJoint();
}

Link& Link::upstreamLink()
{
    return upstreamJoint().upstreamLink();
}

const Link& Link::upstreamLink() const
{
    return const_cast<Link*>(this)->upstreamLink();
}

Joint& Link::downstreamJoint(size_t num)
{
    if( !_myRobot->verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a downstream joint index which is out of bounds "
                      "for link '"+name()+"' in robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyJoint;
    
    return *_downstreamJoints[num];
}

const Joint& Link::downstreamJoint(size_t num) const
{
    return const_cast<Link*>(this)->downstreamJoint(num);
}

Link& Link::downstreamLink(size_t num)
{
    if( !_myRobot->verb.Assert( num < _childJoints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a downstream link index which is out of bounds "
                      "for link '"+name()+"' in robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyLink;
    
    return downstreamJoint(num).downstreamLink();
}

const Link& Link::downstreamLink(size_t num) const
{
    return const_cast<Link*>(this)->downstreamLink(num);
}

size_t Link::numDownstreamJoints() const { return _downstreamJoints.size(); }
size_t Link::numDownstreamLinks() const { return _downstreamJoints.size(); }

Manipulator& Link::manip(size_t manipNum)
{
    if( !_myRobot->verb.Assert( manipNum < _manips.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a manipulator index which is out of bounds "
                      "for link '"+name()+"' in robot '"+_myRobot->name()+"'!"))
        return *_myRobot->_dummyManip;
    
    return *_manips[manipNum];
}

const Manipulator& Link::manip(size_t manipNum) const
{
    return const_cast<Link*>(this)->manip(manipNum);
}

size_t Link::numManips() const
{
    return _manips.size();
}

Robot& Link::robot()
{
    return *_myRobot;
}

const Robot& Link::robot() const
{
    return *_myRobot;
}

bool Link::isDummy() const { return _isDummy; }

std::ostream& operator<<(std::ostream& oStrStream, const akin::Link& someLink)
{
    oStrStream << "Link named '" << someLink.name() << "' ";
    if(someLink.isRoot())
    {
        oStrStream << "is the root link ";
    }
    if(someLink.isAnchor())
    {
        if(someLink.isRoot())
            oStrStream << "and ";
        oStrStream << "is the anchor link ";
    }
    
    if(!someLink.isRoot())
    {
        if(someLink.isAnchor())
            oStrStream << "with ";
        else
            oStrStream << " has ";
        oStrStream << "parent joint " << someLink.parentJoint().name();
    }
    
    oStrStream << "\n";
    
    if(someLink.numChildJoints() == 0)
    {
        oStrStream << "This link has no child joints";
    }
    else
    {
        oStrStream << "Child joints are: ";
        for(size_t i=0; i<someLink.numChildJoints(); ++i)
        {
            oStrStream << someLink.childJoint(i).name();
            if(i+1 < someLink.numChildJoints())
                oStrStream << ", ";
        }
    }
    
    oStrStream << "\n" << (akin::Body&)someLink;
    
    return oStrStream;
}
