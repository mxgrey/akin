
#include "Link.h"

using namespace akin;
using namespace std;

Robot::Robot()
{
    _dummyLink = new Link(*this, Frame::World(), "dummy", false);
    _dummyLink->_isDummy = true;

    _dummyJoint = new Joint(this);
    _dummyJoint->_myType = Joint::DUMMY;
}


Robot::createRootLink(string rootLinkName, Frame& referenceFrame)
{
    verb.Assert(_links.size()==0, verbosity::ASSERT_CASUAL,
                "Cannot create a root link, because one already exists!");

    Link* rootLink = new Link(*this, referenceFrame, rootLinkName, true);
    _insertLink(rootLink);
}


Robot::createJointLinkPair(Link &parentLink,
                           string newLinkName, string newJointName,
                           const Transform &baseTransform, const Axis &jointAxis,
                           Joint::Type jointType, double minJointValue, double maxJointValue)
{
    verb.Assert(parentLink.belongsTo(this), verbosity::ASSERT_CRITICAL,
                "You wanted to make '"+newLinkName+"' a child of '"+parentLink.name()+
                "', but '"+parentLink.name()+"' does not belong to the robot '"+name()+"'");

    Link* newLink = new Link(*this, parentLink, newLinkName, false);
    _insertLink(newLink);

    Joint* newJoint = new Joint(this, _joints.size(), newJointName, &parentLink, newLink,
                                baseTransform, jointAxis, jointType,
                                minJointValue, maxJointValue);
    _insertJoint(newJoint);
}


void Robot::_insertLink(Link *newLink)
{
    _links.push_back(newLink);
    _linkNameToIndex[newLink->name()] = _links.size()-1;
}

void Robot::_insertJoint(Joint *newJoint)
{
    _joints.push_back(newJoint);
    _jointNameToIndex[newJoint->name()] = _joints.size()-1;
}

Robot::belongsTo(const Joint &someJoint) const
{
    for(size_t i=0; i<_joints.size(); ++i)
        if(_joints[i] == &someJoint)
            return true;

    return false;
}

Robot::belongsTo(const Link &someLink) const
{
    for(size_t i=0; i<_links.size(); ++i)
        if(_links[i] == &someLink)
            return true;

    return false;
}
