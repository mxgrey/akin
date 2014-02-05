
#include "Link.h"
#include "urdfParsing.h"

using namespace akin;
using namespace std;

Robot::Robot(construction_t method, string construction_info,
             Frame &rootReferenceFrame,
             verbosity::verbosity_level_t report_level)
{
    StringArray construction_array;
    construction_array.push_back(construction_info);
    _initializeRobot(method, construction_array, rootReferenceFrame, report_level);
}

Robot::Robot(construction_t method, StringArray construction_info,
             Frame &rootReferenceFrame,
             verbosity::verbosity_level_t report_level)
{
    _initializeRobot(method, construction_info, rootReferenceFrame, report_level);
}

void Robot::_initializeRobot(construction_t method, StringArray construction_info, Frame &rootReferenceFrame, verbosity::verbosity_level_t report_level)
{
    verb.level = report_level;
    _enforceJointLimits = true;
    _name = "some_robot";

    _dummyLink = new Link(this, Frame::World(), "dummy", 0, false);
    _dummyLink->_isDummy = true;

    _dummyJoint = new Joint(this, -1, "dummy", _dummyLink, _dummyLink);
    _dummyJoint->_myType = Joint::DUMMY;
    _dummyJoint->_changeParentLink(_dummyLink);
    _dummyJoint->_childLink = _dummyLink;

    if(URDF_FILE == method)
    {
        if(construction_info.size() < 2)
        {
            cout << "To parse a URDF_FILE, construction_info requires two entries:\n"
                    << "(1) URDF Filename\n"
                    << "(2) URDF Base Package Directory" << endl;
            return;
        }
#ifdef HAVE_URDFPARSING
        akinUtils::loadURDF(*this, construction_info[0], construction_info[1], rootReferenceFrame);
#else  // HAVE_URDF_PARSING
        cout << "I cannot parse the URDF file '" << construction_info[1]
             << "' because urdfdom and/or urdfdom_headers were not installed "
             << "when akin was compiled on your computer!" << endl;
#endif // HAVE URDF_PARSING
    }
    else if(URDF_STRING == method)
    {
        if(construction_info.size() < 2)
        {
            cout << "To parse a URDF_STRING, construction_info requires two entries:\n"
                    << "(1) URDF Text String\n"
                    << "(2) URDF Base Package Directory" << endl;
            return;
        }
#ifdef HAVE_URDFPARSING
        robotPackageDirectory = construction_info[1];
        akinUtils::loadURDFstring(*this, construction_info[0], rootReferenceFrame);
#else  // HAVE_URDFPARSING
        cout << "I cannot parse the URDF string for this robot, because "
             << "urdfdom and/or urdfom_headers were not installed "
             << "when akin was compiled on your computer!" << endl;
#endif // HAVE_URDFPARSING
    }
    else
    {
        if(construction_info.size() > 0)
            createRootLink(construction_info[0], rootReferenceFrame);
        else
            createRootLink("root_link", rootReferenceFrame);
    }
}

Robot::~Robot()
{
    for(size_t i=0; i<_links.size(); ++i)
    {
        delete _links[i];
    }
    
    for(size_t j=0; j<_joints.size(); ++j)
    {
        delete _joints[j];
    }
}

void Robot::name(string newName)
{
    _name = newName;
}

bool Robot::createRootLink(string rootLinkName, Frame& referenceFrame)
{
    if(!verb.Assert(_links.size()==0, verbosity::ASSERT_CASUAL,
                "Cannot create a root link, because one already exists!"))
        return false;

    Link* rootLink = new Link(this, referenceFrame, rootLinkName, 0, true);
    _insertLink(rootLink);
    _root = rootLink;
    _anchor = rootLink;
    
    return true;
}


int Robot::createJointLinkPair(Link &parentLink,
                               const string &newLinkName, const string &newJointName,
                               const Transform &baseTransform, const Axis &jointAxis,
                               Joint::Type jointType, double minJointValue, double maxJointValue)
{
    if(!verb.Assert(!parentLink.isDummy(), verbosity::ASSERT_CRITICAL,
                    "Error: You wanted to make '"+newLinkName+"' a child of a dummy link in robot '"+name()+"'!"))
        return -1;
    
    if(!verb.Assert(parentLink.belongsTo(*this), verbosity::ASSERT_CRITICAL,
                "Error: You wanted to make '"+newLinkName+"' a child of '"+parentLink.name()+
                "', but '"+parentLink.name()+"' does not belong to the robot '"+name()+"'!"))
        return -2;
    
    if(!verb.Assert(!checkForLinkName(newLinkName), verbosity::ASSERT_CRITICAL,
                    "Error: You wanted to create a new link named '"+newLinkName+
                    "' but a link by that name already exists in the robot named '"
                    +name()+"'!"))
        return -3;
    
    if(!verb.Assert(!checkForJointName(newJointName), verbosity::ASSERT_CRITICAL,
                    "Error: You wanted to create a new joint named '"+newJointName+
                    "' but a joint by that name already exists in the robot named '"
                    +name()+"'!"))
        return -4;

    Link* newLink = new Link(this, parentLink, newLinkName, _links.size(), false);
    _insertLink(newLink);

    Joint* newJoint = new Joint(this, _joints.size(), newJointName,
                                &parentLink, newLink,
                                baseTransform, jointAxis, jointType,
                                minJointValue, maxJointValue);
    _insertJoint(newJoint);
    
    parentLink._addChildJoint(newJoint);
    newLink->_setParentJoint(newJoint);
    
    return _links.size()-1;
}

int Robot::createJointLinkPair(size_t parentLinkID,
                               const string &newLinkName, const string &newJointName,
                               const Transform &baseTransform, const Axis &jointAxis,
                               Joint::Type jointType, double minJointValue, double maxJointValue)
{
    return createJointLinkPair(link(parentLinkID),
                               newLinkName, newJointName,
                               baseTransform, jointAxis,
                               jointType, minJointValue, maxJointValue);
}

void Robot::removeConnection(string &jointName, bool fillInGap)
{
    removeConnection(joint(jointName).id(), fillInGap);
}

void Robot::removeConnection(size_t jointNum, bool fillInGap)
{
    if(fillInGap)
    {
        Link& savedAnchor = anchorLink();
        anchorLink(link(0));
        
        Joint& lostJoint = joint(jointNum);
        Link& lostLink = lostJoint.childLink();
        
        Link& parentLink = lostJoint.parentLink();
        
        for(size_t i=0; i<lostLink.numChildJoints(); ++i)
        {
            Joint& childJoint = lostLink.childJoint(i);
            childJoint._baseTransform = lostJoint._baseTransform * childJoint._baseTransform;
            childJoint._changeParentLink( lostJoint._parentLink );
            childJoint._computeRefTransform();
            
            parentLink._addChildJoint(&childJoint);
            
            Link& childLink = childJoint.childLink();
            childLink.changeRefFrame(parentLink);
        }
        
        parentLink._removeChildJoint(&lostJoint);
        _deleteConnection(&lostJoint);
        
        anchorLink(savedAnchor);
    }
    else
    {
        Joint& lostJoint = joint(jointNum);
        Link& parentLink = lostJoint.parentLink();
        
        Link* savedAnchor = &anchorLink();
        anchorLink(link(0));
        if(savedAnchor->descendsFrom(parentLink))
            savedAnchor = &parentLink;
        
        parentLink._removeChildJoint(&lostJoint);
        _recursiveDeleteConnection(&lostJoint);
        
        anchorLink(*savedAnchor);
    }
}

void Robot::_recursiveDeleteConnection(Joint *deadJoint)
{
    Link* deadLink = &(deadJoint->childLink());
    for(size_t i=0; i<deadLink->numChildJoints(); ++i)
    {
        _recursiveDeleteConnection(&(deadLink->childJoint(i)));
    }
    _deleteConnection(deadJoint);
}

void Robot::_deleteConnection(Joint *deadJoint)
{
    Link* deadLink = &(deadJoint->childLink());
    
    for(size_t i=0; i<_joints.size(); ++i)
    {
        if(_joints[i] == deadJoint)
        {
            _joints[i] = _dummyJoint;
            break;
        }
    }
    
    for(size_t i=0; i<_links.size(); ++i)
    {
        if(_links[i] == deadLink)
        {
            _links[i] = _dummyLink;
            break;
        }
    }
    
    delete deadLink;
    delete deadJoint;
}

Joint& Robot::joint(const string &jointName)
{
    StringMap::const_iterator j = _jointNameToIndex.find(jointName);
    if( !verb.Assert( j != _jointNameToIndex.end(),
                      verbosity::ASSERT_CASUAL,
                     "There is no joint named '"+jointName+"' in the robot named '"
                     +name()+"'!") )
        return *_dummyJoint;
    
    return *_joints.at(j->second);
}

Joint& Robot::joint(size_t jointNum)
{
    if( !verb.Assert( jointNum < _joints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a joint index which is out of bounds "
                      "on the robot named '"+name()+"'") )
        return *_dummyJoint;
    
    return *_joints[jointNum];
}

Link& Robot::link(const string &linkName)
{
    StringMap::const_iterator i = _linkNameToIndex.find(linkName);
    if( !verb.Assert( i != _linkNameToIndex.end(),
                      verbosity::ASSERT_CASUAL,
                      "There is no link named '"+linkName+"' in the robot named '"
                      +name()+"'!") )
        return *_dummyLink;
    
    return *_links.at(i->second);
}

Link& Robot::link(size_t linkNum)
{
    if( !verb.Assert( linkNum < _links.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a link index which is out of bounds "
                      "on the robot named '"+name()+"'") )
        return *_dummyLink;
    
    return *_links[linkNum];
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

bool Robot::belongsTo(const Joint &someJoint) const
{
    for(size_t i=0; i<_joints.size(); ++i)
        if(_joints[i] == &someJoint)
            return true;

    return false;
}

bool Robot::belongsTo(const Link &someLink) const
{
    for(size_t i=0; i<_links.size(); ++i)
        if(_links[i] == &someLink)
            return true;

    return false;
}

bool Robot::checkForLinkName(const string& name) const
{
    StringMap::const_iterator n = _linkNameToIndex.find(name);
    if( n == _linkNameToIndex.end() )
        return false;
    
    return true;
}

bool Robot::checkForJointName(const string& name) const
{
    StringMap::const_iterator n = _jointNameToIndex.find(name);
    if( n == _jointNameToIndex.end() )
        return false;
    
    return true;
}

void Robot::anchorLink(Link &newAnchor)
{
    // TODO
}

void Robot::anchorLink(size_t linkNum)
{
    // TODO
}

void Robot::enforceJointLimits(bool enforce)
{
    _enforceJointLimits = enforce;
    
    if(enforce)
    {
        for(size_t i=0; i<numJoints(); ++i)
        {
            if(!joint(i).withinLimits())
                joint(i).value(joint(i).value());
        }
    }
}


