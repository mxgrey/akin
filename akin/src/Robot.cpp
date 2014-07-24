
#include "akin/Link.h"
#include "akinUtils/urdfParsing.h"

using namespace akin;
using namespace std;

Robot::Robot(akin::Frame& referenceFrame, verbosity::verbosity_level_t report_level) :
    _com(referenceFrame, "CenterOfMass")
{
    _initializeRobot(referenceFrame, report_level);
}

void Robot::_initializeRobot(akin::Frame& referenceFrame, verbosity::verbosity_level_t report_level)
{
    verb.level = report_level;
    _enforceJointLimits = true;
    _name = "default_robot";

    _dummyLink = new Link(this, Frame::World(), "dummy", 0, false);
    _dummyLink->_isDummy = true;

    _dummyJoint = new Joint(this, -1, "dummy", _dummyLink, _dummyLink);
    _dummyJoint->_myType = Joint::DUMMY;
    _dummyJoint->_changeParentLink(_dummyLink);
    _dummyJoint->_childLink = _dummyLink;
    
    _dummyManip = new Manipulator(this, *_dummyLink, "dummy");
    _dummyManip->_isDummy = true;
    
    // Create the root degrees of freedom
    Link* pos_x_link = new Link(this, referenceFrame, "DOF_POS_X", DOF_POS_X, true);
    _root_dummy_links.push_back(pos_x_link);
    _linkNameToIndex[pos_x_link->name()] = pos_x_link->id();
    
    Link* pos_y_link = new Link(this, *pos_x_link, "DOF_POS_Y", DOF_POS_Y, false);
    _root_dummy_links.push_back(pos_y_link);
    _linkNameToIndex[pos_y_link->name()] = pos_y_link->id();
    
    Joint* pos_x_joint = new Joint(this, DOF_POS_X, "DOF_POS_X", pos_x_link, pos_y_link, 
                                   Transform::Identity(), Axis(1,0,0), Joint::PRISMATIC);
    _root_dummy_joints.push_back(pos_x_joint);
    _jointNameToIndex[pos_x_joint->name()] = DOF_POS_X;
    
    //////////
    
    Link* pos_z_link = new Link(this, *pos_y_link, "DOF_POS_Z", DOF_POS_Z, false);
    _root_dummy_links.push_back(pos_z_link);
    _linkNameToIndex[pos_z_link->name()] = pos_z_link->id();
    
    Joint* pos_y_joint = new Joint(this, DOF_POS_Y, "DOF_POS_Y", pos_y_link, pos_z_link,
                                   Transform::Identity(), Axis(0,1,0), Joint::PRISMATIC);
    _root_dummy_joints.push_back(pos_y_joint);
    _jointNameToIndex[pos_y_joint->name()] = DOF_POS_Y;
    
    //////////
    
    Link* rot_x_link = new Link(this, *pos_z_link, "DOF_ROT_X", DOF_ROT_X, false);
    _root_dummy_links.push_back(rot_x_link);
    _linkNameToIndex[rot_x_link->name()] = rot_x_link->id();
    
    Joint* pos_z_joint = new Joint(this, DOF_POS_Z, "DOF_POS_Z", pos_z_link, rot_x_link,
                                   Transform::Identity(), Axis(0,0,1), Joint::PRISMATIC);
    _root_dummy_joints.push_back(pos_z_joint);
    _jointNameToIndex[pos_z_joint->name()] = DOF_POS_Z;
    
    //////////
    
    Link* rot_y_link = new Link(this, *rot_x_link, "DOF_ROT_Y", DOF_ROT_X, false);
    _root_dummy_links.push_back(rot_y_link);
    _linkNameToIndex[rot_y_link->name()] = rot_y_link->id();
    
    Joint* rot_x_joint = new Joint(this, DOF_ROT_X, "DOF_ROT_X", rot_x_link, rot_y_link,
                                   Transform::Identity(), Axis(1,0,0), Joint::REVOLUTE);
    _root_dummy_joints.push_back(rot_x_joint);
    _jointNameToIndex[rot_x_joint->name()] = DOF_ROT_X;
    
    //////////
    
    Link* rot_z_link = new Link(this, *rot_y_link, "DOF_ROT_Z", DOF_ROT_Z, false);
    _root_dummy_links.push_back(rot_z_link);
    _linkNameToIndex[rot_z_link->name()] = rot_z_link->id();
    
    Joint* rot_y_joint = new Joint(this, DOF_ROT_Y, "DOF_ROT_Y", rot_y_link, rot_z_link,
                                   Transform::Identity(), Axis(0,1,0), Joint::REVOLUTE);
    _root_dummy_joints.push_back(rot_y_joint);
    _jointNameToIndex[rot_y_joint->name()] = DOF_ROT_Y;
    
    //////////
    
    // DOF_ROT_Z joint gets created when createRootLink is called
    
    for(size_t i=0; i<_root_dummy_links.size(); ++i)
    {
        _root_dummy_links[i]->addVisual(Geometry());
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
    
    for(size_t i=0; i<_root_dummy_links.size(); ++i)
    {
        delete _root_dummy_links[i];
    }
    
    for(size_t j=0; j<_root_dummy_joints.size(); ++j)
    {
        delete _root_dummy_joints[j];
    }
}

Frame& Robot::refFrame()
{
    return _root_dummy_links.front()->refFrame();
}

const Frame& Robot::const_refFrame() const
{
    return const_cast<Robot*>(this)->refFrame();
}

bool Robot::changeRefFrame(Frame &newRefFrame)
{
    return _root_dummy_links.front()->changeRefFrame(newRefFrame);
}

const KinTranslation& Robot::com() const
{
    _com = com(const_anchorLink(), _com.refFrame());
    return _com;
}

Translation Robot::com(const Link& startLink, const Frame& referenceFrame,
                       Crawler::policy p) const
{
    Translation com_;
    double mass_ = 0, lmass = 0;
    
    _crawler.reset(startLink, p);
    const Link* current = _crawler.nextLink();
    while(current != NULL)
    {
        lmass = current->mass;
        com_ += lmass*current->com.withRespectTo(referenceFrame);
        mass_ += lmass;
        
        for(size_t i=0; i<current->numManips(); ++i)
        {
            const Manipulator& manip_ = current->const_manip(i);
            lmass = manip_.mass();
            com_ += lmass*manip_.com().withRespectTo(referenceFrame);
            mass_ += lmass;
        }
        
        current = _crawler.nextLink();
    }
    
//    if( verb.Assert(_mass > 0, verbosity::ASSERT_CASUAL,
//                    "Center of Mass requested for Robot '"+name()+"' starting at Link '"
//                    +startLink.name()+"', but there was no mass at all!") )
    if( _mass > 0 )
        com_ = com_/mass_;
    else
        com_.setZero();
    
    return com_;
}

double Robot::mass(const Link &startLink, Crawler::policy p) const
{
    double mass_ = 0;
    
    _crawler.reset(startLink, p);
    const Link* current = _crawler.nextLink();
    while(current != NULL)
    {
        mass_ += current->mass;
        
        for(size_t i=0; i<current->numManips(); ++i)
        {
            const Manipulator& manip_ = current->const_manip(i);
            mass_ += manip_.mass();
        }
        
        current = _crawler.nextLink();
    }
    
    return mass_;
}

const double& Robot::mass() const
{
    _mass = mass(const_anchorLink());
    return _mass;
}

void Robot::name(string newName)
{
    _name = newName;
}

bool Robot::createRootLink(string rootLinkName)
{
    if(!verb.Assert(_links.size()==0, verbosity::ASSERT_CASUAL,
                "Cannot create a root link, because one already exists!"))
        return false;

    Link* rootLink = new Link(this, *_root_dummy_links.back(), rootLinkName, 0, true);
    _insertLink(rootLink);
    _root = rootLink;
    _anchor = rootLink;
    
    Joint* rot_z_joint = new Joint(this, DOF_ROT_Z, "DOF_ROT_Z",
                                   _root_dummy_links.back(), rootLink,
                                   Transform::Identity(), Axis(0,0,1), Joint::REVOLUTE);
    _root_dummy_joints.push_back(rot_z_joint);
    _jointNameToIndex[rot_z_joint->name()] = DOF_ROT_Z;
    
    _com.changeRefFrame(*rootLink);
    
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

size_t Robot::getJointIndex(const string &jointName) const
{
    StringMap::const_iterator j = _jointNameToIndex.find(jointName);
    if( !verb.Assert( j != _jointNameToIndex.end(),
                      verbosity::ASSERT_CASUAL,
                     "There is no joint named '"+jointName+"' in the robot named '"
                     +name()+"'!") )
        return DOF_INVALID;
    
    return j->second;
}

Joint& Robot::joint(const string &jointName)
{
    return joint(getJointIndex(jointName));
}

const Joint& Robot::const_joint(const string &jointName) const
{
    return const_joint(getJointIndex(jointName));
}

Joint& Robot::joint(size_t jointNum)
{
    if( DOF_POS_X <= jointNum && jointNum <= DOF_ROT_Z )
    {
        return *_root_dummy_joints[jointNum-DOF_POS_X];
    }
    
    if( !verb.Assert( jointNum < _joints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a joint index which is out of bounds "
                      "on the robot named '"+name()+"'") )
        return *_dummyJoint;
    
    return *_joints[jointNum];
}

const Joint& Robot::const_joint(size_t jointNum) const
{
    return const_cast<Robot*>(this)->joint(jointNum);
}

size_t Robot::getLinkIndex(const string &linkName) const
{
    StringMap::const_iterator i = _linkNameToIndex.find(linkName);
    if( !verb.Assert( i != _linkNameToIndex.end(),
                      verbosity::ASSERT_CASUAL,
                      "There is no link named '"+linkName+"' in the robot named '"
                      +name()+"'!") )
        return DOF_INVALID;
    
    return i->second;
}

Link& Robot::link(const string &linkName)
{
    return link(getLinkIndex(linkName));
}

const Link& Robot::const_link(const string &linkName) const
{
    return const_link(getLinkIndex(linkName));
}

Link& Robot::anchorLink() { return *_anchor; }
const Link& Robot::const_anchorLink() const { return *_anchor; }

Link& Robot::link(size_t linkNum)
{
    if( DOF_POS_X <= linkNum && linkNum <= DOF_ROT_Z )
    {
        return *_root_dummy_links[linkNum-DOF_POS_X];
    }
    
    if( !verb.Assert( linkNum < _links.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a link index which is out of bounds "
                      "on the robot named '"+name()+"'") )
        return *_dummyLink;
    
    return *_links[linkNum];
}

const Link& Robot::const_link(size_t linkNum) const
{
    return const_cast<Robot*>(this)->link(linkNum);
}

int Robot::addManipulator(Frame &attachment, const string &name, Transform relativeTf)
{
    if(checkForManipName(name))
        return -1;
    
    Manipulator* newManip = new Manipulator(this, attachment, name);
    newManip->respectToRef(relativeTf);
    
    _insertManip(newManip);
    if(newManip->parentLink().isDummy())
        return -2;
    
    newManip->parentLink()._manips.push_back(newManip);
    return _manips.size()-1;
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

void Robot::_insertManip(Manipulator* newManip)
{
    _manips.push_back(newManip);
    _manipNameToIndex[newManip->name()] = _manips.size()-1;
}

bool Robot::owns(const Joint &someJoint) const
{
    for(size_t i=0; i<_joints.size(); ++i)
        if(_joints[i] == &someJoint)
            return true;

    return false;
}

bool Robot::owns(const Link &someLink) const
{
    for(size_t i=0; i<_links.size(); ++i)
        if(_links[i] == &someLink)
            return true;

    return false;
}

bool Robot::owns(const Manipulator &someManip) const
{
    for(size_t i=0; i<_manips.size(); ++i)
        if(_manips[i] == &someManip)
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

bool Robot::checkForManipName(const string &name) const
{
    StringMap::const_iterator n = _manipNameToIndex.find(name);
    if( n == _manipNameToIndex.end() )
        return false;
    
    return true;
}

void Robot::anchorLink(Link &)
{
    // TODO
}

void Robot::anchorLink(size_t )
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

std::vector<const Link*> Robot::Crawler::getPath(const Link &startLink, const Link &endLink)
{
    Crawler crawl(startLink, endLink);
    
    return crawl._path;
}

std::vector<size_t> Robot::Crawler::getIdPath(const Link &startLink, const Link &endLink)
{
    Crawler crawl(startLink, endLink);
    
    std::vector<size_t> result; result.reserve(crawl._path.size());
    const Link* link = crawl.nextLink();
    while(link != NULL)
    {
        result.push_back(link->id());
    }
    
    return result;
}

Robot::Crawler::Crawler()
{
    _init();
    _p = INACTIVE;
}

Robot::Crawler::Crawler(const akin::Link& startLink, const akin::Link& endLink)
{
    _init();
    reset(startLink, endLink);
}

Robot::Crawler::Crawler(const akin::Link& startLink, policy p)
{
    _init();
    reset(startLink, p);
}

void Robot::Crawler::_init()
{
    _recorder.reserve(100);
    _path.reserve(100);
    _temp.reserve(100);
    stopAtRoot = false;
}

void Robot::Crawler::reset(const akin::Link& startLink, const akin::Link& endLink)
{
    _p = PATH;
    _pathLocation = 0;
    
    _path.clear();
    _temp.clear();
    
    if(!startLink.const_robot().owns(endLink))
        return;
    
    if(startLink.descendsFrom(endLink))
    {
        const Link* current = &startLink;
        while(current != &endLink)
        {
            _temp.push_back(current);
            current = &current->const_parentLink();
        }
        _temp.push_back(current);
        
        for(size_t i=_temp.size(); i>0; --i)
            _path.push_back(_temp[i-1]);
    }
    else if(endLink.descendsFrom(startLink))
    {
        const Link* current = &startLink;
        while(current != &endLink)
        {
            _path.push_back(current);
            current = &current->const_parentLink();
        }
        _path.push_back(current);
    }
    else
    {
        const Link* current = &startLink;
        while(!current->isRoot())
        {
            _path.push_back(current);
            current = &current->const_parentLink();
        }
        _path.push_back(current);
        
        current = &endLink;
        while(!current->isRoot())
        {
            _temp.push_back(current);
            current = &current->const_parentLink();
        }
        
        for(size_t i=_temp.size(); i>0; --i)
            _path.push_back(_temp[i-1]);
    }
}

void Robot::Crawler::reset(const Link &startLink, policy p)
{
    if(p==PATH)
    {
        _p = INACTIVE;
        return;
    }
    
    _p = p;
    _recorder.clear();
    _first = &startLink;
    _finished = false;
}

const Link* Robot::Crawler::nextLink()
{
    if(_p == PATH)
    {
        if(_pathLocation < _path.size())
        {
            ++_pathLocation;
            return _path[_pathLocation-1];
        }
        else
            return NULL;
    }
    else if(_p == DOWNSTREAM)
    {
        if(_finished)
            return NULL;
        
        if(_recorder.size()==0)
        {
            _recorder.push_back(Recorder(_first, 0));
            return _first;
        }
        
        while(_recorder.size() > 0)
        {
            Recorder& t = _recorder.back();
            if(t.count < (int)(t.link->numDownstreamLinks()))
            {
                _recorder.push_back(Recorder(&t.link->const_downstreamLink(t.count), 0));
                ++t.count;
                return _recorder.back().link;
            }
            else
            {
                _recorder.pop_back();
            }
        }
        
        _finished = true;
    }
    else if(_p == UPSTREAM)
    {
        if(_finished)
            return NULL;
        
        if(_recorder.size()==0)
        {
            _recorder.push_back(Recorder(_first, -1));
            return _first;
        }
        
        while(_recorder.size() > 0)
        {
            Recorder& t = _recorder.back();
            if(t.count == -1)
            {
                if(t.link->const_upstreamLink().isDummy())
                {
                    ++t.count;
                }
                else if(_recorder.size() == 1)
                {
                    _recorder.push_back(Recorder(&t.link->const_upstreamLink(), -1));
                    ++t.count;
                    return _recorder.back().link;
                }
                else if(&t.link->const_upstreamLink() != _recorder[_recorder.size()-2].link)
                {
                    _recorder.push_back(Recorder(&t.link->const_upstreamLink(), -1));
                    ++t.count;
                    return _recorder.back().link;
                }
                else
                {
                    ++t.count;
                }
            }
            else if(t.count < (int)(t.link->numDownstreamLinks()))
            {
                if(_recorder.size()==1)
                {
                    ++t.count;
                }
                else if(&t.link->const_downstreamLink(t.count) != _recorder[_recorder.size()-2].link)
                {
                    _recorder.push_back(Recorder(&t.link->const_downstreamLink(t.count), -1));
                    ++t.count;
                    if(_recorder.back().link->isDummy())
                        std::cout << "Dummy found as downstream #" << t.count-1 << " of " 
                                  << t.link->name() << " which has " 
                                  << t.link->numDownstreamLinks() << " downstreams" << std::endl;
                    return _recorder.back().link;
                }
                else
                {
                    ++t.count;
                }
            }
            else
            {
                _recorder.pop_back();
            }
        }
        
        _finished = true;
    }
    
    return NULL;
}

Robot::Crawler::Recorder::Recorder() :
    link(NULL),
    count(0)
{
    
}

Robot::Crawler::Recorder::Recorder(const Link *link_, size_t count_) :
    link(link_),
    count(count_)
{
    
}
