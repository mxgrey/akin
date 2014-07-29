
#include "akin/Robot.h"
#include "akin/RobotConstraint.h"

using namespace akin;
using namespace std;

Manipulator::Manipulator(Robot *robot, Frame &referenceFrame, const string &manipName) :
    Frame(referenceFrame, manipName),
    _point(*this, manipName+"_point"),
    _com(*this, manipName+"_com"),
    _myRobot(robot)
{
    _findParentLink();
    if(!const_parentLink().isDummy())
    {
        resetConstraints();
    }
    else
    {
        for(size_t i=0; i<NUM_MODES; ++i)
        {
            _constraints[i] = new NullManipConstraint;
            _ownConstraint[i] = true;
        }
    }
}

Manipulator::~Manipulator()
{
    for(size_t i=0; i<NUM_MODES; ++i)
    {
        if(_ownConstraint[i])
            delete _constraints[i];
    }
}

ManipConstraintBase& Manipulator::constraint(Mode mode)
{
    if( mode < 0 || NUM_MODES <= mode)
        return *_constraints[0];
    return *_constraints[mode];
}

void Manipulator::setConstraint(Mode mode, ManipConstraintBase *newConstraint, bool giveOwnership)
{
    if( !_myRobot->verb.Assert(0 <= mode || mode < NUM_MODES, verbosity::ASSERT_CRITICAL,
                               "Trying to set constraint for invalid mode ("+to_string((int)mode)
                               +", Max:"+to_string((int)NUM_MODES)+")"))
        return;
    
    if(_ownConstraint[mode])
        delete _constraints[mode];
    _constraints[mode] = newConstraint;
    _ownConstraint[mode] = giveOwnership;
}

void Manipulator::resetConstraint(Mode mode)
{
    if( !_myRobot->verb.Assert(0 <= mode || mode < NUM_MODES, verbosity::ASSERT_CRITICAL,
                               "Trying to reset constraint for invalid mode ("+to_string((int)mode)
                               +", Max:"+to_string((int)NUM_MODES)+")"))
        return;
    
    ManipConstraintBase* constraint;
    if(FREE==mode)
    {
        constraint = new NullManipConstraint;
    }
    else if(LINKAGE==mode)
    {
        std::vector<size_t> joints;
        const Joint* current = &const_parentLink().const_parentJoint();
        do {
            
            joints.push_back(current->id());
            current = &current->const_parentJoint();
            
        } while(current->numChildJoints()==1 && !current->const_childLink().isAnchor());
        
        constraint = new ManipConstraintX(joints.size(), *this, joints);
    }
    else if(FULLBODY==mode)
    {
        std::vector<size_t> joints = Robot::Explorer::getIdPath(_myRobot->const_joint(DOF_POS_X),
                                                            const_parentLink().const_parentJoint());
        constraint = new ManipConstraintX(joints.size(), *this, joints);
    }
    else if(CUSTOM==mode)
    {
        constraint = new NullManipConstraint;
    }
    
    setConstraint(mode, constraint, true);
}

void Manipulator::resetConstraints()
{
    for(size_t i=0; i<NUM_MODES; ++i)
        resetConstraint((Mode)i);
}

const KinTranslation& Manipulator::point() const { return _point; }

int Manipulator::attachItem(Body* item)
{
    for(size_t i=0; i<_items.size(); ++i)
    {
        if(item == _items[i])
            return i;
    }
    
    if(!item->changeRefFrame(*this))
        return -1;
    
    _items.push_back(item);
    return _items.size()-1;
}

Body* Manipulator::item(size_t itemNum)
{
    if( !verb.Assert(itemNum<_items.size(), verbosity::ASSERT_CASUAL,
                     "Trying to access item #"+to_string(itemNum)+" held by manip '"
                     +name()+"', but it only has "+to_string(_items.size())+" entries!"))
        return NULL;
    
    return _items[itemNum];
}

const Body* Manipulator::const_item(size_t itemNum) const
{
    return const_cast<Manipulator*>(this)->item(itemNum);
}

size_t Manipulator::numItems() const { return _items.size(); }

bool Manipulator::detachItem(Body* item)
{
    for(size_t i=0; i<_items.size(); ++i)
    {
        if(item == _items[i])
            return detachItem(i);
    }
    
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "Trying to remove item '"+item->name()+"' from the grasp of manip '"
                +name()+"', but it is not being held by that manipulator!");
    return false;
}

bool Manipulator::detachItem(size_t itemNum)
{
    if( !verb.Assert(itemNum < _items.size(), verbosity::ASSERT_CASUAL,
                     "Trying to remove item #"+to_string(itemNum)+" held by manip '"
                     +name()+"', but it only has "+to_string(_items.size())+" entries!"))
        return false;
    
    _items[itemNum]->changeRefFrame(Frame::World());
    _items.erase(_items.begin()+itemNum);
    return true;
}

bool Manipulator::deleteItem(Body* item)
{
    if(detachItem(item))
    {
        delete item;
        return true;
    }
    
    return false;
}

bool Manipulator::deleteItem(size_t itemNum)
{
    Body* oldItem = item(itemNum);
    if(oldItem==NULL)
        return false;
    
    return deleteItem(oldItem);
}


int Manipulator::attachRobot(Robot* robot)
{
    for(size_t i=0; i<_robots.size(); ++i)
    {
        if(robot == _robots[i])
            return i;
    }
    
    if(refFrame().descendsFrom(robot->refFrame()))
        return -1;
    
    if(!robot->changeRefFrame(*this))
        return -2;
    
    _robots.push_back(robot);
    return _robots.size()-1;
}

Robot* Manipulator::robot(size_t robotNum)
{
    if( !verb.Assert(robotNum < _robots.size(), verbosity::ASSERT_CASUAL,
                     "Trying to access robot #"+to_string(robotNum)+" held by manip '"
                     +name()+"', but it only has "+to_string(_robots.size())+"entries!"))
        return NULL;
    
    return _robots[robotNum];
}

const Robot* Manipulator::const_robot(size_t robotNum) const
{
    return const_cast<Manipulator*>(this)->robot(robotNum);
}

size_t Manipulator::numRobots() const { return _robots.size(); }

bool Manipulator::detachRobot(Robot *robot)
{
    for(size_t i=0; i<_robots.size(); ++i)
    {
        if(robot == _robots[i])
            return detachRobot(i);
    }
    
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "Trying to remove robot '"+robot->name()+"' from the grasp of manip '"
                +name()+"', but it is not being held by that manipulator!");
    return false;
}

bool Manipulator::detachRobot(size_t robotNum)
{
    if( !verb.Assert(robotNum < _robots.size(), verbosity::ASSERT_CASUAL,
                     "Trying to remove robot #"+to_string(robotNum)+" held by manip '"
                     +name()+"', but it only has "+to_string(_robots.size())+" entries!"))
        return false;
    
    _robots[robotNum]->changeRefFrame(Frame::World());
    _robots.erase(_robots.begin()+robotNum);    
    return true;
}

Robot& Manipulator::parentRobot()
{
    return *_myRobot;
}

const Robot& Manipulator::const_parentRobot() const
{
    return const_cast<Manipulator*>(this)->parentRobot();
}

Link& Manipulator::parentLink() { return *_myLink; }
const Link& Manipulator::const_parentLink() const { return *_myLink; }

bool Manipulator::changeRefFrame(Frame &newRefFrame)
{
    if(!Frame::changeRefFrame(newRefFrame))
        return false;
    
    _findParentLink();
    
    if(parentLink().isDummy())
        return false;
    
    return true;
}

const KinTranslation& Manipulator::com() const
{
    _com.setZero();
    _mass = 0;
    double bmass;
    
    for(size_t i=0; i<_items.size(); ++i)
    {
        const Body* item_ = _items[i];
        bmass = item_->mass;
        _com += bmass*item_->com.withRespectTo(refFrame());
        _mass += bmass;
    }
    
    for(size_t i=0; i<_robots.size(); ++i)
    {
        const Robot* robot_ = _robots[i];
        bmass = robot_->mass();
        _com += bmass*robot_->com().withRespectTo(refFrame());
        _mass += bmass;
    }
    
    if(_mass>0)
        _com.respectToRef() = _com.respectToRef()/_mass;
    
    return _com;
}

const double& Manipulator::mass() const
{
    _mass = 0;
    
    for(size_t i=0; i<_items.size(); ++i)
    {
        const Body* item_ = _items[i];
        _mass += item_->mass;
    }
    
    for(size_t i=0; i<_robots.size(); ++i)
    {
        const Robot* robot_ = _robots[i];
        _mass += robot_->mass();
    }
    
    return _mass;
}

void Manipulator::_findParentLink()
{
    Frame* checkFrame = &refFrame();
    
    do
    {
        if(checkFrame->isLink())
        {
            Link* checkLink = static_cast<Link*>(checkFrame);
            if( verb.Assert(_myRobot->owns(*checkLink), verbosity::ASSERT_CASUAL,
                            "The manipulator named '"+name()+"' is attached to frame '"
                            +refFrame().name()+"' which does not belong to robot "
                            "'"+_myRobot->name()+"'!", " The closest link to '"
                            +refFrame().name()+"' is '"+checkLink->name()+"' which "
                            "belongs to robot '"+checkLink->robot().name()+"'."))
            {
                _myLink = checkLink;
                return;
            }
            else
            {
                _myLink = _myRobot->_dummyLink;
                return;
            }
        }
        
        checkFrame = &checkFrame->refFrame();
    } while(!checkFrame->isWorld());
    
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "The manipulator named '"+name()+"' is attached to frame '"
                +refFrame().name()+"' which does not belong to any robot!",
                " A manipulator can only function inside of a robot.");
    
    _myLink = _myRobot->_dummyLink;
}
