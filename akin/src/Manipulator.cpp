
#include "akin/Robot.h"
#include "akin/RobotConstraint.h"
#include "akin/AnalyticalIKBase.h"
#include "akin/Solver.h"

using namespace akin;
using namespace std;

std::string Manipulator::mode_to_string(Mode m)
{
    switch(m)
    {
        case FREE:          return "FREE";
        case LINKAGE:       return "LINKAGE";
        case FULLBODY:      return "FULLBODY";
        case ANALYTICAL:    return "ANALYTICAL";
        case SUPPORT:       return "SUPPORT";
        case CUSTOM:        return "CUSTOM";
        default:            return "INVALID";
    }

    return "IMPOSSIBLE";
}

Manipulator::Manipulator(Robot *robot, Frame &referenceFrame, const string &manipName) :
    Frame(referenceFrame, manipName),
    mode(FREE),
    _point(*this, manipName+"_point"),
    _com(*this, manipName+"_com"),
    _myRobot(robot)
{
    for(size_t i=0; i<NUM_MODES; ++i)
    {
        _solvers[i] = new RobotSolverX(parentRobot());
    }

    _findParentLink();
    for(size_t i=0; i<NUM_MODES; ++i)
        _ownConstraint[i] = false;
    
    if(!parentLink().isDummy())
    {
        resetConstraints();
    }
    else
    {
        for(size_t i=0; i<NUM_MODES; ++i)
        {
            _constraints[i] = new NullManipConstraint(parentRobot());
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
        delete _solvers[i];
    }
}

bool Manipulator::ik(Eigen::VectorXd &config, const Transform &targetTf, Frame &relativeFrame)
{
    return ik(mode, config, targetTf, relativeFrame);
}

bool Manipulator::ik(Mode m, Eigen::VectorXd& config, const akin::Transform& targetTf,
                         akin::Frame& relativeFrame)
{
    _myRobot->verb.Assert( 0 <= m && m < NUM_MODES, verbosity::ASSERT_CASUAL,
                           "Selected invalid IK mode ("+std::to_string((int)m)+")!");

    _constraints[m]->target.changeRefFrame(relativeFrame);
    _constraints[m]->target = targetTf;

    if(ANALYTICAL == m)
    {
        bool result = _analytical->getBestSolutionX(config, config).valid;
        _myRobot->setConfig(_analytical->getDofs(), config);
        return result;
    }

    return _solvers[m]->solve(config);
}

bool Manipulator::ik(Eigen::VectorXd& config, KinTransform& targetTf)
{
    return ik(mode, config, targetTf);
}

bool Manipulator::ik(Mode m, Eigen::VectorXd &config, KinTransform &targetTf)
{
    return ik(m, config, targetTf.respectToRef(), targetTf.refFrame());
}

akin::ManipConstraintBase* Manipulator::constraint() { return constraint(mode); }

akin::ManipConstraintBase* Manipulator::constraint(Mode m)
{
    if( m < 0 || NUM_MODES <= m)
        return _constraints[0];
    return _constraints[m];
}

RobotSolverX& Manipulator::solver(Mode m)
{
    if( m < 0 || NUM_MODES <= m)
        return *_solvers[0];
    return *_solvers[m];
}

AnalyticalIKBase& Manipulator::analyticalIK()
{
    return *_analytical;
}

void Manipulator::setConstraint(Mode m, ManipConstraintBase *newConstraint, bool giveOwnership)
{
    if( !_myRobot->verb.Assert(0 <= m || m < NUM_MODES, verbosity::ASSERT_CRITICAL,
                               "Trying to set constraint for invalid mode ("+to_string((int)m)
                               +", Max:"+to_string((int)NUM_MODES)+")"))
    {
        if(giveOwnership)
            delete newConstraint;
        return;
    }
    
    if(ANALYTICAL==m)
    {
        AnalyticalIKBase* check = dynamic_cast<AnalyticalIKBase*>(newConstraint);
        if( !_myRobot->verb.Assert(check != nullptr, verbosity::ASSERT_CRITICAL,
                                   "Trying to set the manipulator's Analytical mode to a constraint"
                                   " class which does not derive from AnalyticalIKBase!"))
        {
            if(giveOwnership)
                delete newConstraint;
            return;
        }
        
        _analytical = check;
    }
    
    if(_ownConstraint[m])
        delete _constraints[m];
    _constraints[m] = newConstraint;
    _ownConstraint[m] = giveOwnership;
    _solvers[m]->setMandatoryConstraint(_constraints[m]);
}

void Manipulator::resetConstraint(Mode m)
{
    if( !_myRobot->verb.Assert(0 <= m || m < NUM_MODES, verbosity::ASSERT_CRITICAL,
                               "Trying to reset constraint for invalid mode ("+to_string((int)m)
                               +", Max:"+to_string((int)NUM_MODES)+")"))
        return;
    
    ManipConstraintBase* constraint;
    if(LINKAGE==m || SUPPORT==m)
    {
        std::vector<size_t> dofs;
        const Joint* current = &parentLink().parentJoint();
        do {
            
            for(size_t d=0; d<current->numDofs(); ++d)
                dofs.push_back(current->dof(d).id());
            current = &current->parentJoint();
            
        } while(current->numChildJoints()==1 && !current->childLink().isAnchor());
        
        constraint = new ManipConstraintX(dofs.size(), *this, dofs);
    }
    else if(FULLBODY==m)
    {
        std::vector<size_t> dofs = Robot::Explorer::getDofIds(_myRobot->joint(BASE_INDEX),
                                                              parentLink().parentJoint());
        constraint = new ManipConstraintX(dofs.size(), *this, dofs);
    }
    else if(ANALYTICAL==m)
    {
        constraint = new NullAnalyticalIK;
    }
    else if(CUSTOM==m)
    {
        constraint = new NullManipConstraint(parentRobot());
    }
    else
    {
        constraint = new NullManipConstraint(parentRobot());
    }
    
    setConstraint(m, constraint, true);
}

void Manipulator::resetConstraints()
{
    for(size_t i=0; i<NUM_MODES; ++i)
        resetConstraint((Mode)i);
}

const KinTranslation& Manipulator::point() const { return _point; }

int Manipulator::attachItem(Body* newItem)
{
    for(size_t i=0; i<_items.size(); ++i)
    {
        if(newItem == _items[i])
            return i;
    }
    
    if(!newItem->changeRefFrame(*this))
        return -1;
    
    newItem->_attachment = &parentLink();
    _items.push_back(newItem);
    return _items.size()-1;
}

Body* Manipulator::item(size_t itemNum)
{
    if( !verb.Assert(itemNum<_items.size(), verbosity::ASSERT_CASUAL,
                     "Trying to access item #"+to_string(itemNum)+" held by manip '"
                     +name()+"', but it only has "+to_string(_items.size())+" entries!"))
        return nullptr;
    
    return _items[itemNum];
}

const Body* Manipulator::item(size_t itemNum) const
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
    if(oldItem==nullptr)
        return false;
    
    return deleteItem(oldItem);
}


int Manipulator::attachRobot(Robot* newRobot)
{
    for(size_t i=0; i<_robots.size(); ++i)
    {
        if(newRobot == _robots[i])
            return i;
    }
    
    if(refFrame().descendsFrom(newRobot->refFrame()))
        return -1;
    
    if(!newRobot->changeRefFrame(*this))
        return -2;
    
    newRobot->_attachment = &parentLink();
    _robots.push_back(newRobot);
    return _robots.size()-1;
}

Robot* Manipulator::robot(size_t robotNum)
{
    if( !verb.Assert(robotNum < _robots.size(), verbosity::ASSERT_CASUAL,
                     "Trying to access robot #"+to_string(robotNum)+" held by manip '"
                     +name()+"', but it only has "+to_string(_robots.size())+"entries!"))
        return nullptr;
    
    return _robots[robotNum];
}

const Robot* Manipulator::robot(size_t robotNum) const
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
    
    _robots[robotNum]->_attachment = nullptr;
    _robots[robotNum]->changeRefFrame(Frame::World());
    _robots.erase(_robots.begin()+robotNum);
    return true;
}

Robot& Manipulator::parentRobot()
{
    return *_myRobot;
}

const Robot& Manipulator::parentRobot() const
{
    return const_cast<Manipulator*>(this)->parentRobot();
}

Link& Manipulator::parentLink() { return *_myLink; }
const Link& Manipulator::parentLink() const { return *_myLink; }

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
        _com = _com.respectToRef()/_mass;
    
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

std::ostream& operator<<(std::ostream& stream, const akin::Manipulator::Mode& m)
{
    stream << akin::Manipulator::mode_to_string(m);
    return stream;
}
