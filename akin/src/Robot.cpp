
#include "../Robot.h"
#include "../RobotConstraint.h"
#include "../Solver.h"

using namespace akin;
using namespace std;

Robot::Robot(akin::Frame& referenceFrame, verbosity::verbosity_level_t report_level) :
    forward_method(STANDARD_NEWTON_EULER),
    inverse_method(FEATHERSTONE),
    zeroValue(1e-8),
    _com(referenceFrame, "CenterOfMass"),
    _balance(NULL),
    _ownsBalance(false),
    _task(NULL),
    _ownsTask(false),
    _solver(new RobotSolverX(*this))
{
    _initializeRobot(referenceFrame, report_level);
}



bool Robot::_needSupportUpdate() const 
{
    for(size_t i=0; i<_supportMemory.size(); ++i)
    {
        bool usedToBeSupport = _supportMemory[i].support;
        if( ( (!usedToBeSupport) && (manip(i).mode == Manipulator::SUPPORT) ) ||
            (   usedToBeSupport  && (manip(i).mode != Manipulator::SUPPORT) ) )
            return true;
        if( manip(i).mode == Manipulator::SUPPORT 
                && _supportTrackers[i]->needsUpdate() )
        {
            if(manip(i).respectToWorld().diff(_supportMemory[i].tf).norm() > zeroValue)
                return true;
        }
    }
    
    for(size_t i=_supportMemory.size(); i<numManips(); ++i)
    {
        if( manip(i).mode == Manipulator::SUPPORT )
            return true;
    }
    
    return false;
}

const std::vector<Eigen::Vector2d>& Robot::getSupportPolygon()
{
    if(_needSupportUpdate())
    {
        for(size_t i=0; i<_supportMemory.size(); ++i)
        {
            _supportMemory[i].support = (manip(i).mode == Manipulator::SUPPORT);
            _supportMemory[i].tf = manip(i).respectToWorld();
            _supportTrackers[i]->clearNotification();
        }
        
        for(size_t i=_supportMemory.size(); i<numManips(); ++i)
        {
            _supportMemory.push_back(ManipMemory(manip(i).mode==Manipulator::SUPPORT,
                                                 manip(i).respectToWorld()));
            _supportTrackers[i]->clearNotification();
        }
        
        verb.debug() << "Updating support polygon for robot '"+name()+"'";
        verb.end();
        
        _supportPolgyon = computeSupportPolgon();
        _supportCenter = computeCentroid(_supportPolgyon);
    }
    
    return _supportPolgyon;
}

const Eigen::Vector2d& Robot::getSupportCenter()
{
    if(_needSupportUpdate())
        getSupportPolygon();
    
    return _supportCenter;
}

std::vector<Eigen::Vector2d> Robot::computeSupportPolgon() const
{
    _supportPoints.clear();

    for(size_t i=0; i<numManips(); ++i)
    {
        if(Manipulator::SUPPORT == manip(i).mode)
        {
            const std::vector<KinTranslation>& points = manip(i).supportGeometry;
            for(size_t j=0; j<points.size(); ++j)
            {
                const KinTranslation& p = points[j];
                _supportPoints.push_back(p.respectToWorld().block<2,1>(0,0));
            }
        }
    }

    return computeConvexHull(_supportPoints);
}

void Robot::forceSupportUpdate()
{
    _supportPolgyon = computeSupportPolgon();
    _supportCenter = computeCentroid(_supportPolgyon);
}

void Robot::_initializeRobot(akin::Frame& referenceFrame, verbosity::verbosity_level_t report_level)
{
    verb.level = report_level;
    _enforceJointLimits = true;
    _name = "default_robot";

    _dummyLink = new Link(this, Frame::World(), "dummy", 0, false);
    _dummyLink->_isDummy = true;

    _dummyJoint = new Joint(this, -1, "dummy", _dummyLink, _dummyLink);
    _dummyJoint->_type = Joint::FIXED;
    _dummyJoint->_isDummy = true;
    _dummyJoint->_changeParentLink(_dummyLink);
    _dummyJoint->_childLink = _dummyLink;
    
    _dummyManip = new Manipulator(this, *_dummyLink, "dummy");
    _dummyManip->_isDummy = true;
    
    // Create the root degrees of freedom
    Link* pos_x_link = new Link(this, referenceFrame, "DOF_POS_X", DOF_POS_X, true);
    pos_x_link->_isDummy = true;
    _root_dummy_links.push_back(pos_x_link);
    _linkNameToIndex[pos_x_link->name()] = pos_x_link->id();
    pos_x_link->_setParentJoint(_dummyJoint);
    
    Link* pos_y_link = new Link(this, *pos_x_link, "DOF_POS_Y", DOF_POS_Y, false);
    pos_y_link->_isDummy = true;
    _root_dummy_links.push_back(pos_y_link);
    _linkNameToIndex[pos_y_link->name()] = pos_y_link->id();
    
    Joint* pos_x_joint = new Joint(this, DOF_POS_X, "DOF_POS_X", pos_x_link, pos_y_link, 
                                   Transform::Identity(), Axis(1,0,0), Joint::PRISMATIC);
    pos_x_joint->_isDummy = true;
    _root_dummy_joints.push_back(pos_x_joint);
    _jointNameToIndex[pos_x_joint->name()] = DOF_POS_X;
    
    pos_x_link->_addChildJoint(pos_x_joint);
    pos_y_link->_setParentJoint(pos_x_joint);
    
    //////////
    
    Link* pos_z_link = new Link(this, *pos_y_link, "DOF_POS_Z", DOF_POS_Z, false);
    pos_z_link->_isDummy = true;
    _root_dummy_links.push_back(pos_z_link);
    _linkNameToIndex[pos_z_link->name()] = pos_z_link->id();
    
    Joint* pos_y_joint = new Joint(this, DOF_POS_Y, "DOF_POS_Y", pos_y_link, pos_z_link,
                                   Transform::Identity(), Axis(0,1,0), Joint::PRISMATIC);
    pos_y_joint->_isDummy = true;
    _root_dummy_joints.push_back(pos_y_joint);
    _jointNameToIndex[pos_y_joint->name()] = DOF_POS_Y;
    
    pos_y_link->_addChildJoint(pos_y_joint);
    pos_z_link->_setParentJoint(pos_y_joint);
    
    //////////
    
    Link* rot_x_link = new Link(this, *pos_z_link, "DOF_ROT_X", DOF_ROT_X, false);
    rot_x_link->_isDummy = true;
    _root_dummy_links.push_back(rot_x_link);
    _linkNameToIndex[rot_x_link->name()] = rot_x_link->id();
    
    Joint* pos_z_joint = new Joint(this, DOF_POS_Z, "DOF_POS_Z", pos_z_link, rot_x_link,
                                   Transform::Identity(), Axis(0,0,1), Joint::PRISMATIC);
    pos_z_joint->_isDummy = true;
    _root_dummy_joints.push_back(pos_z_joint);
    _jointNameToIndex[pos_z_joint->name()] = DOF_POS_Z;
    
    pos_z_link->_addChildJoint(pos_z_joint);
    rot_x_link->_setParentJoint(pos_z_joint);
    
    //////////
    
    Link* rot_y_link = new Link(this, *rot_x_link, "DOF_ROT_Y", DOF_ROT_X, false);
    rot_y_link->_isDummy = true;
    _root_dummy_links.push_back(rot_y_link);
    _linkNameToIndex[rot_y_link->name()] = rot_y_link->id();
    
    Joint* rot_x_joint = new Joint(this, DOF_ROT_X, "DOF_ROT_X", rot_x_link, rot_y_link,
                                   Transform::Identity(), Axis(1,0,0), Joint::REVOLUTE);
    rot_x_joint->_isDummy = true;
    _root_dummy_joints.push_back(rot_x_joint);
    _jointNameToIndex[rot_x_joint->name()] = DOF_ROT_X;
    
    rot_x_link->_addChildJoint(rot_x_joint);
    rot_y_link->_setParentJoint(rot_x_joint);
    
    //////////
    
    Link* rot_z_link = new Link(this, *rot_y_link, "DOF_ROT_Z", DOF_ROT_Z, false);
    rot_z_link->_isDummy = true;
    _root_dummy_links.push_back(rot_z_link);
    _linkNameToIndex[rot_z_link->name()] = rot_z_link->id();
    
    Joint* rot_y_joint = new Joint(this, DOF_ROT_Y, "DOF_ROT_Y", rot_y_link, rot_z_link,
                                   Transform::Identity(), Axis(0,1,0), Joint::REVOLUTE);
    rot_y_joint->_isDummy = true;
    _root_dummy_joints.push_back(rot_y_joint);
    _jointNameToIndex[rot_y_joint->name()] = DOF_ROT_Y;
    
    rot_y_link->_addChildJoint(rot_y_joint);
    rot_z_link->_setParentJoint(rot_y_joint);
    
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
    
    for(size_t m=0; m<_manips.size(); ++m)
    {
        delete _manips[m];
    }
    
    for(size_t m=0; m<_supportTrackers.size(); ++m)
    {
        delete _supportTrackers[m];
    }

    if(_ownsBalance)
        delete _balance;

    if(_ownsTask)
        delete _task;

    delete _solver;
}

Eigen::VectorXd Robot::getConfig(const std::vector<size_t>& dofs) const
{
    Eigen::VectorXd config(dofs.size());
    for(size_t i=0; i<dofs.size(); ++i)
        config[i] = dof(dofs[i]).value();
    return config;
}

bool Robot::setConfig(const std::vector<size_t>& dofs, const Eigen::VectorXd &values)
{
    if(!verb.Assert((int)dofs.size()==values.size(), verbosity::ASSERT_CASUAL,
                    "Mismatch in array sizes ("+std::to_string(dofs.size())+":"
                    +std::to_string(values.size())+") in call to Robot::setConfig(~,~)"))
        return false;

    bool result = true;
    for(size_t i=0; i<dofs.size(); ++i)
    {
        if(!dof(dofs[i]).value(values[i]))
            result = false;
    }

    return result;
}

Frame& Robot::refFrame()
{
    return _root_dummy_links.front()->refFrame();
}

const Frame& Robot::refFrame() const
{
    return const_cast<Robot*>(this)->refFrame();
}

bool Robot::changeRefFrame(Frame &newRefFrame)
{
    return _root_dummy_links.front()->changeRefFrame(newRefFrame);
}

Frame& Robot::frame()
{
    // TODO: Should this return the anchor link frame or always the root link frame?
//    return anchorLink();
    return *_root;
}

const Frame& Robot::frame() const
{
    return const_cast<Robot*>(this)->frame();
}


BalanceConstraintBase* Robot::balance() { return _balance; }
const BalanceConstraintBase* Robot::balance() const { return _balance; }

void Robot::setBalanceConstraint(BalanceConstraintBase *newConstraint, bool ownConstraint)
{
    if(_ownsBalance)
        delete _balance;

    _balance = newConstraint;
    _ownsBalance = ownConstraint;
}

void Robot::setDefaultBalanceConstraint()
{
    setBalanceConstraint(new CenterOfMassConstraintX(numJoints()+6, *this,
                                    Explorer::getJointIds(joint(DOF_POS_X))));
}

void Robot::name(string newName)
{
    _name = newName;
}

RobotConstraintBase* Robot::task() { return _task; }
const RobotConstraintBase* Robot::task() const { return _task; }

void Robot::setTaskConstraint(RobotConstraintBase *newConstraint, bool ownConstraint)
{
    if(_ownsTask)
        delete _task;

    _task = newConstraint;
    _ownsTask = ownConstraint;
    _solver->setMandatoryConstraint(_task);
}

void Robot::setDefaultTaskConstraint()
{
    setTaskConstraint(new RobotTaskConstraintX(numJoints()+6, *this,
                                    Explorer::getJointIds(joint(DOF_POS_X))));
}

void Robot::setDefaultRobotConstraints()
{
    setDefaultBalanceConstraint();
    setDefaultTaskConstraint();
}

RobotSolverX& Robot::solver() { return *_solver; }

bool Robot::solve()
{
    if(_task == NULL)
        return true;

    _taskConfig = getConfig(_task->getJoints());
    bool result = _solver->solve(_taskConfig);
//    std::cout << "\nGradient:" << _taskConfig.transpose() << "\n\n" << std::endl;



    setConfig(_task->getJoints(), _taskConfig);

    return result;
}

const KinTranslation& Robot::com() const
{
    _com = com(anchorLink(), _com.refFrame());
    return _com;
}

Translation Robot::com(const Link& startLink, const Frame& referenceFrame,
                       Explorer::policy p) const
{
    Translation fcom;
    double fmass = 0, lmass = 0;

    _crawler.reset(startLink, p);
    const Link* current = _crawler.nextLink();
    while(current != NULL)
    {
        lmass = current->mass;
        fcom += lmass*current->com.withRespectTo(referenceFrame);
        fmass += lmass;

        for(size_t i=0; i<current->numManips(); ++i)
        {
            const Manipulator& manip_ = current->manip(i);
            lmass = manip_.mass();
            fcom += lmass*manip_.com().withRespectTo(referenceFrame);
            fmass += lmass;
        }

        current = _crawler.nextLink();
    }

//    if( verb.Assert(fmass > 0, verbosity::ASSERT_CASUAL,
//                    "Center of Mass requested for Robot '"+name()+"' starting at Link '"
//                    +startLink.name()+"', but there was no mass at all!") )
    if( fmass > 0 )
        fcom = fcom/fmass;
    else
        fcom.setZero();

    return fcom;
}

double Robot::mass(const Link &startLink, Explorer::policy p) const
{
    double mass_ = 0;

    _crawler.reset(startLink, p);
    const Link* current = _crawler.nextLink();
    while(current != NULL)
    {
        mass_ += current->mass;

        for(size_t i=0; i<current->numManips(); ++i)
        {
            const Manipulator& manip_ = current->manip(i);
            mass_ += manip_.mass();
        }

        current = _crawler.nextLink();
    }

    return mass_;
}

const double& Robot::mass() const
{
    _mass = mass(anchorLink());
    return _mass;
}

Translation Robot::getCom(const Frame &withRespectToFrame) const
{
    return com().withRespectTo(withRespectToFrame);
}

double Robot::getMass() const
{
    return mass();
}

Eigen::Matrix3d Robot::getInertiaTensor(const Frame& ) const
{
    // TODO FIXME
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "getInertiaTensor() is not implemented for Robot yet!");

    return Eigen::Matrix3d::Identity();
}

FreeVector Robot::getForces(const Frame& ) const
{
    // TODO FIXME
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "getForces() is not implemented for Robot yet!");

    return FreeVector::Zero();
}

FreeVector Robot::getMoments(const Frame &) const
{
    // TODO FIXME
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "getMoments() is not implemented for Robot yet!");

    return FreeVector::Zero();
}

Screw Robot::getWrench(const Frame &withRespectToFrame) const
{
    return Screw(getForces(withRespectToFrame),getMoments(withRespectToFrame));
}

const string& Robot::name() const { return _name; }

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
    rot_z_joint->_isDummy = true;
    _root_dummy_joints.push_back(rot_z_joint);
    _jointNameToIndex[rot_z_joint->name()] = DOF_ROT_Z;
    
//    _com.changeRefFrame(*rootLink);

    _root_dummy_links.back()->_addChildJoint(rot_z_joint);
    rootLink->_setParentJoint(rot_z_joint);
    
    return true;
}


//int Robot::createJointLinkPair(Link &parentLink,
//                               const string &newLinkName, const string &newJointName,
//                               const Transform &baseTransform, const Axis &jointAxis,
//                               Joint::Type jointType, double minJointValue, double maxJointValue)
int Robot::createJointLinkPair(Link& parentLink, const string& newLinkName,
                               const ProtectedJointProperties& properties)
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
    
    if(!verb.Assert(!checkForJointName(properties._name), verbosity::ASSERT_CRITICAL,
                    "Error: You wanted to create a new joint named '"+properties._name+
                    "' but a joint by that name already exists in the robot named '"
                    +name()+"'!"))
        return -4;

    Link* newLink = new Link(this, parentLink, newLinkName, _links.size(), false);
    _insertLink(newLink);

    Joint* newJoint = new Joint(this, _joints.size(), properties._name,
                                &parentLink, newLink,
                                properties._baseTransform, properties._axis,
                                properties._type, properties._minValue, properties._maxValue,
                                properties._maxSpeed, properties._maxAcceleration,
                                properties._maxTorque);
    _insertJoint(newJoint);
    
    parentLink._addChildJoint(newJoint);
    newLink->_setParentJoint(newJoint);
    
    return _links.size()-1;
}

//int Robot::createJointLinkPair(size_t parentLinkID,
//                               const string &newLinkName, const string &newJointName,
//                               const Transform &baseTransform, const Axis &jointAxis,
//                               Joint::Type jointType, double minJointValue, double maxJointValue,
//                               double maxSpeed, double maxAcceleration, double maxTorque)
int Robot::createJointLinkPair(size_t parentLinkID, const string& newLinkName,
                               const ProtectedJointProperties& properties)
{
//    return createJointLinkPair(link(parentLinkID),
//                               newLinkName, newJointName,
//                               baseTransform, jointAxis,
//                               jointType, minJointValue, maxJointValue);
    return createJointLinkPair(link(parentLinkID), newLinkName, properties);
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

void Robot::_computeABA_pass2() const
{
    anchorLink()._computeABA_pass2();
}

void Robot::_computeABA_pass3() const
{
    anchorLink()._computeABA_pass3();
}

const Matrix6d& Robot::_ABA_Ia() const { return anchorLink()._ABA_Ia(); }
const Vector6d& Robot::_ABA_pa() const { return anchorLink()._ABA_pa(); }
const Vector6d& Robot::_ABA_c() const { return anchorLink()._ABA_c(); }
const Vector6d& Robot::_ABA_a() const { return anchorLink()._ABA_a(); }
const Vector6d& Robot::_ABA_arel() const { return anchorLink()._ABA_arel(); }

const Matrix6Xd& Robot::_ABA_h() const { return anchorLink()._ABA_h(); }
const Eigen::VectorXd& Robot::_ABA_u() const { return anchorLink()._ABA_u(); }
const Eigen::MatrixXd& Robot::_ABA_d() const { return anchorLink()._ABA_d(); }
const Eigen::VectorXd& Robot::_ABA_qdd() const { return anchorLink()._ABA_qdd(); }

Joint& Robot::joint(size_t jointNum)
{
    if( !verb.Assert( jointNum < _joints.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a joint index ("
                      +to_string(jointNum)+") which is out of bounds "
                      "on the robot named '"+name()+"'") )
        return *_dummyJoint;
    
    return *_joints[jointNum];
}

Joint& Robot::joint(const string &jointName)
{
    return joint(getJointIndex(jointName));
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

const Joint& Robot::joint(size_t jointNum) const
{
    return const_cast<Robot*>(this)->joint(jointNum);
}

const Joint& Robot::joint(const string &jointName) const
{
    return joint(getJointIndex(jointName));
}

size_t Robot::numJoints() const { return _joints.size(); }

DegreeOfFreedom& Robot::dof(size_t dofNum)
{
    if( !(dofNum < _dofs.size()) )
    {
        verb.Assert( false, verbosity::ASSERT_CASUAL,
                      "You have requested a DOF index ("
                      +to_string(dofNum)+") which is out of bounds "
                      "on the robot named '"+name()+"'");
        return *_dummyDof;
    }

    return *_dofs[dofNum];
}

DegreeOfFreedom& Robot::dof(const string &dofName)
{
    return dof(getDofIndex(dofName));
}

size_t Robot::getDofIndex(const string &dofName) const
{
    StringMap::const_iterator d = _dofNameToIndex.find(dofName);
    if( !verb.Assert( d != _dofNameToIndex.end(),
                      verbosity::ASSERT_CASUAL,
                      "There is no DOF named '"+dofName+"' in the robot named '"
                      +name()+"'!") )
        return DOF_INVALID;

    return d->second;
}

const DegreeOfFreedom& Robot::dof(size_t dofNum) const
{
    return const_cast<Robot*>(this)->dof(dofNum);
}

const DegreeOfFreedom& Robot::dof(const string& dofName) const
{
    return dof(getDofIndex(dofName));
}

size_t Robot::numDofs() const { return _dofs.size(); }

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

const Link& Robot::link(const string &linkName) const
{
    return link(getLinkIndex(linkName));
}

size_t Robot::numLinks() const { return _links.size(); }

Link& Robot::anchorLink() { return *_anchor; }
const Link& Robot::anchorLink() const { return *_anchor; }

void Robot::anchorLink(Link &)
{
    // TODO
}

void Robot::anchorLink(size_t )
{
    // TODO
}

Link& Robot::rootLink() { return *_root; }
const Link& Robot::rootLink() const { return *_root; }

Link& Robot::link(size_t linkNum)
{
    if( !verb.Assert( linkNum < _links.size(),
                      verbosity::ASSERT_CASUAL,
                      "You have requested a link index ("
                      +to_string(linkNum)+") which is out of bounds ("
                      +to_string(_links.size())+") "
                      "on the robot named '"+name()+"'") )
        return *_dummyLink;
    
    return *_links[linkNum];
}

const Link& Robot::link(size_t linkNum) const
{
    return const_cast<Robot*>(this)->link(linkNum);
}

int Robot::addManipulator(Frame &attachment, const string &name, Transform relativeTf)
{
    if(checkForManipName(name))
        return -1;
    
    Manipulator* newManip = new Manipulator(this, attachment, name);
    newManip->respectToRef(relativeTf);
    
    if(newManip->parentLink().isDummy())
        return -2;
    
    _insertManip(newManip);
    
    newManip->parentLink()._manips.push_back(newManip);
    return _manips.size()-1;
}

bool Robot::removeManipulator(Manipulator &m)
{
    for(size_t i=0; i<_manips.size(); ++i)
    {
        if( &m == _manips[i] )
        {
            delete _manips[i];
            _manips[i] = _dummyManip;
            return true;
        }
    }
    
    return false;
}

Manipulator& Robot::manip(const string &manipName)
{
    return manip(getManipIndex(manipName));
}

size_t Robot::getManipIndex(const string &manipName) const
{
    StringMap::const_iterator i = _manipNameToIndex.find(manipName);
    if( !verb.Assert( i != _manipNameToIndex.end(), verbosity::ASSERT_CASUAL,
                      "There is no manipulator named '"+manipName+"' in the robot named '"
                      +name()+"'!") )
        return DOF_INVALID;
    
    return i->second;
}

const Manipulator& Robot::manip(const string &manipName) const
{
    return const_cast<Robot*>(this)->manip(manipName);
}

Manipulator& Robot::manip(size_t manipNum)
{
    if( !verb.Assert( manipNum < _manips.size(), verbosity::ASSERT_CASUAL,
                      "You have requested a manipulator index ("
                      +to_string(manipNum)+") which is out of bounds ("
                      +to_string(_manips.size())+") on the robot named '"
                      +name()+"'") )
        return *_dummyManip;
    
    return *_manips[manipNum];
}

const Manipulator& Robot::manip(size_t manipNum) const
{
    return const_cast<Robot*>(this)->manip(manipNum);
}

size_t Robot::numManips() const { return _manips.size(); }

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
    _supportTrackers.push_back(new Tracker(*newManip, newManip->name()+"_support_tracker"));
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

bool Robot::checkForDofName(const string &name) const
{
    StringMap::const_iterator n = _dofNameToIndex.find(name);
    if( n == _dofNameToIndex.end() )
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

void Robot::setDynamicsMode(dynamics_mode_t mode)
{
    anchorLink().setDynamicsMode(mode);
}

dynamics_mode_t Robot::getDynamicsMode() const
{
    return anchorLink().getDynamicsMode();
}

bool Robot::notifyDynUpdate()
{
    return anchorLink().notifyDynUpdate();
}

bool Robot::needsDynUpdate() const
{
    return anchorLink().needsDynUpdate();
}

std::vector<const Link*> Robot::Explorer::getPath(const Link &startLink, const Link &endLink)
{
    Explorer crawl(startLink, endLink);
    
    return crawl._path;
}

std::vector<size_t> Robot::Explorer::getIdPath(const Link &startLink, const Link &endLink)
{
    Explorer crawl(startLink, endLink);
    
    std::vector<size_t> result; result.reserve(crawl._path.size());
    const Link* link = crawl.nextLink();
    while(link != NULL)
    {
        result.push_back(link->id());
        link = crawl.nextLink();
    }
    
    return result;
}

std::vector<const Joint*> Robot::Explorer::getPath(const Joint &startJoint, const Joint &endJoint)
{
    Explorer crawl(startJoint, endJoint);
    
    std::vector<const Joint*> result; result.reserve(crawl._path.size());
    const Joint* joint = crawl.nextJoint();
    while(joint != NULL)
    {
        result.push_back(joint);
        joint = crawl.nextJoint();
    }
    
    return result;
}

std::vector<size_t> Robot::Explorer::getIdPath(const Joint &startJoint, const Joint &endJoint)
{
    Explorer crawl(startJoint, endJoint);
    
    std::vector<size_t> result; result.reserve(crawl._path.size());
    const Joint* joint = crawl.nextJoint();
    while(joint != NULL)
    {
        result.push_back(joint->id());
        joint = crawl.nextJoint();
    }
    
    return result;
}

std::vector<const Link*> Robot::Explorer::getLinks(const Link &startLink, policy p)
{
    Explorer crawl(startLink, p);
    
    std::vector<const Link*> result;
    const Link* nextLink;
    while( (nextLink = crawl.nextLink()) )
        result.push_back(nextLink);
    
    return result;
}

std::vector<const Joint*> Robot::Explorer::getJoints(const Joint &startJoint, policy p)
{
    Explorer crawl(startJoint, p);
    
    std::vector<const Joint*> result;
    const Joint* nextJoint;
    while( (nextJoint = crawl.nextJoint()) )
        result.push_back(nextJoint);
    
    return result;
}

std::vector<size_t> Robot::Explorer::getLinkIds(const Link &startLink, policy p)
{
    Explorer crawl(startLink, p);
    
    std::vector<size_t> result;
    const Link* nextLink;
    while( (nextLink = crawl.nextLink()) )
        result.push_back(nextLink->id());
    
    return result;
}

std::vector<size_t> Robot::Explorer::getJointIds(const Joint &startJoint, policy p)
{
    Explorer crawl(startJoint, p);
    
    std::vector<size_t> result;
    const Joint* nextJoint;
    while( (nextJoint = crawl.nextJoint()) )
        result.push_back(nextJoint->id());
    
    return result;
}

Robot::Explorer::Explorer()
{
    _init();
    _p = INACTIVE;
}

Robot::Explorer::Explorer(const akin::Link& startLink, const akin::Link& endLink)
{
    _init();
    reset(startLink, endLink);
}

Robot::Explorer::Explorer(const akin::Link& startLink, policy p)
{
    _init();
    reset(startLink, p);
}

Robot::Explorer::Explorer(const Joint &startJoint, const Joint &endJoint)
{
    _init();
    reset(startJoint, endJoint);
}

Robot::Explorer::Explorer(const Joint &startJoint, policy p)
{
    _init();
    reset(startJoint, p);
}

void Robot::Explorer::_init()
{
    _recorder.reserve(100);
    _path.reserve(100);
    _temp.reserve(100);
    stopAtRoot = false;
}

void Robot::Explorer::reset(const akin::Link& startLink, const akin::Link& endLink)
{
    _p = PATH;
    _pathLocation = 0;
    
    _path.clear();
    _temp.clear();
    
    if(!startLink.robot().owns(endLink))
        return;
    
    if(startLink.descendsFrom(endLink))
    {
        const Link* current = &startLink;
        while(current != &endLink)
        {
            _path.push_back(current);
            current = &current->parentLink();
        }
        _path.push_back(current);
    }
    else if(endLink.descendsFrom(startLink))
    {
        const Link* current = &endLink;
        while(current != &startLink)
        {
            _temp.push_back(current);
            current = &current->parentLink();
        }
        _temp.push_back(current);
        
        for(size_t i=_temp.size(); i>0; --i)
            _path.push_back(_temp[i-1]);
    }
    else
    {
        const Link* current = &startLink;
        while(!current->isRoot())
        {
            _path.push_back(current);
            current = &current->parentLink();
        }
        _path.push_back(current);
        
        current = &endLink;
        while(!current->isRoot())
        {
            _temp.push_back(current);
            current = &current->parentLink();
        }
        
        for(size_t i=_temp.size(); i>0; --i)
            _path.push_back(_temp[i-1]);
    }
}

void Robot::Explorer::reset(const Link &startLink, policy p)
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

void Robot::Explorer::reset(const Joint &startJoint, const Joint &endJoint)
{
    reset(startJoint.childLink(), endJoint.childLink());
}

void Robot::Explorer::reset(const Joint &startJoint, policy p)
{
    reset(startJoint.childLink(), p);
}

const Link* Robot::Explorer::currentLink() const
{
    if(_recorder.size()==0)
    {
        if(_finished)
            return NULL;
        
        return _first;
    }
    
    return _recorder.back().link;
}

Link* Robot::Explorer::nonconst_currentLink() const
{
    return const_cast<Link*>(currentLink());
}

const Joint* Robot::Explorer::currentJoint() const
{
    const Link* link = currentLink();
    if(link == NULL)
        return NULL;
    
    return &link->parentJoint();
}

Joint* Robot::Explorer::nonconst_currentJoint() const
{
    return const_cast<Joint*>(currentJoint());
}

const Link* Robot::Explorer::nextLink()
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
                std::cout << "Hitting " << _recorder.back().link->name() << std::endl;
                _recorder.push_back(Recorder(&t.link->downstreamLink(t.count), 0));
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
                if(t.link->upstreamLink().isDummy())
                {
                    ++t.count;
                }
                else if(_recorder.size() == 1)
                {
                    _recorder.push_back(Recorder(&t.link->upstreamLink(), -1));
                    ++t.count;
                    return _recorder.back().link;
                }
                else if(&t.link->upstreamLink() != _recorder[_recorder.size()-2].link)
                {
                    _recorder.push_back(Recorder(&t.link->upstreamLink(), -1));
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
                else if(&t.link->downstreamLink(t.count) != _recorder[_recorder.size()-2].link)
                {
                    _recorder.push_back(Recorder(&t.link->downstreamLink(t.count), -1));
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

const Joint* Robot::Explorer::nextJoint()
{
    const Link* link = nextLink();
    if(link == NULL)
        return NULL;
    
    return &link->parentJoint();
}

Joint* Robot::Explorer::nonconst_nextJoint()
{
    return const_cast<Joint*>(nextJoint());
}

Robot::Explorer::Recorder::Recorder() :
    link(NULL),
    count(0)
{
    
}

Robot::Explorer::Recorder::Recorder(const Link *link_, size_t count_) :
    link(link_),
    count(count_)
{
    
}
