
#include "akin/Robot.h"

using namespace akin;
using namespace std;

std::string PublicJointProperties::type_to_string(akin::Joint::Type myJointType)
{
    switch(myJointType)
    {
        case FIXED:             return "FIXED";
        case REVOLUTE:          return "REVOLUTE";
        case PRISMATIC:         return "PRISMATIC";
        case FLOATING:          return "FLOATING";
        case CUSTOM:            return "CUSTOM";
        case JOINT_TYPE_SIZE:   return "INVALID (JOINT_TYPE_SIZE)";
    }
    
    return "UNKNOWN JOINT TYPE ("+to_string((int)myJointType)+")";
}

DegreeOfFreedom& Joint::dof(size_t num)
{
    if( !(num < _dofs.size()) )
    {
        verb.Assert( false, verbosity::ASSERT_CASUAL,
                     "You have requested a DOF index ("
                     +to_string(num)+") in which is out of bounds "
                     "for Joint '"+name()+"' on the robot named '"
                     +_robot->name()+"'");
        return *_robot->_dummyDof;
    }

    return *_dofs[num];
}

const DegreeOfFreedom& Joint::dof(size_t num) const
{
    return const_cast<Joint*>(this)->dof(num);
}

size_t Joint::numDofs() const { return _dofs.size(); }

const Eigen::VectorXd& Joint::values() const
{
    for(size_t i=0; i<numDofs(); ++i)
        _values[i] = dof(i).value();

    return _values;
}

bool Joint::values(const Eigen::VectorXd &newValues)
{
    if(newValues.size() != (int)numDofs())
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL,
                    "Error: Passing a VectorXd with "+to_string(newValues.size())
                    +" components to values(~) function of Joint '"+name()
                    +"' which has "+to_string(numDofs())+" Degrees of Freedom!");
        return false;
    }

    bool inBounds = true;
    for(size_t i=0; i<numDofs(); ++i)
        inBounds &= dof(i).value(newValues[i]);
    return inBounds;
}

const Eigen::VectorXd& Joint::velocities() const
{
    for(size_t i=0; i<numDofs(); ++i)
        _velocities[i] = dof(i).velocity();

    return _velocities;
}

bool Joint::velocities(const Eigen::VectorXd &newVelocities)
{
    if(newVelocities.size() != (int)numDofs())
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL,
                    "Error: Passing a VectorXd with "+to_string(newVelocities.size())
                    +" components to velocities(~) function of Joint '"+name()
                    +"' which has "+to_string(numDofs())+" Degrees of Freedom!");
        return false;
    }

    bool inBounds = true;
    for(size_t i=0; i<numDofs(); ++i)
        inBounds &= dof(i).velocity(newVelocities[i]);
    return inBounds;
}

const Eigen::VectorXd& Joint::accelerations() const
{
    for(size_t i=0; i<numDofs(); ++i)
        _accelerations[i] = dof(i).acceleration();

    return _accelerations;
}

bool Joint::accelerations(const Eigen::VectorXd &newAccelerations)
{
    if(newAccelerations.size() != (int)numDofs())
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL,
                    "Error: Passing a VectorXd with "+to_string(newAccelerations.size())
                    +" components to accelerations(~) function of Joint '"+name()
                    +"' which has "+to_string(numDofs())+" Degrees of Freedom!");
        return false;
    }

    bool inBounds = true;
    for(size_t i=0; i<numDofs(); ++i)
        inBounds &= dof(i).acceleration(newAccelerations[i]);
    return inBounds;
}

const Eigen::VectorXd& Joint::efforts() const
{
    for(size_t i=0; i<numDofs(); ++i)
        _efforts[i] = dof(i).effort();

    return _efforts;
}

bool Joint::efforts(const Eigen::VectorXd &newEfforts)
{
    if(newEfforts.size() != (int)numDofs())
    {
        verb.Assert(false, verbosity::ASSERT_CRITICAL,
                    "Error: Passing a VectorXd with "+to_string(newEfforts.size())
                    +" components to efforts(~) function of Joint '"+name()
                    +"' which has "+to_string(numDofs())+" Degrees of Freedom!");
        return false;
    }

    bool inBounds = true;
    for(size_t i=0; i<numDofs(); ++i)
        inBounds &= dof(i).effort(newEfforts[i]);
    return inBounds;
}

void Joint::_computeRefTransform() const
{
    if(_isDummy)
        return;

    // Handle different joint types
    Transform respectToRef = _baseTransform;
    if(REVOLUTE == _type)
    {
        respectToRef = respectToRef * Transform(Translation(0,0,0),
                                                Rotation(dof(0).value(), _axis));
    }
    else if(PRISMATIC == _type)
    {
        respectToRef = respectToRef * Transform(dof(0).value()*_axis, Rotation());
    }
    else if(FLOATING == _type)
    {
//        respectToRef = respectToRef * Transform(
//                    Translation(dof(0).value(),dof(1).value(),dof(2).value()),
//                    Rotation(FreeVector(dof(3).value(),dof(4).value(),dof(5).value())));
        respectToRef.translate(Vec3(dof(0).value(),dof(1).value(),dof(2).value()));
        respectToRef.rotate(Rotation(dof(3).value(),Vec3(1,0,0)));
        respectToRef.rotate(Rotation(dof(4).value(),Vec3(0,1,0)));
        respectToRef.rotate(Rotation(dof(5).value(),Vec3(0,0,1)));
    }

    // Handle if the kinematic direction is reversed
    if(_reversed)
    {
        _downstreamLink->_respectToRef = respectToRef.inverse();
    }
    else
    {
        _downstreamLink->_respectToRef = respectToRef;
    }

    _needsPosUpdate = false;
}

void Joint::_computeRelVelocity() const
{
    if(_isDummy)
        return;

    Vec3 v, w;
    if(REVOLUTE==_type)
    {
        v.setZero();
        w = _axis*dof(0).velocity();
    }
    else if(PRISMATIC==_type)
    {
        v = _axis*dof(0).velocity();
        w.setZero();
    }
    else if(FLOATING==_type)
    {
        for(size_t i=0; i<3; ++i)
        {
            v[i] = dof(i).velocity();
            w[i] = dof(i+3).velocity();
        }
    }
    else
    {
        v.setZero();
        w.setZero();
    }

    _downstreamLink->_relativeLinearVel = v;
    _downstreamLink->_relativeAngularVel = w;

    _needsVelUpdate = false;
}

void Joint::_computeRelAcceleration() const
{
    if(_isDummy)
        return;

    Vec3 a, alpha;
    if(REVOLUTE==_type)
    {
        a.setZero();
        alpha = _axis*dof(0).acceleration();
    }
    else if(PRISMATIC==_type)
    {
        a = _axis*dof(0).acceleration();
        alpha.setZero();
    }
    else if(FLOATING==_type)
    {
        for(size_t i=0; i<3; ++i)
        {
            a[i] = dof(i).acceleration();
            alpha[i] = dof(i+3).acceleration();
        }
    }

    _downstreamLink->_relativeLinearAcc = a;
    _downstreamLink->_relativeAngularAcc = alpha;

    _needsAccUpdate = false;
}

ProtectedJointProperties::ProtectedJointProperties(const string &jointName,
        PublicJointProperties::Type mType,
        const Transform &mBaseTransform,
        const Vec3 &mJointAxis) :
    _baseTransform(mBaseTransform),
    _axis(mJointAxis),
    _type(mType),
    _id(0),
    _name(jointName),
    _isDummy(false)
{

}

Joint::Joint(Robot* mRobot, size_t jointID, Link* mParentLink, Link* mChildLink,
             const ProtectedJointProperties &joint_properties,
             const DofProperties& dof_properties) :
    ProtectedJointProperties(joint_properties),
    verb(mRobot->verb),
    _parentLink(mParentLink),
    _childLink(mChildLink),
    _reversed(false),
    _upstreamLink(mParentLink),
    _downstreamLink(mChildLink),
    _robot(mRobot)
{
    _id = jointID;

    if( !mChildLink->isDummy() )
        verb.Assert(mParentLink == &mChildLink->refFrame(), verbosity::ASSERT_CRITICAL,
                "Joint '"+name()+"' wants to connect '"+mChildLink->name()
                +"' to '"+mParentLink->name()+"' but '"+mChildLink->name()
                +"' is in the reference frame of '"+mChildLink->refFrame().name()
                +"'!", " A joint must always connect a link to its parent frame!");

    _createDofs(dof_properties);
    axis(_axis);
}

void Joint::_createDofs(const DofProperties& dof_properties)
{
    _dofs.clear();
    if(type()==PRISMATIC || type()==REVOLUTE)
    {
        DegreeOfFreedom* newdof = new DegreeOfFreedom(this, _name, dof_properties);
        newdof->_localID = 0;
        newdof->_id = _robot->_dofs.size();

        _robot->_insertDof(newdof);
        _dofs.push_back(newdof);
    }
    else if(type()==FIXED)
    {
        // Do nothing?
    }
    else if(type()==FLOATING)
    {
        DegreeOfFreedom* newdof;

        for(size_t i=0; i<6; ++i)
        {
            std::string dofname;
            switch(i)
            {
                case 0: dofname = _name+"_POS_X"; break;
                case 1: dofname = _name+"_POS_Y"; break;
                case 2: dofname = _name+"_POS_Z"; break;
                case 3: dofname = _name+"_ROT_X"; break;
                case 4: dofname = _name+"_ROT_Y"; break;
                case 5: dofname = _name+"_ROT_Z"; break;
            }
            newdof = new DegreeOfFreedom(this, dofname, dof_properties);
            newdof->_localID = i;
            newdof->_id = _robot->_dofs.size();
            _robot->_insertDof(newdof);
            _dofs.push_back(newdof);
        }
    }

    notifyPosUpdate();
    notifyVelUpdate();
    notifyAccUpdate();
    notifyDynUpdate();
}

Joint::~Joint()
{
    
}

Joint& Joint::operator=(const Joint& otherJoint)
{
    (PublicJointProperties&)(*this) = (PublicJointProperties&)(otherJoint);
    (ProtectedJointProperties&)(*this) = (ProtectedJointProperties&)(otherJoint);

    return *this;
}

Joint::Type Joint::type() const { return _type; }
void Joint::type(akin::Joint::Type newType, const akin::DofProperties& properties)
{
    _type = newType;
    _createDofs(properties);
    axis(_axis);
}

void Joint::_changeParentLink(Link *newParent)
{
    _upstreamLink = newParent;
    _parentLink = newParent;
}

void Joint::axis(const akin::Vec3& newAxis)
{
    _axis = Axis(newAxis);

    if(childLink().isAnchor() || FLOATING == _type)
    {
        _dofMatrix = Matrix6d::Identity();
        _dofMatrixDerivative = Matrix6d::Zero();
    }
    else
    {
        _dofMatrix.resize(Eigen::NoChange, 1);
        if(Joint::PRISMATIC==_type)
        {
            _dofMatrix.block<3,1>(0,0).setZero();
            _dofMatrix.block<3,1>(3,0) = _axis;
        }
        else if(Joint::REVOLUTE==_type)
        {
            _dofMatrix.block<3,1>(0,0) = _axis;
            _dofMatrix.block<3,1>(3,0).setZero();
        }
        _dofMatrixDerivative = Vector6d::Zero();
    }

    notifyPosUpdate();
    notifyVelUpdate();
    notifyAccUpdate();
    notifyDynUpdate();
}
const Vec3& Joint::axis() const { return _axis; }

const Matrix6Xd& Joint::getDofMatrix() const
{
    return _dofMatrix;
}

const Matrix6Xd& Joint::getDofMatrixDerivative() const
{
    return _dofMatrixDerivative;
}

void Joint::baseTransform(const Transform &newBaseTf)
{
    _baseTransform = newBaseTf;
    _computeRefTransform();
}

const Transform& Joint::baseTransform() const { return _baseTransform; }

void Joint::_integrate(integration_method_t method, double dt)
{
    switch(method)
    {
        case EXPLICIT_EULER: return _explicit_euler_integration(dt);
        default:
            verb.Assert(false, verbosity::ASSERT_CASUAL,
                        "Trying to integrate Joint '"+name()+"' with invalid method ("
                        +to_string(method)+")");
    }
}

void Joint::_explicit_euler_integration(double dt)
{
    // TODO
    if(REVOLUTE==_type || PRISMATIC==_type)
    {
        dof(0).value(dof(0)._value + dof(0)._velocity*dt);
        dof(0).velocity(dof(0)._velocity+ dof(0)._acceleration*dt);
    }
    else if(FLOATING==_type)
    {
        for(size_t i=0; i<3; ++i)
            dof(i).value( dof(i).value() + dof(i)._velocity*dt );

        Rotation R(dof(3).value(),Vec3(1,0,0));
        R *= Rotation(dof(4).value(),Vec3(0,1,0));
        R *= Rotation(dof(5).value(),Vec3(0,0,1));
        R *= Rotation(FreeVector(dt*Vec3(dof(3)._velocity,dof(4)._velocity,dof(5)._velocity)));

        const Vec3& angles = R.getEulerAngles();
        for(size_t i=0; i<3; ++i)
            dof(i+3).value(angles[i]);

        for(size_t i=0; i<6; ++i)
            dof(i).velocity(dof(i)._velocity + dof(i)._acceleration*dt);
    }

}

size_t Joint::id() const { return _id; }

const std::string& Joint::name() const { return _name; }

//Robot& Joint::robot() const { return *_myRobot; }

bool Joint::name(const string &new_name)
{
    if( verb.Assert(!_robot->checkForJointName(new_name),
                         verbosity::ASSERT_CRITICAL,
                         "You requested to change joint named '"+name()+"' to '"
                         +new_name+"', but robot '"+_robot->name()+"' already has "
                         +"a joint with that name!"))
        return false;
    
    StringMap::iterator n = _robot->_jointNameToIndex.find(name());
    size_t index = n->second;
    _robot->_jointNameToIndex.erase(n);
    _robot->_jointNameToIndex[new_name] = index;
    
    _name = new_name;
    
    return true;
}

Link& Joint::parentLink() { return *_parentLink; }
const Link& Joint::parentLink() const { return const_cast<Joint*>(this)->parentLink(); }

Link& Joint::childLink() { return *_childLink; }
const Link& Joint::childLink() const { return const_cast<Joint*>(this)->childLink(); }

Joint& Joint::parentJoint() { return _parentLink->parentJoint(); }
const Joint& Joint::parentJoint() const { return const_cast<Joint*>(this)->parentJoint(); }

Joint& Joint::childJoint(size_t num) { return _childLink->childJoint(num); }
const Joint& Joint::childJoint(size_t num) const
    { return const_cast<Joint*>(this)->childJoint(num); }

size_t Joint::numChildJoints() const { return _childLink->numChildJoints(); }

Link& Joint::upstreamLink() { return *_upstreamLink; }
const Link& Joint::upstreamLink() const { return const_cast<Joint*>(this)->upstreamLink(); }

Link& Joint::downstreamLink() { return *_downstreamLink; }
const Link& Joint::downstreamLink() const
    { return const_cast<Joint*>(this)->downstreamLink(); }

Joint& Joint::upstreamJoint() { return _upstreamLink->upstreamJoint(); }
const Joint& Joint::upstreamJoint() const 
    { return const_cast<Joint*>(this)->upstreamJoint(); }

Joint& Joint::downstreamJoint(size_t num) { return _downstreamLink->downstreamJoint(num); }
const Joint& Joint::downstreamJoint(size_t num) const
    { return const_cast<Joint*>(this)->downstreamJoint(num); }

size_t Joint::numDownstreamJoints() const { return _downstreamLink->numDownstreamJoints(); }

bool Joint::belongsTo(const Robot &someRobot) const { return &someRobot == _robot; }
Robot& Joint::robot() { return *_robot; }
const Robot& Joint::robot() const { return const_cast<Joint*>(this)->robot(); }

bool Joint::isDummy() const { return _isDummy; }

bool Joint::isReversed() const { return _reversed; }

bool Joint::needsPosUpdate() const { return _needsPosUpdate; }
void Joint::notifyPosUpdate()
{
    if(_isDummy)
    {
        _needsPosUpdate = false;
        return;
    }

    _needsPosUpdate = true;
    downstreamLink().notifyPosUpdate();
}

bool Joint::needsVelUpdate() const { return _needsVelUpdate; }
void Joint::notifyVelUpdate()
{
    if(_isDummy)
    {
        _needsVelUpdate = false;
        return;
    }

    _needsVelUpdate = true;
    downstreamLink().notifyVelUpdate();
}

bool Joint::needsAccUpdate() const { return _needsAccUpdate; }
void Joint::notifyAccUpdate()
{
    if(_isDummy)
    {
        _needsAccUpdate = false;
        return;
    }

    _needsAccUpdate = true;
    downstreamLink().notifyAccUpdate();
}

bool Joint::needsDynUpdate() const { return _needsDynUpdate; }
void Joint::notifyDynUpdate()
{
    if(_isDummy)
    {
        _needsDynUpdate = false;
        return;
    }

    _needsDynUpdate = true;
    downstreamLink().notifyDynUpdate();
}

std::ostream& operator<<(std::ostream& oStrStream, akin::Joint::Type type)
{
    oStrStream << Joint::type_to_string(type);
    return oStrStream;
}

std::ostream& operator<<(std::ostream& oStrStream, const akin::Joint& someJoint)
{
    oStrStream << "Joint named '" << someJoint.name() << "' with ID " << someJoint.id()
               << " connects Parent Link '" << someJoint.parentLink().name() << "' to Child '" 
               << someJoint.childLink().name() << "' for robot '" 
               << someJoint.robot().name() << "'\n";
    if(someJoint.isReversed())
        oStrStream << "[Parent/Child are currently kinematically reversed]\n";
    oStrStream << "Axis: <" << someJoint.axis().transpose() << "> (" << someJoint.type() 
               << ") with Base Transform:\n" << someJoint.baseTransform() << "\n";
    oStrStream << "Current value: " << someJoint.dof(0).value() << " (min "
               << someJoint.dof(0).min() << " | max " << someJoint.dof(0).max() << ")";
    if(!someJoint.dof(0).withinLimits())
        oStrStream << " [Currently outside its limits!]";
    oStrStream << "\n";
    
    return oStrStream;
}

