
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

//bool Joint::value(double newJointValue)
//{
//    bool inBounds = true;
    
//    if(newJointValue != newJointValue)
//    {
//        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set value for joint '"
//                    +name()+"' to NaN!");
//        return false;
//    }
    
//    if(newJointValue < _minValue)
//    {
//        if(_myRobot->enforceJointLimits())
//            newJointValue = _minValue;
//        inBounds = false;
//    }
    
//    if(newJointValue > _maxValue)
//    {
//        if(_myRobot->enforceJointLimits())
//            newJointValue = _maxValue;
//        inBounds = false;
//    }
    
//    if(newJointValue == _value)
//        return inBounds;
    
//    _value = newJointValue;
    
//    _computeRefTransform();
    
//    return inBounds;
//}

//double Joint::value() const { return _value; }

//bool Joint::velocity(double newJointVelocity)
//{
//    bool inBounds = true;

//    if(newJointVelocity != newJointVelocity)
//    {
//        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set velocity for joint '"
//                    +name()+"' to Nan!");
//        return false;
//    }

//    if(fabs(newJointVelocity) > _maxSpeed)
//    {
//        if(_myRobot->enforceJointLimits())
//            newJointVelocity = newJointVelocity>0? _maxSpeed : -_maxSpeed;
//        inBounds = false;
//    }

//    if(newJointVelocity==_velocity)
//        return inBounds;

//    _velocity = newJointVelocity;

//    Frame::coord_t C;
//    if(REVOLUTE == _type)
//        C = Frame::ANGULAR;
//    else if(PRISMATIC == _type)
//        C = Frame::LINEAR;
//    else
//        return false;

//    if(_reversed)
//        _downstreamLink->relativeVelocity(
//                    -_velocity*_downstreamLink->respectToRef().rotation()*_axis, C);
//        // TODO: Test the crap out of this _reversed case
//    else
//        _downstreamLink->relativeVelocity(_velocity*_baseTransform.rotation()*_axis, C);

//    return inBounds;
//}

//double Joint::velocity() const { return _velocity; }

//bool Joint::acceleration(double newJointAcceleration)
//{
//    bool inBounds = true;

//    if(newJointAcceleration != newJointAcceleration)
//    {
//        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set acceleration for joint '"
//                    +name()+"' to NaN!");
//        return false;
//    }

//    if(fabs(newJointAcceleration) > _maxAcceleration)
//    {
//        if(_myRobot->enforceJointLimits())
//            newJointAcceleration = newJointAcceleration>0? _maxAcceleration : -_maxAcceleration;
//        inBounds = false;
//    }

//    if(newJointAcceleration==_acceleration)
//        return inBounds;

//    _acceleration = newJointAcceleration;

//    Frame::coord_t C;
//    if(REVOLUTE == _type)
//        C = Frame::ANGULAR;
//    else if(PRISMATIC == _type)
//        C = Frame::LINEAR;
//    else
//        return false;

//    if(_reversed)
//        _downstreamLink->relativeAcceleration(
//                    -_acceleration*_downstreamLink->respectToRef().rotation()*_axis, C);
//        // TODO: Test the crap out of this _reversed case
//    else
//        _downstreamLink->relativeAcceleration(
//                    _acceleration*_baseTransform.rotation()*_axis, C);

//    return inBounds;
//}

//double Joint::acceleration() const
//{
//    if(downstreamLink().getDynamicsMode()==INVERSE)
//        return _acceleration;

//    if(!isDummy())
//    {
//        if(downstreamLink().needsDynUpdate())
//            downstreamLink()._computeABA_pass3();
//    }
//    else
//    {
//        if(_myRobot->anchorLink().needsDynUpdate())
//            _myRobot->anchorLink()._computeABA_pass3();
//    }

//    return _acceleration;
//}

//void Joint::_computeTransformedJointAxis(Vec3 &z_i, const akin::Frame& refFrame) const
//{
//    z_i = _reversed ?
//            Vec3(-childLink().respectToRef().rotation()*_axis) :
//            Vec3(childLink().respectToRef().rotation()*_axis);
    
//    // Put z_i into the reference frame
//    if(refFrame.isWorld())
//        z_i = childLink().respectToWorld().rotation()*z_i;
//    else
//        z_i = refFrame.respectToWorld().rotation().transpose()
//              *childLink().respectToWorld().rotation()*z_i;
//}

//Vec3 Joint::_computePosJacobian(const Vec3 &z_i, const KinTranslation &point,
//                                const Frame &refFrame) const
//{
//    if(type()==REVOLUTE)
//    {
//        return z_i.cross( point.withRespectTo(refFrame)
//                          - childLink().withRespectTo(refFrame).translation() );
//    }
//    else if(type()==PRISMATIC)
//        return z_i;
    
//    return Vec3::Zero();
//}

//Vec3 Joint::_computeRotJacobian(const Vec3 &z_i) const
//{
//    if(type()==REVOLUTE)
//        return z_i;
//    else if(type()==PRISMATIC)
//        return Vec3::Zero();
    
//    return Vec3::Zero();
//}

//Vec3 Joint::Jacobian_rotOnly(const KinTranslation &point, const Frame &refFrame,
//                         bool checkDependence) const
//{
//    if(checkDependence)
//    {
//        if(!point.descendsFrom(childLink()))
//            return Vec3::Zero();
//    }
    
//    Vec3 z_i;
//    _computeTransformedJointAxis(z_i, refFrame);
    
//    return _computeRotJacobian(z_i);
//}

//bool Joint::torque(double newTorque)
//{

//    bool inBounds = true;

//    if(newTorque != newTorque)
//    {
//        verb.Assert(false, verbosity::ASSERT_CRITICAL, "Attempting to set torque for joint '"
//                    +name()+"' to NaN!");
//        return false;
//    }

//    if(fabs(newTorque) > _maxTorque)
//    {
//        if(_myRobot->enforceJointLimits())
//            newTorque = newTorque<0? -_maxTorque : _maxTorque;
//        inBounds = false;
//    }

//    if(newTorque == _torque)
//        return inBounds;

//    _torque = newTorque;

//    if(_myRobot->getDynamicsMode()==FORWARD)
//        childLink().notifyDynUpdate();

//    return inBounds;
//}

//double Joint::torque() const
//{
//    return _torque;
//}

//Screw Joint::reciprocalWrench() const
//{
//    // TODO FIXME
//    verb.Assert(false, verbosity::ASSERT_CASUAL, "Joint reciprocal wrench calculations are not ready yet");
//    return Screw::Zero();
//}

//Vec3 Joint::Jacobian_posOnly(const KinTranslation &point, const Frame &refFrame,
//                      bool checkDependence) const
//{
//    if(checkDependence)
//    {
//        if(!point.descendsFrom(childLink()))
//            return Vec3::Zero();
//    }
    
//    Vec3 z_i;
//    _computeTransformedJointAxis(z_i, refFrame);
    
//    return _computePosJacobian(z_i, point, refFrame);
//}

//Screw Joint::Jacobian(const KinTranslation& point, const Frame &refFrame,
//                      bool checkDependence) const
//{
//    if(checkDependence)
//    {
//        if(!point.descendsFrom(childLink()))
//            return Screw::Zero();
//    }
    
//    Vec3 z_i;
//    _computeTransformedJointAxis(z_i, refFrame);
    
//    return Screw(_computePosJacobian(z_i,point,refFrame), _computeRotJacobian(z_i));
//}

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
}

void Joint::_createDofs(const DofProperties& dof_properties)
{
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

//Joint::Joint(Robot *mRobot, size_t jointID, const string &jointName,
//             Link *mParentLink, Link *mChildLink,
//             const Transform &mBaseTransform,
//             const Vec3& mJointAxis, akin::Joint::Type mType,
//             double mininumValue, double maximumValue,
//             double maxSpeed, double maxAcceleration, double maxTorque) :
//    ProtectedJointProperties(jointName, mBaseTransform, mJointAxis, mType),
//    verb(mRobot->verb),
//    _parentLink(mParentLink),
//    _childLink(mChildLink),
//    _reversed(false),
//    _upstreamLink(mParentLink),
//    _downstreamLink(mChildLink),
//    _robot(mRobot)
//{
//    _id = jointID;
//    axis(_axis);
    
//    if( !mChildLink->isDummy() )
//        verb.Assert(mParentLink == &mChildLink->refFrame(), verbosity::ASSERT_CRITICAL,
//                "Joint '"+jointName+"' wants to connect '"+mChildLink->name()
//                +"' to '"+mParentLink->name()+"' but '"+mChildLink->name()
//                +"' is in the reference frame of '"+mChildLink->refFrame().name()
//                +"'!", " A joint must always connect a link to its parent frame!");
//}

Joint::~Joint()
{
    
}

Joint& Joint::operator=(const Joint& otherJoint)
{
    (PublicJointProperties&)(*this) = (PublicJointProperties&)(otherJoint);
    (ProtectedJointProperties&)(*this) = (ProtectedJointProperties&)(otherJoint);

    return *this;
}

//bool Joint::min(double newMinValue)
//{
//    bool inBounds = true;
//    if(newMinValue > _maxValue)
//    {
//        newMinValue = _maxValue;
//        inBounds = false;
//    }
    
//    value(value());
    
//    return inBounds;
//}

//double Joint::min() const { return _minValue; }

//bool Joint::max(double newMaxValue)
//{
//    bool inBounds = true;
//    if(newMaxValue < _minValue)
//    {
//        newMaxValue = _minValue;
//        inBounds = false;
//    }
    
//    value(value());
    
//    return inBounds;
//}

//double Joint::max() const { return _maxValue; }

//bool Joint::withinLimits() const
//{
//    return withinLimits(value());
//}

//bool Joint::withinLimits(double someValue) const
//{
//    if( min() <= someValue && someValue <= max() )
//        return true;
    
//    return false;
//}

Joint::Type Joint::type() const { return _type; }
void Joint::type(akin::Joint::Type newType) { _type = newType; _computeRefTransform(); }

void Joint::_changeParentLink(Link *newParent)
{
    _upstreamLink = newParent;
    _parentLink = newParent;
}

void Joint::axis(const akin::Vec3& newAxis)
{
    _axis = Axis(newAxis);

    if(childLink().isAnchor())
    {
        _dofMatrix = Matrix6d::Identity();
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
    }

    _computeRefTransform();
}
const Vec3& Joint::axis() const { return _axis; }

const Matrix6Xd& Joint::getDofMatrix() const
{
    return _dofMatrix;
}

void Joint::baseTransform(const Transform &newBaseTf)
{
    _baseTransform = newBaseTf;
    _computeRefTransform();
}

const Transform& Joint::baseTransform() const { return _baseTransform; }

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

