
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

const Transform& Link::respectToRef() const
{
    if(_upstreamJoint->needsPosUpdate())
        _upstreamJoint->_computeRefTransform();

    return _respectToRef;
}

void Link::respectToRef(const Transform &)
{
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "You cannot use the respectToRef(~) function to set the relative transform of "
                "a link. Use its degrees of freedom instead!");
}

const Velocity& Link::relativeLinearVelocity() const
{
    if(_upstreamJoint->needsVelUpdate())
        _upstreamJoint->_computeRelVelocity();

    return _relativeLinearVel;
}

void Link::relativeLinearVelocity(const Velocity &)
{
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "You cannot use the relativeLinearVelocity(~) function to set the velocity of "
                "a link. Use its degrees of freedom instead!");
}

const Velocity& Link::relativeAngularVelocity() const
{
    if(_upstreamJoint->needsVelUpdate())
        _upstreamJoint->_computeRelVelocity();

    return _relativeAngularVel;
}

void Link::relativeAngularVelocity(const Velocity &)
{
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "You cannot use the relativeAngularVelocity(~) function to set the velocity of "
                "a link. Use its degrees of freedom instead!");
}

const Acceleration& Link::relativeLinearAcceleration() const
{
    if(INVERSE==_mode)
    {
        if(_upstreamJoint->needsAccUpdate())
            _upstreamJoint->_computeRelAcceleration();
    }
    else
    {
        if(_needsDynUpdate)
            _computeABA_pass3();
    }

    return _relativeLinearAcc;
}

void Link::relativeLinearAcceleration(const Acceleration &)
{
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "You cannot use the relativeLinearAcceleration(~) function to set the acceleration "
                "of a link. Use its degrees of freedom instead!");
}

const Acceleration& Link::relativeAngularAcceleration() const
{
    if(INVERSE==_mode)
    {
        if(_upstreamJoint->needsAccUpdate())
            _upstreamJoint->_computeRelAcceleration();
    }
    else
    {
        if(_needsDynUpdate)
            _computeABA_pass3();
    }

    return _relativeAngularAcc;
}

void Link::relativeAngularAcceleration(const Acceleration &) const
{
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "You cannot use the relativeAngularAcceleration(~) function to set the acceleration"
                " of a link. Use its degrees of freedom instead!");
}

const string& Link::name() const
{
    return KinObject::name();
}

bool Link::name(const string& newName)
{
    if( !_myRobot->verb.Assert(!_myRobot->checkForLinkName(newName),
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
    _attachment = _upstreamJoint->_parentLink;
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
    if( !_myRobot->verb.Assert(_parentJoint != nullptr, verbosity::ASSERT_CRITICAL,
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

void Link::setDynamicsMode(dynamics_mode_t mode)
{
    if(_mode==mode)
        return;

    for(size_t i=0, I=numManips(); i<I; ++i)
    {
        for(size_t j=0, J=manip(i).numItems(); j<J; ++j)
            manip(i).item(j)->setDynamicsMode(mode);

        for(size_t j=0, J=manip(i).numRobots(); j<J; ++j)
            manip(i).robot(j)->setDynamicsMode(mode);
    }

    for(size_t i=0, I=numDownstreamLinks(); i<I; ++i)
        downstreamLink(i).setDynamicsMode(mode);

    upstreamLink().setDynamicsMode(mode);
}

void Link::integrate(integration_method_t method, double dt)
{
    robot().integrate(method, dt);
}

void Link::_explicit_euler_integration(double dt)
{
    robot().integrate(EXPLICIT_EULER,dt);
}

bool Link::notifyDynUpdate()
{
    if(!InertiaBase::notifyDynUpdate())
        return false;

    for(size_t i=0, I=numManips(); i<I; ++i)
    {
        for(size_t j=0, J=manip(i).numItems(); j<J; ++j)
            manip(i).item(j)->notifyDynUpdate();

        for(size_t j=0, J=manip(i).numRobots(); j<J; ++j)
            manip(i).robot(j)->notifyDynUpdate();
    }

    for(size_t i=0, I=numDownstreamLinks(); i<I; ++i)
        downstreamLink(i).notifyDynUpdate();

    upstreamLink().notifyDynUpdate();

    return true;
}

void Link::_computeABA_pass2() const
{
    if(isDummy())
        return;

    Spatial v;
    v.upper() = respectToWorld().rotation()*angularVelocity();
    v.lower() = respectToWorld().rotation()*linearVelocity();

    _c.block<3,1>(0,0) = v.upper().cross(relativeAngularVelocity());
    _c.block<3,1>(0,0) = v.lower().cross(relativeAngularVelocity())
            + v.upper().cross(relativeLinearVelocity());
    _c += parentJoint().getDofMatrixDerivative()*parentJoint().velocities();

    _pa.block<3,1>(0,0) = v.upper().cross(_Ia.block<3,3>(0,0)*v.upper())
            + mass()*_inertia.centerOfMass.cross(v.upper().cross(v.lower()));
    _pa.block<3,1>(3,0) = mass()*v.upper().cross(v.upper().cross(_inertia.centerOfMass))
            + mass()*v.upper().cross(v.lower());

    for(size_t i=0, end=numDownstreamLinks(); i<end; ++i)
    {
        const Link& child = childLink(i);
        const Matrix6d& iXj_star = spatial_transform_star(child.respectToRef().inverse());
        const Matrix6d& jXi = spatial_transform(child.respectToRef());

        _Ia += iXj_star*child._ABA_Ia()*jXi;
        _pa += iXj_star*child._ABA_pa();
    }


    for(size_t i=0, M=numManips(); i<M; ++i)
    {
        const Manipulator& m = manip(i);

        // TODO: Consider condensing all these very similar blocks of code
        for(size_t j=0, end=m.numItems(); j<end; ++j)
        {
            const Body& child = *m.item(j);
            const Matrix6d& iXj_star = spatial_transform_star(child.respectToRef().inverse());
            const Matrix6d& jXi = spatial_transform(child.respectToRef());

            _Ia += iXj_star*child._ABA_Ia()*jXi;
            _pa += iXj_star*child._ABA_pa();
        }

        for(size_t j=0, end=m.numRobots(); j<end; ++j)
        {
            const Link& child = m.robot(j)->anchorLink();
            const Matrix6d& iXj_star = spatial_transform_star(child.respectToRef().inverse());
            const Matrix6d& jXi = spatial_transform(child.respectToRef());

            _Ia += iXj_star*child._ABA_Ia()*jXi;
            _pa += iXj_star*child._ABA_pa();
        }
    }

    Spatial F;
    const Eigen::Matrix3d& R = respectToWorld().rotation();
    const Eigen::Vector3d& g = R.transpose()*_gravity.respectToWorld();

    F.upper() = R.transpose()*_appliedMoments_wrtWorld
            + _inertia.mass*_inertia.centerOfMass.cross(g);
    F.lower() = R.transpose()*_appliedForces_wrtWorld + _inertia.mass*g;

    _pa -= F;

    const Matrix6Xd& s = parentJoint().getDofMatrix();

    _h = _Ia*s;
    _D = (s.transpose()*_h).inverse();
    _u = parentJoint().efforts() - _h.transpose()*_c - s.transpose()*_pa;

    _Ia = _Ia - _D.inverse()*_h*_h.transpose();
    _pa = _pa + _Ia*_c + _D.inverse()*_h*_u;

    _needsAbiUpdate = false;
}

//void Link::_computeABA_pass3() const
//{
//    if(isDummy())
//        return;

//    const Matrix6d& X = spatial_transform(respectToRef());
//    const Matrix6Xd& s = parentJoint().getDofMatrix();
//    const Frame& frame = isAnchor()? _myRobot->refFrame() : (const Frame&)parentLink();

//    // TODO: Use attachment pointer to make this whole thing a lot smarter
//    // i.e. attach child links to their parents and leave the root link
//    // pointer blank. In fact, this should make it so that no special case
//    // is really needed for link (as opposed to bodies)
//    Vector6d a_ref;
//    a_ref.block<3,1>(0,0) = frame.respectToWorld().rotation().transpose()*
//            frame.angularAcceleration();
//    a_ref.block<3,1>(3,0) = frame.respectToWorld().rotation().transpose()*(
//            frame.linearAcceleration()
//            - frame.angularVelocity().cross(frame.linearVelocity()) );

//    _a = X*a_ref + _ABA_c();
//    _qdd = _ABA_D().inverse()*(_ABA_u()-_ABA_h().transpose()*_a);
//    _arel = s*_qdd;
//    _a = _a + _arel;

//    _needsDynUpdate = false;
//}

std::ostream& operator<<(std::ostream& stream, const akin::Link& someLink)
{
    stream << "Link named '" << someLink.name() << "' ";
    if(someLink.isRoot())
    {
        stream << "is the root link ";
    }
    if(someLink.isAnchor())
    {
        if(someLink.isRoot())
            stream << "and ";
        stream << "is the anchor link ";
    }
    
    if(!someLink.isRoot())
    {
        if(someLink.isAnchor())
            stream << "with ";
        else
            stream << " has ";
        stream << "parent joint " << someLink.parentJoint().name();
    }
    
    stream << "\n";
    
    if(someLink.numChildJoints() == 0)
    {
        stream << "This link has no child joints";
    }
    else
    {
        stream << "Child joints are: ";
        for(size_t i=0; i<someLink.numChildJoints(); ++i)
        {
            stream << someLink.childJoint(i).name();
            if(i+1 < someLink.numChildJoints())
                stream << ", ";
        }
    }
    
    stream << "\n" << (akin::Body&)someLink;
    
    return stream;
}
