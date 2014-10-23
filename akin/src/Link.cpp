
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

    // TODO: Consider computing this when new inertial parameters are passed in
    _Ia.block<3,3>(0,0) = _inertiaTensor_wrtLocalFrame;
    _Ia.block<3,3>(0,3) = mass*skew(com.respectToRef());
    _Ia.block<3,3>(3,0) = -_Ia.block<3,3>(0,3);
    _Ia.block<3,3>(3,3) = mass*Eigen::Matrix3d::Identity();

    Spatial v;
    v.upper() = respectToWorld().rotation()*angularVelocity();
    v.lower() = respectToWorld().rotation()*linearVelocity();

    _c.block<3,1>(0,0) = v.upper().cross(relativeAngularVelocity());
    _c.block<3,1>(0,0) = v.lower().cross(relativeAngularVelocity())
            + v.upper().cross(relativeLinearVelocity());

    _pa.block<3,1>(0,0) = v.upper().cross(_Ia.block<3,3>(0,0)*v.upper())
            + mass*com.cross(v.upper().cross(v.lower()));
    _pa.block<3,1>(3,0) = mass*v.upper().cross(v.upper().cross(com))
            + mass*v.upper().cross(v.lower());

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

    const Matrix6Xd& s = parentJoint().getDofMatrix();

    _h = _Ia*s;
    _d = s.transpose()*_h;

    Spatial F;
    const Eigen::Matrix3d& R = respectToWorld().rotation();
    const Eigen::Vector3d& g = R.transpose()*_gravity.respectToWorld();

    F.upper() = R.transpose()*_appliedMoments_wrtWorld + mass*com.cross(g);
    F.lower() = R.transpose()*_appliedForces_wrtWorld + mass*g;

    if(isAnchor())
    {
        F[0] += _myRobot->joint(DOF_ROT_X)._torque;
        F[1] += _myRobot->joint(DOF_ROT_Y)._torque;
        F[2] += _myRobot->joint(DOF_ROT_Z)._torque;

        F[3] += _myRobot->joint(DOF_POS_X)._torque;
        F[4] += _myRobot->joint(DOF_POS_Y)._torque;
        F[5] += _myRobot->joint(DOF_POS_Z)._torque;
    }
    else
    {
        F += s*parentJoint()._torque;
    }

    _u = s.transpose()*F - s.transpose()*_pa;

    _Ia = _Ia - _d.inverse()*_h*_h.transpose();
    _pa = _pa + _Ia*_c + _d.inverse()*_h*_u;

    _needsAbiUpdate = false;
}

void Link::_computeABA_pass3() const
{
    if(isDummy())
        return;

    const Matrix6d& X = spatial_transform(respectToRef());
    const Matrix6Xd& s = parentJoint().getDofMatrix();
    const Frame& frame = isAnchor()? _myRobot->refFrame() : (const Frame&)parentLink();

    Vector6d a_ref;
    a_ref.block<3,1>(0,0) = frame.respectToWorld().rotation().transpose()*
            frame.angularAcceleration();
    a_ref.block<3,1>(3,0) = frame.respectToWorld().rotation().transpose()*(
            frame.linearAcceleration()
            - frame.angularVelocity().cross(frame.linearVelocity()) );

    _a = X*a_ref + _ABA_c();
    _qdd = _ABA_d().inverse()*(_ABA_u()-_ABA_h().transpose()*_a);
    _arel = s*_qdd;
    _a = _a + _arel;

    if(isAnchor())
    {
        // ... Why don't these need const_cast?
        _myRobot->joint(DOF_ROT_X).acceleration(_qdd[0]);
        _myRobot->joint(DOF_ROT_Y).acceleration(_qdd[1]);
        _myRobot->joint(DOF_ROT_Z).acceleration(_qdd[2]);

        _myRobot->joint(DOF_POS_X).acceleration(_qdd[3]);
        _myRobot->joint(DOF_POS_Y).acceleration(_qdd[4]);
        _myRobot->joint(DOF_POS_Z).acceleration(_qdd[5]);
    }
    else
    {
        // TODO: Think about this const_cast and decide if it's both necessary and appropriate
        const_cast<Joint&>(parentJoint()).acceleration(_qdd[0]);
    }

    _needsDynUpdate = false;
}

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
