
#include "dartAkin/Wrapper.h"
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/Joint.h>

#include <dart/dynamics/RevoluteJoint.h>
#include <dart/dynamics/PrismaticJoint.h>
#include <dart/dynamics/FreeJoint.h>
#include <dart/dynamics/BallJoint.h>
#include <dart/dynamics/EulerJoint.h>
#include <dart/dynamics/PlanarJoint.h>
#include <dart/dynamics/ScrewJoint.h>
#include <dart/dynamics/TranslationalJoint.h>
#include <dart/dynamics/UniversalJoint.h>


namespace dartAkin {

Wrapper::Wrapper() :
    _dart_skeleton(nullptr),
    _akin_robot(nullptr)
{

}

Wrapper::Wrapper(std::shared_ptr<dart::dynamics::Skeleton> dartSkeleton) :
    _dart_skeleton(dartSkeleton),
    _akin_robot(nullptr)
{
    if(_dart_skeleton)
        _create_dart();
}

Wrapper::Wrapper(std::shared_ptr<akin::Robot> akinRobot) :
    _dart_skeleton(nullptr),
    _akin_robot(akinRobot)
{
    if(_akin_robot)
        _create_akin();
}

std::shared_ptr<akin::Robot> Wrapper::wrap_dart_skeleton(
        std::shared_ptr<dart::dynamics::Skeleton> dartSkeleton)
{
    _dart_skeleton = dartSkeleton;
    if(_dart_skeleton)
        _create_akin();
    else
        _akin_robot = nullptr;

    return _akin_robot;
}

std::shared_ptr<dart::dynamics::Skeleton> Wrapper::wrap_akin_robot(
        std::shared_ptr<akin::Robot> akinRobot)
{
    _akin_robot = akinRobot;
    if(_akin_robot)
        _create_dart();
    else
        _dart_skeleton = nullptr;

    return _dart_skeleton;
}

std::shared_ptr<dart::dynamics::Skeleton> Wrapper::dart_skeleton() { return _dart_skeleton; }
std::shared_ptr<const dart::dynamics::Skeleton> Wrapper::dart_skeleton() const { return _dart_skeleton; }

std::shared_ptr<akin::Robot> Wrapper::akin_robot() { return _akin_robot; }
std::shared_ptr<const akin::Robot> Wrapper::akin_robot() const { return _akin_robot; }




static void grabAkinDofProperties(dart::dynamics::Joint* joint, akin::Joint& akin_joint)
{
    for(size_t i=0; i<akin_joint.numDofs(); ++i)
    {
        joint->setPositionLowerLimit(i, akin_joint.dof(i).min());
        joint->setPositionUpperLimit(i, akin_joint.dof(i).max());

        joint->setVelocityLowerLimit(i, -akin_joint.dof(i).maxSpeed());
        joint->setVelocityUpperLimit(i,  akin_joint.dof(i).maxSpeed());

        joint->setAccelerationLowerLimit(i, -akin_joint.dof(i).maxAcceleration());
        joint->setAccelerationUpperLimit(i,  akin_joint.dof(i).maxAcceleration());
    }
}

static dart::dynamics::Joint* createDartJoint(akin::Joint& akin_joint)
{
    dart::dynamics::Joint* joint;
    if(akin_joint.type() == akin::Joint::REVOLUTE)
    {
        joint = new dart::dynamics::RevoluteJoint(akin_joint.axis(), akin_joint.name());
    }
    else if(akin_joint.type() == akin::Joint::PRISMATIC)
    {
        joint = new dart::dynamics::PrismaticJoint(akin_joint.axis(), akin_joint.name());
    }
    else if(akin_joint.type() == akin::Joint::FLOATING)
    {
        joint = new dart::dynamics::FreeJoint(akin_joint.name());
    }
    else
    {
        // TODO What to do here?
        // TODO Add more joint types as they get implemented
    }

    grabAkinDofProperties(joint, akin_joint);

    joint->setTransformFromParentBodyNode(akin_joint.baseTransform());

    return joint;
}

static void setBodyNodeProperties(dart::dynamics::BodyNode* bn, const akin::Link& link)
{
    akin::StandardInertiaParameters inertia = link.inertia();
    bn->setMass(inertia.mass);
    bn->setLocalCOM(inertia.centerOfMass);

    // Note: DART expects the MomentOfInertia to be with respect to the center of mass
    inertia.convertTo(akin::INERTIA_WRT_COM);
    bn->setMomentOfInertia(inertia.tensor(0,0), inertia.tensor(1,1), inertia.tensor(2,2),
                           inertia.tensor(0,1), inertia.tensor(0,2), inertia.tensor(1,2));
}

void Wrapper::_create_dart()
{
    _dart_skeleton.reset(new dart::dynamics::Skeleton);
    _dart_skeleton->setName(_akin_robot->name());

    dart::dynamics::BodyNode* rootBn = new dart::dynamics::BodyNode(_akin_robot->rootLink().name());
    dart::dynamics::Joint* rootJoint = createDartJoint(_akin_robot->joint(akin::BASE_INDEX));
    rootBn->setParentJoint(rootJoint);
    setBodyNodeProperties(rootBn, _akin_robot->rootLink());
    _dart_skeleton->addBodyNode(rootBn);

    _exploreLink(_akin_robot->rootLink(), *rootBn);
}

void Wrapper::_exploreLink(akin::Link& link, dart::dynamics::BodyNode& parentBn)
{
    for(size_t i=0; i<link.numChildLinks(); ++i)
    {
        akin::Link& childLink = link.childLink(i);
        if(childLink.isDummy())
        {
            std::cerr << "Error: Link named '" << link.name() << "' has a dummy child (#"
                      << i << ")!" << std::endl;
            continue;
        }

        akin::Joint& childJoint = link.childJoint(i);
        if(childJoint.isDummy())
        {
            std::cerr << "Error: Link named '" << childLink.name() << "' has a dummy parent joint!"
                      << std::endl;
            continue;
        }

        dart::dynamics::BodyNode* childBn = new dart::dynamics::BodyNode(childLink.name());
        dart::dynamics::Joint* joint = createDartJoint(childJoint);
        childBn->setParentJoint(joint);
        setBodyNodeProperties(childBn, childLink);
        parentBn.addChildBodyNode(childBn);
        _dart_skeleton->addBodyNode(childBn);

        _exploreLink(childLink, *childBn);
    }
}


static std::vector<akin::DofProperties> grabDartDofProperties(
        dart::dynamics::Joint& joint)
{
    std::vector<akin::DofProperties> dp;
    dp.reserve(joint.getNumDofs());
    for(size_t i=0; i<joint.getNumDofs(); ++i)
    {
        akin::DofProperties prop;

        prop._minValue = joint.getPositionLowerLimit(i);
        prop._maxValue = joint.getPositionUpperLimit(i);
        prop._maxSpeed = joint.getVelocityUpperLimit(i);
        prop._maxAcceleration = joint.getAccelerationUpperLimit(i);
        prop._maxEffort = joint.getForceUpperLimit(i);

        dp.push_back(prop);
    }

    return dp;
}

static akin::ProtectedJointProperties grabDartJointProperties(
        dart::dynamics::Joint& joint,
        const Eigen::Isometry3d& parentNodeOffsetFromItsJoint = Eigen::Isometry3d::Identity())
{
    using namespace dart::dynamics;
    // TODO
    akin::ProtectedJointProperties p;
    p._name = joint.getName();
    p._baseTransform = parentNodeOffsetFromItsJoint*
            joint.getTransformFromParentBodyNode();

    const RevoluteJoint* rj = dynamic_cast<const RevoluteJoint*>(&joint);
    if(rj)
    {
        p._type = akin::Joint::REVOLUTE;
        p._axis = rj->getAxis();
        return p;
    }

    const PrismaticJoint* pj = dynamic_cast<const PrismaticJoint*>(&joint);
    if(pj)
    {
        p._type = akin::Joint::PRISMATIC;
        p._axis = pj->getAxis();
        return p;
    }

    const FreeJoint* fj = dynamic_cast<const FreeJoint*>(&joint);
    if(fj)
    {
        p._type = akin::Joint::FLOATING;
        p._axis = akin::Vec3(1,0,0);
        return p;
    }

    const BallJoint* bj = dynamic_cast<const BallJoint*>(&joint);
    if(bj)
    {
        // TODO
        p._type = akin::Joint::CUSTOM;
        return p;
    }

    const EulerJoint* ej = dynamic_cast<const EulerJoint*>(&joint);
    if(ej)
    {
        // TODO
        p._type = akin::Joint::CUSTOM;
        return p;
    }

    const PlanarJoint* plj = dynamic_cast<const PlanarJoint*>(&joint);
    if(plj)
    {
        // TODO
        p._type = akin::Joint::CUSTOM;
        return p;
    }

    const ScrewJoint* sj = dynamic_cast<const ScrewJoint*>(&joint);
    if(sj)
    {
        // TODO
        p._type = akin::Joint::CUSTOM;
        return p;
    }

    const TranslationalJoint* tj = dynamic_cast<const TranslationalJoint*>(&joint);
    if(tj)
    {
        // TODO
        p._type = akin::Joint::CUSTOM;
        return p;
    }

    const UniversalJoint* uj = dynamic_cast<const UniversalJoint*>(&joint);
    if(uj)
    {
        // TODO
        p._type = akin::Joint::CUSTOM;
        return p;
    }

    return p;
}

static void grabBodyNodeProperties(dart::dynamics::BodyNode& bn, akin::Link& currentLink)
{
    akin::StandardInertiaParameters inertia;
    inertia.mass = bn.getMass();
    inertia.centerOfMass = bn.getParentJoint()->getTransformFromChildBodyNode().inverse()*
            bn.getLocalCOM();
    bn.getMomentOfInertia(inertia.tensor(0,0), inertia.tensor(1,1), inertia.tensor(2,2),
                          inertia.tensor(0,1), inertia.tensor(0,2), inertia.tensor(1,2));
    inertia.mirrorTheCurrentTensorValues();
    currentLink.inertia(inertia);
}

void Wrapper::_create_akin()
{
    _akin_robot.reset(new akin::Robot);
    _akin_robot->link(0).name(_dart_skeleton->getName());
    dart::dynamics::BodyNode* root = _dart_skeleton->getRootBodyNode();
    if(!root)
    {
        std::cout << "DART skeleton named '" << _dart_skeleton->getName()
                  << "' has no root BodyNode!" << std::endl;
        return;
    }

    grabBodyNodeProperties(*root, _akin_robot->link(0));
    _exploreBodyNode(*root, _akin_robot->link(0));
}

void Wrapper::_exploreBodyNode(dart::dynamics::BodyNode& bn, akin::Link& parentLink)
{
    const Eigen::Isometry3d& offset = bn.getParentJoint()->
            getTransformFromChildBodyNode().inverse();

    for(size_t i=0; i<bn.getNumChildBodyNodes(); ++i)
    {
        dart::dynamics::BodyNode* childBn = bn.getChildBodyNode(i);
        if(!childBn)
        {
            std::cerr << "Error: Index confusion while parsing dart skeleton: "
                      << "BodyNode '" << bn.getName() << "' says it has "
                      << bn.getNumChildBodyNodes() << " children, "
                      << "but child #" << i << " returned null!" << std::endl;
            continue;
        }

        dart::dynamics::Joint* childJoint = childBn->getParentJoint();
        if(!childJoint)
        {
            std::cerr << "BodyNode named '" << bn.getName() << "' has a null parent joint!"
                      << std::endl;
            continue;
        }

        akin::ProtectedJointProperties joint_properties = grabDartJointProperties(*childJoint, offset);
        std::vector<akin::DofProperties> dof_properties = grabDartDofProperties(*childJoint);
        int newLink = _akin_robot->createJointLinkPair(parentLink, childBn->getName(),
                                         joint_properties, dof_properties);
        grabBodyNodeProperties(*childBn, _akin_robot->link(newLink));
        _exploreBodyNode(*childBn, _akin_robot->link(newLink));
    }
}


} // dartAkin
