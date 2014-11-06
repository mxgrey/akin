
#include "dartAkin/Wrapper.h"
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/Joint.h>

namespace dartAkin {

Wrapper::Wrapper() :
    _dart_skeleton(nullptr),
    _akin_robot(nullptr)
{

}

Wrapper::Wrapper(dart::dynamics::Skeleton &dartSkeleton) :
    _dart_skeleton(&dartSkeleton),
    _akin_robot(nullptr)
{
    if(_dart_skeleton)
        _create_dart();
}

Wrapper::Wrapper(akin::Robot& akinRobot) :
    _dart_skeleton(nullptr),
    _akin_robot(&akinRobot)
{
    if(_akin_robot)
        _create_akin();
}

akin::Robot* Wrapper::wrap_dart_skeleton(dart::dynamics::Skeleton& dartSkeleton)
{
    _dart_skeleton = &dartSkeleton;
    if(_dart_skeleton)
        _create_akin();
    else
        _akin_robot = nullptr;

    return _akin_robot;
}

dart::dynamics::Skeleton* Wrapper::wrap_akin_robot(akin::Robot& akinRobot)
{
    _akin_robot = &akinRobot;
    if(_akin_robot)
        _create_dart();
    else
        _dart_skeleton = nullptr;

    return _dart_skeleton;
}

dart::dynamics::Skeleton* Wrapper::dart_skeleton() { return _dart_skeleton; }
const dart::dynamics::Skeleton* Wrapper::dart_skeleton() const { return _dart_skeleton; }

akin::Robot* Wrapper::akin_robot() { return _akin_robot; }
const akin::Robot* Wrapper::akin_robot() const { return _akin_robot; }



void Wrapper::_create_dart()
{
    // TODO
}

void Wrapper::_create_akin()
{
    // TODO
}

static akin::ProtectedJointProperties grabJointProperties(dart::dynamics::Joint& djoint)
{
    // TODO
//    akin::ProtectedJointProperties p;
//    p._name = djoint.getName();
//    p._baseTransform = djoint.getTransformFromParentBodyNode();
//    djoint.getTransformFromChildBodyNode()
}

static void grabBodyNodeProperties(dart::dynamics::BodyNode& bn, akin::Link& currentLink)
{

}

void Wrapper::_exploreBodyNode(dart::dynamics::BodyNode& bn, akin::Link& parentLink)
{
    for(size_t i=0; i<bn.getNumChildBodyNodes(); ++i)
    {
        dart::dynamics::Joint* djoint = bn.getParentJoint();
        if(!djoint)
        {
            std::cerr << "BodyNode named '" << bn.getName() << "' has a null parent joint!"
                      << std::endl;
            continue;
        }

        akin::ProtectedJointProperties properties = grabJointProperties(*djoint);



    }
}


} // dartAkin
