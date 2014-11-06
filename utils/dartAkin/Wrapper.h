#ifndef DARTAKIN_WRAPPER_H
#define DARTAKIN_WRAPPER_H

#include <akin/Robot.h>
#include <dart/dynamics/Skeleton.h>

namespace dartAkin {

class Wrapper
{
public:

    Wrapper();
    Wrapper(dart::dynamics::Skeleton& dartSkeleton);
    Wrapper(akin::Robot& akinRobot);

    akin::Robot* wrap_dart_skeleton(dart::dynamics::Skeleton& dartSkeleton);
    dart::dynamics::Skeleton* wrap_akin_robot(akin::Robot& akinRobot);

    dart::dynamics::Skeleton* dart_skeleton();
    const dart::dynamics::Skeleton* dart_skeleton() const;

    akin::Robot* akin_robot();
    const akin::Robot* akin_robot() const;




protected:

    void _create_dart();

    void _create_akin();
    void _exploreBodyNode(dart::dynamics::BodyNode& bn, akin::Link& parentLink);

    dart::dynamics::Skeleton* _dart_skeleton;
    akin::Robot* _akin_robot;

};

} // dartAkin

#endif // WRAPPER_H
