#ifndef DARTAKIN_WRAPPER_H
#define DARTAKIN_WRAPPER_H

#include <akin/Robot.h>
#include <dart/dynamics/Skeleton.h>

#include <memory>

namespace dartAkin {

class Wrapper
{
public:

    Wrapper();
    Wrapper(std::shared_ptr<dart::dynamics::Skeleton> dartSkeleton);
    Wrapper(std::shared_ptr<akin::Robot> akinRobot);

    std::shared_ptr<akin::Robot> wrap_dart_skeleton(
            std::shared_ptr<dart::dynamics::Skeleton> dartSkeleton);
    std::shared_ptr<dart::dynamics::Skeleton> wrap_akin_robot(
            std::shared_ptr<akin::Robot> akinRobot);

    std::shared_ptr<dart::dynamics::Skeleton> dart_skeleton();
    std::shared_ptr<const dart::dynamics::Skeleton> dart_skeleton() const;

    std::shared_ptr<akin::Robot> akin_robot();
    std::shared_ptr<const akin::Robot> akin_robot() const;




protected:

    void _create_dart();
    void _exploreLink(akin::Link& link, dart::dynamics::BodyNode& parentBn);

    void _create_akin();
    void _exploreBodyNode(dart::dynamics::BodyNode& bn, akin::Link& parentLink);

    std::shared_ptr<dart::dynamics::Skeleton> _dart_skeleton;
    std::shared_ptr<akin::Robot> _akin_robot;

};

} // dartAkin

#endif // WRAPPER_H
