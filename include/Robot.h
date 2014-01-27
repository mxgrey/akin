#ifndef ROBOT_H
#define ROBOT_H

#include "Joint.h"
#include <map>

namespace akin {

class Link;

typedef std::map<std::string,size_t> StringMap;
typedef std::vector<Joint*> JointPtrArray;
typedef std::vector<Link*> LinkPtrArray;

class Robot
{
public:

    friend class Joint;
    friend class Link;



    bool createJointLinkPair(Link& parentLink,
                             std::string newLinkName,
                             std::string newJointName,
                             const Transform& baseTransform,
                             const Axis& jointAxis,
                             Joint::Type jointType,
                             double minJointValue,
                             double maxJointValue);

    Joint& joint(size_t jointNum) const;
    Joint& joint(const std::string& jointName) const;

    Link& link(size_t linkNum) const;
    Link& link(const std::string& linkName) const;



protected:

    Link* _anchor;
    Link* _root;

    Link* _dummyLink;
    Joint* _dummyJoint;

    JointPtrArray _joints;
    LinkPtrArray _links;

    StringMap _linkNameToIndex;
    StringMap _jointNameToIndex;

};

} // namespace akin

#endif // ROBOT_H
