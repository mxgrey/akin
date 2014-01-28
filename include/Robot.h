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

    Robot();

    void name(std::string newName);
    std::string name() const;

    bool createRootLink(std::string rootLinkName);

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

    bool belongsTo(const Link& someLink) const;
    bool belongsTo(const Joint& someJoint) const;

    verbosity verb;

protected:

    void _insertLink(Link* newLink);
    void _insertJoint(Joint* newJoint);

    std::string _name;

    Link* _anchor;
    Link* _root;

    Link* _dummyLink;
    Joint* _dummyJoint;

    JointPtrArray _joints;
    LinkPtrArray _links;

    StringMap _linkNameToIndex;
    StringMap _jointNameToIndex;

    Robot* _myRobot;
    void _loseJoint(Joint* lostJoint);
    void _loseLink(Link* lostLink);

};

} // namespace akin

#endif // ROBOT_H
