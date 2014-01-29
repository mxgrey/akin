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

    Robot(verbosity::verbosity_level_t report_level = verbosity::LOG);
    ~Robot();

    void name(std::string newName);
    inline std::string name() const { return _name; }

    bool createRootLink(std::string rootLinkName, Frame& referenceFrame);

    bool createJointLinkPair(Link& parentLink,
                             std::string newLinkName,
                             std::string newJointName,
                             const Transform& baseTransform,
                             const Axis& jointAxis,
                             Joint::Type jointType,
                             double minJointValue,
                             double maxJointValue);
    
    void removeConnection(size_t jointNum, bool fillInGap=false);
    void removeConnection(std::string& jointName, bool fillInGap=false);

    Joint& joint(size_t jointNum);
    Joint& joint(const std::string& jointName);
    inline size_t numJoints() { return _joints.size(); }

    Link& link(size_t linkNum);
    Link& link(const std::string& linkName);
    inline size_t numLinks() { return _links.size(); }

    bool belongsTo(const Link& someLink) const;
    bool belongsTo(const Joint& someJoint) const;
    
    bool checkForLinkName(const std::string &name) const;
    bool checkForJointName(const std::string &name) const;

    inline Link& anchorLink() { return *_anchor; }
    void anchorLink(Link& newAnchor);
    void anchorLink(size_t num);
    
    void enforceJointLimits(bool enforce);
    inline bool enforcingJointLimits() { return _enforceJointLimits; }
    
    verbosity verb;

protected:
    
    bool _enforceJointLimits;

    void _insertLink(Link* newLink);
    void _insertJoint(Joint* newJoint);
    
    void _recursiveDeleteConnection(Joint* deadJoint);
    void _deleteConnection(Joint* deadJoint);

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

};

} // namespace akin

#endif // ROBOT_H
