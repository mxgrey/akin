#ifndef ROBOT_H
#define ROBOT_H

#include "Joint.h"
#include <map>

namespace akin {

class Link;

typedef std::map<std::string,size_t> StringMap;
typedef std::vector<Joint*> JointPtrArray;
typedef std::vector<Link*> LinkPtrArray;
typedef std::vector<std::string> StringArray;

enum {
    DOF_TPD     = (size_t)(-9),
    DOF_TIME    = (size_t)(-8),
    
    DOF_POS_X   = (size_t)(-7),
    DOF_POS_Y   = (size_t)(-6),
    DOF_POS_Z   = (size_t)(-5),
    DOF_ROT_X   = (size_t)(-4),
    DOF_ROT_Y   = (size_t)(-3),
    DOF_ROT_Z   = (size_t)(-2),
    
    DOF_INVALID = (size_t)(-1)
};

class Robot
{
public:

    friend class Joint;
    friend class Link;

    Robot(akin::Frame& referenceFrame = akin::Frame::World(), 
          verbosity::verbosity_level_t report_level = verbosity::LOG);

    ~Robot();

    void name(std::string newName);
    inline std::string name() const { return _name; }

    bool createRootLink(std::string rootLinkName);

    int createJointLinkPair(Link& parentLink,
                             const std::string& newLinkName,
                             const std::string& newJointName,
                             const Transform& baseTransform,
                             const Axis& jointAxis,
                             Joint::Type jointType,
                             double minJointValue,
                             double maxJointValue);
    
    int createJointLinkPair(size_t parentLinkID,
                            const std::string& newLinkName,
                            const std::string& newJointName,
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

    bool owns(const Link& someLink) const;
    bool owns(const Joint& someJoint) const;
    
    bool checkForLinkName(const std::string &name) const;
    bool checkForJointName(const std::string &name) const;

    inline Link& anchorLink() { return *_anchor; }
    void anchorLink(Link& newAnchor);
    void anchorLink(size_t linkNum);
    
    void enforceJointLimits(bool enforce);
    inline bool enforcingJointLimits() { return _enforceJointLimits; }

    std::string robotPackageDirectory;
    
    mutable verbosity verb;

protected:

    void _initializeRobot(akin::Frame& referenceFrame, verbosity::verbosity_level_t report_level);
    
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
    Geometry _dummyGeometry;

    JointPtrArray _joints;
    LinkPtrArray _links;
    
    JointPtrArray _root_dummy_joints;
    LinkPtrArray _root_dummy_links;

    StringMap _linkNameToIndex;
    StringMap _jointNameToIndex;
};

typedef std::vector<Robot*> RobotPtrArray;

} // namespace akin

#endif // ROBOT_H
