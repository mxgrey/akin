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
    
    typedef enum {
        
        MANUAL = 0,
        URDF_FILE,
        URDF_STRING,
        
        CONSTRUCTION_TYPE_MAX
    } construction_t;

    /*!
     * \fn Robot() 
     * \brief Robot construction
     *
     * The constructor for the robot class can be handled in a number of ways
     * depending on which utilities you decided to install. (Currently only
     * URDF parsing utilities are available.)
     * 
     * The construction_info string may contain different kinds of information
     * depending on how you want your robot to be assembled. The method you want
     * is determined by the construction_t method parameter.
     * \li MANUAL - Only the root link will be given to the robot, and you must
     * specify all joint/link relationships. construction_info should contain
     * the desired name for the root link.
     * \li URDF_FILE - The robot will be constructed according to the
     * specifications of a URDF file. construction_info should contain the name
     * of the URDF file.
     * \li URDF_STRING - The robot will be constructed according to the
     * specifications of a URDF string. construction_info should contain the
     * entire text of the URDF string.
     *
     * rootReferenceFrame is the frame which the root of the robot is attached to.
     * If the anchor of the robot is changed, the new anchor will be attached to
     * this same reference frame.
     *
     * report_level represents a measure of how verbose you want the robot's
     * internal operations to be.
     */
    Robot(construction_t method = MANUAL,
          std::string construction_info = "root_link",
          Frame& rootReferenceFrame = Frame::World(),
          verbosity::verbosity_level_t report_level = verbosity::LOG);
    
    ~Robot();

    void name(std::string newName);
    inline std::string name() const { return _name; }

    bool createRootLink(std::string rootLinkName, Frame& referenceFrame = Frame::World());

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

    bool belongsTo(const Link& someLink) const;
    bool belongsTo(const Joint& someJoint) const;
    
    bool checkForLinkName(const std::string &name) const;
    bool checkForJointName(const std::string &name) const;

    inline Link& anchorLink() { return *_anchor; }
    void anchorLink(Link& newAnchor);
    void anchorLink(size_t linkNum);
    
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
