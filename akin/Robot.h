#ifndef ROBOT_H
#define ROBOT_H

#include "Joint.h"
#include "Manipulator.h"
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
    friend class Manipulator;
    
    class Crawler
    {
    public:
        
        typedef enum {
            
            INACTIVE = 0,
            DOWNSTREAM,
            UPSTREAM,
            PATH
            
        } policy;
        
        static std::vector<const Link*> getPath(const Link& startLink, const Link& endLink);
        static std::vector<size_t> getIdPath(const Link& startLink, const Link& endLink);
        
        Crawler();
        Crawler(const Link& startLink, policy p=DOWNSTREAM);
        Crawler(const Link& startLink, const Link& endLink);
        
        void reset(const Link& startLink, policy p=DOWNSTREAM);
        void reset(const Link& startLink, const Link& endLink);
        
        const Link* nextLink();
        Link* nonconst_nextLink();
        
        bool stopAtRoot;
        
    protected:
        
        class Recorder
        {
        public:
            
            Recorder();
            Recorder(const Link* link_, size_t count_);
            
            const Link* link;
            int count;
        };
        
        void _init();
        policy _p;
        const Link* _first;
        bool _finished;
        std::vector<Recorder> _recorder;
        std::vector<const Link*> _path;
        std::vector<const Link*> _temp;
        size_t _pathLocation;
    };

    Robot(akin::Frame& referenceFrame = akin::Frame::World(), 
          verbosity::verbosity_level_t report_level = verbosity::LOG);

    ~Robot();
    
    Frame& refFrame();
    const Frame& const_refFrame() const;
    bool changeRefFrame(Frame& newRefFrame);
    
    const KinTranslation& com() const;
    const double& mass() const;

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
    size_t getJointIndex(const std::string& jointName) const;
    const Joint& const_joint(size_t jointNum) const;
    const Joint& const_joint(const std::string& jointName) const;
    inline size_t numJoints() { return _joints.size(); }

    Link& link(size_t linkNum);
    Link& link(const std::string& linkName);
    size_t getLinkIndex(const std::string& linkName) const;
    const Link& const_link(size_t linkNum) const;
    const Link& const_link(const std::string& linkName) const;
    inline size_t numLinks() { return _links.size(); }
    
    int addManipulator(Frame& attachment, const std::string& name, 
                        Transform relativeTf = Transform::Identity());
    bool removeManipulator(Manipulator& m);
    Manipulator& manip(size_t manipNum);
    Manipulator& manip(const std::string& manipName);
    size_t getManipIndex(const std::string& manipName) const;
    const Manipulator& const_manip(size_t manipNum) const;
    const Manipulator& const_manip(const std::string& manipName) const;
    inline size_t numManips() { return _manips.size(); }

    bool owns(const Link& someLink) const;
    bool owns(const Joint& someJoint) const;
    bool owns(const Manipulator& someManip) const;
    
    bool checkForLinkName(const std::string& name) const;
    bool checkForJointName(const std::string& name) const;
    bool checkForManipName(const std::string& name) const;

    inline Link& anchorLink() { return *_anchor; }
    void anchorLink(Link&);
    void anchorLink(size_t);
    
    void enforceJointLimits(bool enforce);
    inline bool enforcingJointLimits() { return _enforceJointLimits; }

    std::string robotPackageDirectory;
    
    mutable verbosity verb;

protected:

    void _initializeRobot(akin::Frame& referenceFrame, verbosity::verbosity_level_t report_level);
    
    bool _enforceJointLimits;

    void _insertLink(Link* newLink);
    void _insertJoint(Joint* newJoint);
    void _insertManip(Manipulator* newManip);
    
    void _recursiveDeleteConnection(Joint* deadJoint);
    void _deleteConnection(Joint* deadJoint);

    mutable KinTranslation _com;
    mutable double _mass;
    
    std::string _name;

    Link* _anchor;
    Link* _root;

    Link* _dummyLink;
    Joint* _dummyJoint;
    Manipulator* _dummyManip;
    Geometry _dummyGeometry;

    JointPtrArray _joints;
    LinkPtrArray _links;
    ManipPtrArray _manips;
    
    JointPtrArray _root_dummy_joints;
    LinkPtrArray _root_dummy_links;

    StringMap _linkNameToIndex;
    StringMap _jointNameToIndex;
    StringMap _manipNameToIndex;
};

typedef std::vector<Robot*> RobotPtrArray;

} // namespace akin

#endif // ROBOT_H
