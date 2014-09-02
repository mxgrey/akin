#ifndef AKIN_ROBOT_H
#define AKIN_ROBOT_H

#include "Joint.h"
#include "Link.h"
#include "Manipulator.h"
#include "Tracker.h"
#include <map>

namespace akin {

class RobotConstraintBase;
class CenterOfMassConstraintBase;


typedef std::map<std::string,size_t> StringMap;
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
    
    class Explorer
    {
    public:
        
        typedef enum {
            
            INACTIVE = 0,
            DOWNSTREAM,
            UPSTREAM,
            PATH
            
        } policy;
        
        static std::vector<const Link*> getPath(const Link& startLink, const Link& endLink);
        static std::vector<const Joint*> getPath(const Joint& startJoint, const Joint& endJoint);
        static std::vector<size_t> getIdPath(const Link& startLink, const Link& endLink);
        static std::vector<size_t> getIdPath(const Joint& startJoint, const Joint& endJoint);
        
        static std::vector<const Link*> getLinks(const Link& startLink, policy p=DOWNSTREAM);
        static std::vector<const Joint*> getJoints(const Joint& startJoint, policy p=DOWNSTREAM);
        static std::vector<size_t> getLinkIds(const Link& startLink, policy p=DOWNSTREAM);
        static std::vector<size_t> getJointIds(const Joint& startJoint, policy p=DOWNSTREAM);
        
        Explorer();
        Explorer(const Link& startLink, policy p=DOWNSTREAM);
        Explorer(const Link& startLink, const Link& endLink);
        Explorer(const Joint& startJoint, policy p=DOWNSTREAM);
        Explorer(const Joint& startJoint, const Joint& endJoint);
        
        void reset(const Link& startLink, policy p=DOWNSTREAM);
        void reset(const Link& startLink, const Link& endLink);
        void reset(const Joint& startJoint, policy p=DOWNSTREAM);
        void reset(const Joint& startJoint, const Joint& endJoint);
        
        const Link* currentLink() const;
        Link* nonconst_currentLink() const;
        
        const Joint* currentJoint() const;
        Joint* nonconst_currentJoint() const;
        
        const Link* nextLink();
        Link* nonconst_nextLink();
        
        const Joint* nextJoint();
        Joint* nonconst_nextJoint();
        
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

    double zeroValue;

    virtual ~Robot();

    Eigen::VectorXd getConfig(const std::vector<size_t>& joints) const;
    bool setConfig(const std::vector<size_t>& joints, const Eigen::VectorXd& values);

    const std::vector<Eigen::Vector2d>& getSupportPolygon();
    const Eigen::Vector2d& getSupportCenter();
    std::vector<Eigen::Vector2d> computeSupportPolgon() const;
    void forceSupportUpdate();
    
    Frame& refFrame();
    const Frame& const_refFrame() const;
    bool changeRefFrame(Frame& newRefFrame);
    
    Frame& frame();
    const Frame& const_frame() const;
    
    const KinTranslation& com() const;
    Translation com(const Link& startLink, const Frame& referenceFrame = Frame::World(), 
                    Explorer::policy p = Explorer::DOWNSTREAM) const;
    const double& mass() const;
    double mass(const Link& startLink, Explorer::policy p = Explorer::DOWNSTREAM) const;

    CenterOfMassConstraintBase* balance();
    const CenterOfMassConstraintBase* const_balance() const;
    void setBalanceConstraint(CenterOfMassConstraintBase* newConstraint, bool ownConstraint=true);
    void setDefaultBalanceConstraint();

    RobotConstraintBase* task();
    const RobotConstraintBase* const_task() const;
    void setTaskConstraint(RobotConstraintBase* newConstraint, bool ownConstraint=true);
    void setDefaultTaskConstraint();

    void setDefaultRobotConstraints();

    RobotSolverX& solver();
    bool solve();

    void name(std::string newName);
    const std::string& name() const;

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
    size_t numJoints() const;

    Link& link(size_t linkNum);
    Link& link(const std::string& linkName);
    size_t getLinkIndex(const std::string& linkName) const;
    const Link& const_link(size_t linkNum) const;
    const Link& const_link(const std::string& linkName) const;
    size_t numLinks() const;
    
    int addManipulator(Frame& attachment, const std::string& name, 
                        Transform relativeTf = Transform::Identity());
    bool removeManipulator(Manipulator& m);
    Manipulator& manip(size_t manipNum);
    Manipulator& manip(const std::string& manipName);
    size_t getManipIndex(const std::string& manipName) const;
    const Manipulator& const_manip(size_t manipNum) const;
    const Manipulator& const_manip(const std::string& manipName) const;
    size_t numManips() const;

    bool owns(const Link& someLink) const;
    bool owns(const Joint& someJoint) const;
    bool owns(const Manipulator& someManip) const;
    
    bool checkForLinkName(const std::string& name) const;
    bool checkForJointName(const std::string& name) const;
    bool checkForManipName(const std::string& name) const;

    Link& anchorLink();
    const Link& const_anchorLink() const;
    void anchorLink(Link&);
    void anchorLink(size_t);

    Link& rootLink();
    const Link& const_rootLink() const;
    
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
    mutable Explorer _crawler;

    std::vector<Eigen::Vector2d> _supportPolgyon;
    Eigen::Vector2d _supportCenter;
    mutable std::vector<Eigen::Vector2d> _supportPoints;

    class ManipMemory {
    public:
        inline ManipMemory(bool mSupport, Transform mTf) :
            support(mSupport), tf(mTf) { }
        bool support;
        Transform tf;
    };

    std::vector<ManipMemory> _supportMemory;
    TrackerPtrArray _supportTrackers;
    bool _needSupportUpdate() const;
    
    std::string _name;

    Link* _anchor;
    Link* _root;

    Link* _dummyLink;
    Joint* _dummyJoint;
    Manipulator* _dummyManip;
    Geometry _dummyGeometry;

    CenterOfMassConstraintBase* _balance;
    bool _ownsBalance;

    RobotConstraintBase* _task;
    bool _ownsTask;
    Eigen::VectorXd _taskConfig;

    RobotSolverX* _solver;

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

#endif // AKIN_ROBOT_H
