#ifndef AKIN_ROBOT_H
#define AKIN_ROBOT_H

#include "Joint.h"
#include "Link.h"
#include "Manipulator.h"
#include "Tracker.h"
#include <map>

namespace akin {

class RobotConstraintBase;
class BalanceConstraintBase;


typedef std::map<std::string,size_t> StringMap;
typedef std::vector<std::string> StringArray;

enum {

    DOF_POS_X   = 0,
    DOF_POS_Y   = 1,
    DOF_POS_Z   = 2,
    DOF_ROT_X   = 3,
    DOF_ROT_Y   = 4,
    DOF_ROT_Z   = 5,

    DOF_VTD     = (size_t)(-4),
    DOF_TIME    = (size_t)(-3),

    BASE_INDEX  = (size_t)(-2),
    INVALID_INDEX = (size_t)(-1)
};

class Robot : public InertiaBase
{
public:
    
    forward_dynamics_method_t forward_method;
    inverse_dynamics_method_t inverse_method;

    friend class Link;
    friend class Joint;
    friend class DegreeOfFreedom;
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

        static std::vector<const DegreeOfFreedom*> getDofs(const DegreeOfFreedom& startDof,
                                                           policy p=DOWNSTREAM);
        static std::vector<const DegreeOfFreedom*> getDofs(const DegreeOfFreedom& startDof,
                                                           const DegreeOfFreedom& endDof);
        static std::vector<const DegreeOfFreedom*> getDofs(const Joint& startJoint,
                                                           policy p=DOWNSTREAM);
        static std::vector<const DegreeOfFreedom*> getDofs(const Joint& startJoint,
                                                           const Joint& endJoint);

        static std::vector<size_t> getDofIds(const DegreeOfFreedom& startDof, policy p=DOWNSTREAM);
        static std::vector<size_t> getDofIds(const DegreeOfFreedom& startDof,
                                             const DegreeOfFreedom& endDof);
        static std::vector<size_t> getDofIds(const Joint& startJoint, policy p=DOWNSTREAM);
        static std::vector<size_t> getDofIds(const Joint& startJoint, const Joint& endJoint);
        
        Explorer();
        Explorer(const Link& startLink, policy p=DOWNSTREAM);
        Explorer(const Link& startLink, const Link& endLink);
        Explorer(const Joint& startJoint, policy p=DOWNSTREAM);
        Explorer(const Joint& startJoint, const Joint& endJoint);
        Explorer(const DegreeOfFreedom& startDof, policy p=DOWNSTREAM);
        Explorer(const DegreeOfFreedom& startDof, const DegreeOfFreedom& endDof);
        
        void reset(const Link& startLink, policy p=DOWNSTREAM);
        void reset(const Link& startLink, const Link& endLink);
        void reset(const Joint& startJoint, policy p=DOWNSTREAM);
        void reset(const Joint& startJoint, const Joint& endJoint);
        void reset(const DegreeOfFreedom& startDof, policy p=DOWNSTREAM);
        void reset(const DegreeOfFreedom& startDof, const DegreeOfFreedom& endDof);
        
        const Link* currentLink() const;
        Link* nonconst_currentLink() const;
        
        const Joint* currentJoint() const;
        Joint* nonconst_currentJoint() const;

        const DegreeOfFreedom* currentDof() const;
        DegreeOfFreedom* nonconst_currentDof() const;

        const Link* nextLink();
        Link* nonconst_nextLink();

        const Joint* nextJoint();
        Joint* nonconst_nextJoint();

        const DegreeOfFreedom* nextDof();
        DegreeOfFreedom* nonconst_nextDof();
        
    protected:

        template<typename T1, typename T2>
        static std::vector<const DegreeOfFreedom*> _getDofs(T1 start, T2 terminate) {
            Explorer crawl(start, terminate);

            std::vector<const DegreeOfFreedom*> result;
            const DegreeOfFreedom* nextDof;
            while( (nextDof = crawl.nextDof()) )
                result.push_back(nextDof);

            return result;
        }

        template<typename T1, typename T2>
        static std::vector<size_t> _getDofIds(T1 start, T2 terminate) {
            Explorer crawl(start, terminate);

            std::vector<size_t> result;
            const DegreeOfFreedom* nextDof;
            while( (nextDof = crawl.nextDof()) )
                result.push_back(nextDof->id());

            return result;
        }
        
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

        int _dof_counter;
    };

    Robot(akin::Frame& referenceFrame = akin::Frame::World(), 
          verbosity::verbosity_level_t report_level = verbosity::LOG);

    double zeroValue;

    virtual ~Robot();

    Eigen::VectorXd getConfig(const std::vector<size_t>& dofs) const;
    bool setConfig(const std::vector<size_t>& dofs, const Eigen::VectorXd& values);

    const std::vector<Eigen::Vector2d>& getSupportPolygon();
    const Eigen::Vector2d& getSupportCenter();
    std::vector<Eigen::Vector2d> computeSupportPolgon() const;
    void forceSupportUpdate();
    
    Frame& refFrame();
    const Frame& refFrame() const;
    bool changeRefFrame(Frame& newRefFrame);
    
    Frame& frame();
    const Frame& frame() const;

    // Constraint management functions
    BalanceConstraintBase* balance();
    const BalanceConstraintBase* balance() const;
    void setBalanceConstraint(BalanceConstraintBase* newConstraint, bool ownConstraint=true);
    void setDefaultBalanceConstraint();

    RobotConstraintBase* task();
    const RobotConstraintBase* task() const;
    void setTaskConstraint(RobotConstraintBase* newConstraint, bool ownConstraint=true);
    void setDefaultTaskConstraint();

    void setDefaultRobotConstraints();

    RobotSolverX& solver();
    bool solve();

    // Inertia functions
    const KinTranslation& com() const;
    Translation com(const Link& startLink, const Frame& referenceFrame = Frame::World(),
                    Explorer::policy p = Explorer::DOWNSTREAM) const;
    const double& mass() const;
    double mass(const Link& startLink, Explorer::policy p = Explorer::DOWNSTREAM) const;

    Translation getCom(const Frame& withRespectToFrame = Frame::World()) const;
    double getMass() const;
    Eigen::Matrix3d getInertiaTensor(const Frame &withRespectToFrame) const;

    FreeVector getForces(const Frame &withRepsectToFrame) const;
    FreeVector getMoments(const Frame &withRespectToFrame) const;
    Screw getWrench(const Frame &withRespectToFrame) const;


    void name(std::string newName);
    const std::string& name() const;

    int createJointLinkPair(Link& parentLink, const std::string& newLinkName,
                            const ProtectedJointProperties& joint_properties,
                            const DofProperties& dof_properties);

    int createJointLinkPair(size_t parentLinkID, const std::string& newLinkName,
                            const ProtectedJointProperties& joint_properties,
                            const DofProperties& dof_properties);
    
    void removeConnection(size_t jointNum, bool fillInGap=false);
    void removeConnection(std::string& jointName, bool fillInGap=false);

    Joint& joint(size_t jointNum);
    Joint& joint(const std::string& jointName);
    size_t getJointIndex(const std::string& jointName) const;
    const Joint& joint(size_t jointNum) const;
    const Joint& joint(const std::string& jointName) const;
    size_t numJoints() const;

    DegreeOfFreedom& dof(size_t dofNum);
    DegreeOfFreedom& dof(const std::string& dofName);
    size_t getDofIndex(const std::string& dofName) const;
    const DegreeOfFreedom& dof(size_t dofNum) const;
    const DegreeOfFreedom& dof(const std::string &dofName) const;
    size_t numDofs() const;

    Link& link(size_t linkNum);
    Link& link(const std::string& linkName);
    size_t getLinkIndex(const std::string& linkName) const;
    const Link& link(size_t linkNum) const;
    const Link& link(const std::string& linkName) const;
    size_t numLinks() const;
    
    int addManipulator(Frame& attachment, const std::string& name, 
                        Transform relativeTf = Transform::Identity());
    bool removeManipulator(Manipulator& m);
    Manipulator& manip(size_t manipNum);
    Manipulator& manip(const std::string& manipName);
    size_t getManipIndex(const std::string& manipName) const;
    const Manipulator& manip(size_t manipNum) const;
    const Manipulator& manip(const std::string& manipName) const;
    size_t numManips() const;

    bool owns(const Link& someLink) const;
    bool owns(const Joint& someJoint) const;
    bool owns(const Manipulator& someManip) const;
    
    bool checkForLinkName(const std::string& name) const;
    bool checkForJointName(const std::string& name) const;
    bool checkForDofName(const std::string& name) const;
    bool checkForManipName(const std::string& name) const;

    Link& anchorLink();
    const Link& anchorLink() const;
    void anchorLink(Link&);
    void anchorLink(size_t);

    Link& rootLink();
    const Link& rootLink() const;
    
    void enforceJointLimits(bool enforce);
    inline bool enforceJointLimits() { return _enforceJointLimits; }

    std::string robotPackageDirectory;
    
    mutable verbosity verb;

    void setDynamicsMode(dynamics_mode_t mode);
    dynamics_mode_t getDynamicsMode() const;

    void integrate(integration_method_t method, double dt);

    bool notifyDynUpdate();
    bool needsDynUpdate() const;

    const Matrix6d& _ABA_Ia() const;
    const Vector6d& _ABA_pa() const;
    const Vector6d& _ABA_c() const;
    const Vector6d& _ABA_a() const;
    const Vector6d& _ABA_arel() const;

    const Matrix6Xd& _ABA_h() const;
    const Eigen::VectorXd& _ABA_u() const;
    const Eigen::MatrixXd& _ABA_d() const;
    const Eigen::VectorXd& _ABA_qdd() const;

protected:

    void _initializeRobot(akin::Frame& referenceFrame, verbosity::verbosity_level_t report_level);
    bool _createRootLink(const std::string& rootLinkName, akin::Frame& referenceFrame);

    void _explicit_euler_integration(double dt);
    
    bool _enforceJointLimits;

    void _insertLink(Link* newLink);
    void _insertJoint(Joint* newJoint);
    void _insertDof(DegreeOfFreedom* newDof);
    void _insertManip(Manipulator* newManip);
    
    void _recursiveDeleteConnection(Joint* deadJoint);
    void _deleteConnection(Joint* deadJoint);

    void _computeABA_pass2() const;
    void _computeABA_pass3() const;

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
    Joint* _pseudo_joint;
    Link* _pseudo_link;

    Link* _dummyLink;
    Joint* _dummyJoint;
    DegreeOfFreedom* _dummyDof;
    Manipulator* _dummyManip;
    Geometry _dummyGeometry;

    BalanceConstraintBase* _balance;
    bool _ownsBalance;

    RobotConstraintBase* _task;
    bool _ownsTask;
    Eigen::VectorXd _taskConfig;

    RobotSolverX* _solver;

    LinkPtrArray _links;
    JointPtrArray _joints;
    DofPtrArray _dofs;
    ManipPtrArray _manips;

    StringMap _linkNameToIndex;
    StringMap _jointNameToIndex;
    StringMap _dofNameToIndex;
    StringMap _manipNameToIndex;
};

typedef std::vector<Robot*> RobotPtrArray;

} // namespace akin

#endif // AKIN_ROBOT_H
