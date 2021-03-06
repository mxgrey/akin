#ifndef AKIN_JOINT_H
#define AKIN_JOINT_H

#include "DegreeOfFreedom.h"
#include "Body.h"

namespace akin {

class Link;
class Robot;

class PublicJointProperties
{
public:

    typedef enum {

        FIXED = 0,
        REVOLUTE,
        PRISMATIC,
        FLOATING,
        CUSTOM,

        JOINT_TYPE_SIZE
    } Type;

    static std::string type_to_string(Type myJointType);

};

class ProtectedJointProperties
{
public:

    ProtectedJointProperties(const std::string& jointName="",
                             akin::PublicJointProperties::Type mType = PublicJointProperties::FIXED,
                             const Transform& mBaseTransform=Eigen::Isometry3d::Identity(),
                             const Vec3& mJointAxis = Vec3::UnitZ());

    Transform _baseTransform;
    Vec3 _axis;
    Matrix6Xd _dofMatrix;
    Matrix6Xd _dofMatrixDerivative;
    mutable Eigen::VectorXd _values;
    mutable Eigen::VectorXd _velocities;
    mutable Eigen::VectorXd _accelerations;
    mutable Eigen::VectorXd _efforts;

    PublicJointProperties::Type _type;

    size_t _id;
    std::string _name;

    bool _isDummy;
};

class Joint : public PublicJointProperties, protected ProtectedJointProperties
{
public:

    friend class Link;
    friend class Robot;
    friend class DegreeOfFreedom;

    DegreeOfFreedom& dof(size_t num);
    const DegreeOfFreedom& dof(size_t num) const;
    size_t numDofs() const;

    const Eigen::VectorXd& values() const;
    bool values(const Eigen::VectorXd& newValues);
    const Eigen::VectorXd& velocities() const;
    bool velocities(const Eigen::VectorXd& newVelocities);
    const Eigen::VectorXd& accelerations() const;
    bool accelerations(const Eigen::VectorXd& newAccelerations);
    const Eigen::VectorXd& efforts() const;
    bool efforts(const Eigen::VectorXd& newEfforts);

    /*!
     * \fn jointType()
     * \brief Returns this joint's type
     * \return This joint's type
     */
    Type type() const;
    void type(Type newType, const DofProperties& properties = DofProperties());
    void type(Type newType, const std::vector<DofProperties>& properties);

    /*!
     * \fn jointAxis(Axis& jointAxis)
     * \brief Set the axis of this joint
     * \param newAxis
     */
    void axis(const Vec3& newAxis);

    /*!
     * \fn jointAxis()
     * \brief Get the axis of this joint
     * \return This joint's axis
     */
    const Vec3& axis() const;

    const Matrix6Xd& getDofMatrix() const;
    const Matrix6Xd& getDofMatrixDerivative() const;

    /*!
     * \fn baseTransform(const Transform& newBaseTf)
     * \brief Set what the relative transform is when the joint value is 0.
     * \param newBaseTf
     */
    void baseTransform(const Transform& newBaseTf);

    /*!
     * \fn baseTransform()
     * \brief Get what the relative transform is when the joint value is 0.
     * \return
     */
    const Transform& baseTransform() const;

    /*!
     * \fn id()
     * \brief Get this joint's index ID
     * \return
     */
    size_t id() const;

    /*!
     * \fn name()
     * \brief Get this joint's name
     * \return
     */
    const std::string& name() const;

    /*!
     * \fn name(const std::string& new_name)
     * \brief Allows you to change the name of this string
     * \param new_name
     * \return
     *
     * This function can be used to modify a joint's name. If
     * new_name is already the name of a different joint, this function
     * will return false, and this joint's name will not be changed.
     * If this function returns true, then the name of this joint
     * was successfully changed.
     */
    bool name(const std::string& new_name);

    Link& parentLink();
    const Link& parentLink() const;
    Link& childLink();
    const Link& childLink() const;
    
    Joint& parentJoint();
    const Joint& parentJoint() const;
    Joint& childJoint(size_t num);
    const Joint& childJoint(size_t num) const;
    size_t numChildJoints() const;
    
    Link& upstreamLink();
    const Link& upstreamLink() const;
    Link& downstreamLink();
    const Link& downstreamLink() const;
    
    Joint& upstreamJoint();
    const Joint& upstreamJoint() const;

    Joint& downstreamJoint(size_t num);
    const Joint& downstreamJoint(size_t num) const;
    size_t numDownstreamJoints() const;

    bool belongsTo(const Robot& someRobot) const;
    Robot& robot();
    const Robot& robot() const;
    
    bool isDummy() const;
    
    verbosity& verb;
    
    bool isReversed() const;

    bool needsPosUpdate() const;
    void notifyPosUpdate();
    bool needsVelUpdate() const;
    void notifyVelUpdate();
    bool needsAccUpdate() const;
    void notifyAccUpdate();
    bool needsDynUpdate() const;
    void notifyDynUpdate();


protected:
    
//    Joint(Robot* mRobot, size_t jointID=0, const std::string& jointName="joint",
//          Link* mParentLink=nullptr, Link* mChildLink=nullptr,
//          const Transform& mBaseTransform = Transform::Identity(),
//          const Vec3& mJointAxis = Axis(0, 0, 1), Joint::Type mType = Joint::REVOLUTE,
//          double mininumValue=-INFINITY, double maximumValue=INFINITY,
//          double maxSpeed=INFINITY, double maxAcceleration=INFINITY, double maxTorque=INFINITY);

    Joint(Robot* mRobot, size_t jointID,
          Link* mParentLink, Link* mChildLink,
          const ProtectedJointProperties& joint_properties,
          const std::vector<DofProperties>& dof_properties);

    void _integrate(integration_method_t method, double dt);
    void _explicit_euler_integration(double dt);

    // TODO: Implement this for multi-dof joints
//    Joint(Robot* mRobot, size_t jointID, const ProtectedJointProperties& joint_properties,
//          const DofPropertyArray& dof_properties);

    void _createDofs(const std::vector<DofProperties>& dof_properties);

    Joint& operator=(const Joint& otherJoint);

    mutable bool _needsPosUpdate;
    mutable bool _needsVelUpdate;
    mutable bool _needsAccUpdate;
    mutable bool _needsDynUpdate;
    
    void _computeRefTransform() const;
    void _computeRelVelocity() const;
    void _computeRelAcceleration() const;

//    void _computeTransformedJointAxis(Vec3& z_i, const Frame& refFrame) const;
//    Vec3 _computePosJacobian(const Vec3& z_i,
//                             const KinTranslation& point,
//                             const Frame& refFrame) const;
//    Vec3 _computeRotJacobian(const Vec3& z_i) const;
    
    void _changeParentLink(Link* newParent);

    Link* _parentLink;
    Link* _childLink;

    bool _reversed;
    Link* _upstreamLink;
    Link* _downstreamLink;

    DofPtrArray _dofs;

    void _reverse();

    Robot* _robot;
    
    virtual ~Joint();
};

typedef std::vector<Joint*> JointPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, akin::Joint::Type type);
std::ostream& operator<<(std::ostream& oStrStream, const akin::Joint& someJoint);

#endif // AKIN_JOINT_H
