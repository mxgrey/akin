#ifndef AKIN_JOINT_H
#define AKIN_JOINT_H

#include "Frame.h"
#include "Screw.h"

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
        CUSTOM,

        JOINT_TYPE_SIZE
    } Type;

    static std::string type_to_string(Type myJointType);

};

class ProtectedJointProperties
{
public:

    ProtectedJointProperties();
    ProtectedJointProperties(size_t jointID, const std::string& jointName,
                             const Transform& mBaseTransform, const Axis& mJointAxis,
                             akin::PublicJointProperties::Type mType,
                             double minimumValue, double maximumValue);

    Transform _baseTransform;
    Vec3 _axis;

    double _value;
    double _min;
    double _max;

    PublicJointProperties::Type _myType;

    size_t _id;
    std::string _name;

    bool _isDummy;
};

class Joint : public PublicJointProperties, protected ProtectedJointProperties
{
public:

    friend class Link;
    friend class Robot;

    /*!
     * \fn value(double newJointValue)
     * \brief Set this joint's value
     * \param newJointValue
     */
    bool value(double newJointValue);

    /*!
     * \fn value()
     * \brief Get this joint's value
     * \return
     */
    double value() const;
    
    Vec3 Jacobian_posOnly(const KinTranslation& point, const Frame& refFrame,
                      bool checkDependence=true) const;
    Vec3 Jacobian_rotOnly(const KinTranslation& point, const Frame& refFrame,
                      bool checkDependence=true) const;
    Screw Jacobian(const KinTranslation& point, const Frame& refFrame,
                   bool checkDependence=true) const;

    /*!
     * \fn min(double newMinValue)
     * \brief Set this joint's minimum value
     * \param newMinValue
     */
    bool min(double newMinValue);

    /*!
     * \fn min()
     * \brief Get this joint's minimum value
     * \return This joint's minimum value
     */
    double min() const;

    /*!
     * \fn max(double newMaxValue)
     * \brief Set this joint's maximum value
     * \param newMaxValue
     */
    bool max(double newMaxValue);

    /*!
     * \fn max()
     * \brief Get this joint's maximum value
     * \return This joint's maximum value
     */
    double max() const;
    
    bool withinLimits() const;
    bool withinLimits(double someValue) const;

    /*!
     * \fn jointType()
     * \brief Returns this joint's type
     * \return This joint's type
     */
    Type type() const;
    void type(Type newType);

    /*!
     * \fn jointAxis(Axis& jointAxis)
     * \brief Set the axis of this joint
     * \param newAxis
     */
    void axis(Vec3 newAxis);

    /*!
     * \fn jointAxis()
     * \brief Get the axis of this joint
     * \return This joint's axis
     */
    const Vec3& axis() const;

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
    std::string name() const;

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

protected:
    
    Joint(Robot* mRobot, size_t jointID=0, const std::string& jointName="joint",
          Link* mParentLink=NULL, Link* mChildLink=NULL,
          const Transform& mBaseTransform = Transform::Identity(),
          const Axis& mJointAxis = Axis(0, 0, 1), Joint::Type mType = Joint::REVOLUTE,
          double mininumValue=-INFINITY, double maximumValue=INFINITY);
    Joint& operator=(const Joint& otherJoint);
    
    void _computeRefTransform();
    void _computeTransformedJointAxis(Vec3& z_i, const Frame& refFrame) const;
    Vec3 _computePosJacobian(const Vec3& z_i, 
                             const KinTranslation& point, 
                             const Frame& refFrame) const;
    Vec3 _computeRotJacobian(const Vec3& z_i) const;
    
    void _changeParentLink(Link* newParent);

    Link* _parentLink;
    Link* _childLink;

    bool _reversed;
    Link* _upstreamLink;
    Link* _downstreamLink;

    void _reverse();

    Robot* _myRobot;
    
    virtual ~Joint();
};

typedef std::vector<Joint*> JointPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, akin::Joint::Type type);
std::ostream& operator<<(std::ostream& oStrStream, const akin::Joint& someJoint);

#endif // AKIN_JOINT_H
