#ifndef AKIN_LINK_H
#define AKIN_LINK_H

#include "Joint.h"
#include "Manipulator.h"

namespace akin {

class Robot;

class Link : public Body
{
public:

    friend class Joint;
    friend class DegreeOfFreedom;
    friend class Robot;

    const Transform& respectToRef() const;
    void respectToRef(const Transform&);

    const Velocity& relativeLinearVelocity() const;
    void relativeLinearVelocity(const Velocity&);
    const Velocity& relativeAngularVelocity() const;
    void relativeAngularVelocity(const Velocity&);

    const Acceleration& relativeLinearAcceleration() const;
    void relativeLinearAcceleration(const Acceleration&);
    const Acceleration& relativeAngularAcceleration() const;
    void relativeAngularAcceleration(const Acceleration&) const;
    
    const std::string& name() const;
    bool name(const std::string& newName);
    
    size_t id() const;

    /*!
     * \fn isAnchor()
     * \brief Returns true iff this link is the robot's anchor to the outside world
     * \return
     *
     * A link can be made into an anchor by calling setAsAnchor(). The anchor of the
     * robot represents the frame which does not move when joint angles are changed.
     * Any link can be set as the robot's anchor.
     */
    bool isAnchor() const;

    /*!
     * \fn setAsAnchor()
     * \brief Sets this link as the anchor point of the robot
     */
    void setAsAnchor();

    /*!
     * \fn isRoot()
     * \brief Returns true iff this link is the root link of the robot
     * \return
     */
    bool isRoot() const;

    /*!
     * \fn parentLink()
     * \brief Returns the parent of this link
     * \return
     */
    Link& parentLink();
    const Link& parentLink() const;

    /*!
     * \fn parentJoint()
     * \brief Returns the parent joint of this link
     * \return
     */
    Joint& parentJoint();
    const Joint& parentJoint() const;

    /*!
     * \fn childLink(size_t num)
     * \brief Returns the numth child of this link
     * \param num
     * \return
     */
    Link& childLink(size_t num);
    const Link& childLink(size_t num) const;

    /*!
     * \fn childJoint(size_t num)
     * \brief Returns the numth child joint of this link
     * \param num
     * \return
     */
    Joint& childJoint(size_t num);
    const Joint& childJoint(size_t num) const;

    size_t numChildJoints() const;
    size_t numChildLinks() const;

    Joint& upstreamJoint();
    const Joint& upstreamJoint() const;
    Link& upstreamLink();
    const Link& upstreamLink() const;

    Joint& downstreamJoint(size_t num);
    const Joint& downstreamJoint(size_t num) const;
    Link& downstreamLink(size_t num);
    const Link& downstreamLink(size_t num) const;
    
    size_t numDownstreamJoints() const;
    size_t numDownstreamLinks() const;
    
    Manipulator& manip(size_t manipNum);
    const Manipulator& manip(size_t manipNum) const;
    size_t numManips() const;

    bool belongsTo(const Robot& someRobot) const;
    Robot& robot();
    const Robot& robot() const;

    bool isDummy() const;
    
    void setDynamicsMode(dynamics_mode_t mode);

    void integrate(integration_method_t method, double dt);

    bool notifyDynUpdate();

protected:
    
    Link(Robot* mRobot, Frame& referenceFrame, std::string linkName, size_t mID, bool root=false);

    void _explicit_euler_integration(double dt);

    void _computeABA_pass2() const;
//    void _computeABA_pass3() const;

    size_t _id;
    
    void _addChildJoint(Joint* newChild);
    void _removeChildJoint(Joint* oldChild);
    void _setParentJoint(Joint* newParent);

    bool _isRoot;
    bool _isDummy;

    Joint* _parentJoint;
    JointPtrArray _childJoints;

    Joint* _upstreamJoint;
    JointPtrArray _downstreamJoints;
    
    ManipPtrArray _manips;

    Robot* _myRobot;
    
    virtual ~Link();
};

typedef std::vector<Link*> LinkPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::Link& someLink);

#endif // AKIN_LINK_H
