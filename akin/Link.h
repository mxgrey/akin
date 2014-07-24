#ifndef LINK_H
#define LINK_H

#include "akin/Body.h"
#include "akin/Robot.h"

namespace akin {


class Link : public Body
{
public:

    friend class Joint;
    friend class Robot;
    
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
    
    const Link& const_parentLink() const;

    /*!
     * \fn parentJoint()
     * \brief Returns the parent joint of this link
     * \return
     */
    Joint& parentJoint();
    const Joint& const_parentJoint() const;

    /*!
     * \fn childLink(size_t num)
     * \brief Returns the numth child of this link
     * \param num
     * \return
     */
    Link& childLink(size_t num);
    const Link& const_childLink(size_t num) const;

    /*!
     * \fn childJoint(size_t num)
     * \brief Returns the numth child joint of this link
     * \param num
     * \return
     */
    Joint& childJoint(size_t num);
    const Joint& const_childJoint(size_t num) const;

    size_t numChildJoints() const;
    inline size_t numChildLinks() const;

    Joint& upstreamJoint();
    const Joint& const_upstreamJoint() const;
    Link& upstreamLink();
    const Link& const_upstreamLink() const;

    Joint& downstreamJoint(size_t num);
    const Joint& const_downstreamJoint(size_t num) const;
    Link& downstreamLink(size_t num);
    const Link& const_downstreamLink(size_t num) const;
    
    size_t numDownstreamJoints() const;
    size_t numDownstreamLinks() const;
    
    Manipulator& manip(size_t manipNum);
    const Manipulator& const_manip(size_t manipNum) const;
    size_t numManips() const;

    bool belongsTo(const Robot& someRobot) const;
    Robot& robot();
    const Robot& const_robot() const;

    bool isDummy() const;

protected:
    
    Link(Robot* mRobot, Frame& referenceFrame, std::string linkName, size_t mID, bool root=false);
    
    size_t _id;
    
    void _addChildJoint(Joint* newChild);
    void _removeChildJoint(Joint* oldChild);
    void _setParentJoint(Joint* newParent);

    bool _isAnchor;
    bool _isRoot;
    bool _isDummy;

    Joint* _parentJoint;
    JointPtrArray _childJoints;

    Joint* _upstreamJoint;
    JointPtrArray _downstreamJoints;
    
    ManipPtrArray _manips;

    Robot* _myRobot;
    
    ~Link();
};

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::Link& someLink);

#endif // LINK_H
