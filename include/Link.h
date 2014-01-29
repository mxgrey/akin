#ifndef LINK_H
#define LINK_H

#include "Robot.h"

namespace akin {


class Link : public Frame
{
public:

    friend class Joint;
    friend class Robot;

    /*!
     * \fn isAnchor()
     * \brief Returns true iff this link is the robot's anchor to the outside world
     * \return
     *
     * A link can be made into an anchor by calling setAsAnchor(). The anchor of the
     * robot represents the frame which does not move when joint angles are changed.
     * Any link can be set as the robot's anchor.
     */
    inline bool isAnchor() { return _isAnchor; }

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
    inline bool isRoot() { return _isRoot; }

    /*!
     * \fn parentLink()
     * \brief Returns the parent of this link
     * \return
     */
    Link& parentLink();

    /*!
     * \fn parentJoint()
     * \brief Returns the parent joint of this link
     * \return
     */
    Joint& parentJoint();

    /*!
     * \fn childLink(size_t num)
     * \brief Returns the numth child of this link
     * \param num
     * \return
     */
    Link& childLink(size_t num);

    /*!
     * \fn childJoint(size_t num)
     * \brief Returns the numth child joint of this link
     * \param num
     * \return
     */
    Joint& childJoint(size_t num);

    inline size_t numChildJoints() { return _childJoints.size(); }
    inline size_t numChildLinks() { return _childJoints.size(); }

    Joint& upstreamJoint();
    Link& upstreamLink();

    Joint& downstreamJoint(size_t num);
    Link& downstreamLink(size_t num);
    
    inline size_t numDownstreamJoints() { return _downstreamJoints.size(); }
    inline size_t numDownstreamLinks() { return _downstreamJoints.size(); }

    bool belongsTo(const Robot& someRobot) const;

    inline bool isDummy() { return _isDummy; }

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

    Robot* _myRobot;
    
    ~Link();

};

} // namespace akin


#endif // LINK_H
