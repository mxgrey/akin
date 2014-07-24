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
    inline bool isAnchor() const { return _isAnchor; }

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
    inline bool isRoot() const { return _isRoot; }

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

    inline size_t numChildJoints() const { return _childJoints.size(); }
    inline size_t numChildLinks() const { return _childJoints.size(); }

    Joint& upstreamJoint();
    const Joint& const_upstreamJoint() const;
    Link& upstreamLink();
    const Link& const_upstreamLink() const;

    Joint& downstreamJoint(size_t num);
    const Joint& const_downstreamJoint(size_t num) const;
    Link& downstreamLink(size_t num);
    const Link& const_downstreamLink(size_t num) const;
    
    inline size_t numDownstreamJoints() const { return _downstreamJoints.size(); }
    inline size_t numDownstreamLinks() const { return _downstreamJoints.size(); }
    
    Manipulator& manip(size_t manipNum);
    const Manipulator& const_manip(size_t manipNum) const;
    size_t numManips() const;

    bool belongsTo(const Robot& someRobot) const;
    Robot& robot();
    const Robot& const_robot() const;

    inline bool isDummy() const { return _isDummy; }

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

inline std::ostream& operator<<(std::ostream& oStrStream, const akin::Link& someLink)
{
    oStrStream << "Link named '" << someLink.name() << "' ";
    if(someLink.isRoot())
    {
        oStrStream << "is the root link ";
    }
    if(someLink.isAnchor())
    {
        if(someLink.isRoot())
            oStrStream << "and ";
        oStrStream << "is the anchor link ";
    }
    
    if(!someLink.isRoot())
    {
        if(someLink.isAnchor())
            oStrStream << "with ";
        else
            oStrStream << " has ";
        oStrStream << "parent joint " << someLink.const_parentJoint().name();
    }
    
    oStrStream << "\n";
    
    if(someLink.numChildJoints() == 0)
    {
        oStrStream << "This link has no child joints";
    }
    else
    {
        oStrStream << "Child joints are: ";
        for(size_t i=0; i<someLink.numChildJoints(); ++i)
        {
            oStrStream << someLink.const_childJoint(i).name();
            if(i+1 < someLink.numChildJoints())
                oStrStream << ", ";
        }
    }
    
    oStrStream << "\n" << (akin::Frame&)someLink << std::endl;
    
    return oStrStream;
}

#endif // LINK_H
