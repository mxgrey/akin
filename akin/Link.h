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
    Robot& robot();

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

inline std::ostream& operator<<(std::ostream& oStrStream, akin::Link& someLink)
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
        oStrStream << "parent joint " << someLink.parentJoint().name();
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
            oStrStream << someLink.childJoint(i).name();
            if(i+1 < someLink.numChildJoints())
                oStrStream << ", ";
        }
    }
    
    oStrStream << "\n" << (akin::Frame&)someLink << std::endl;
    
    return oStrStream;
}

#endif // LINK_H
