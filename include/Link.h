#ifndef LINK_H
#define LINK_H

#include "Frame.h"

namespace akin {

class Joint;

class Link : public Frame
{
public:

    friend class Joint;

    /*!
     * \fn isAnchor()
     * \brief Returns true iff this link is the robot's anchor to the outside world
     * \return
     *
     * A link can be made into an anchor by calling setAsAnchor()
     */
    bool isAnchor();

    /*!
     * \fn setAsAnchor()
     * \brief Sets this link as the anchor point of the robot
     */
    void setAsAnchor();



protected:

    bool _isAnchor;

    Joint* _parentJoint;
    std::vector<Joint*> _childJoints;

};

} // namespace akin


#endif // LINK_H
