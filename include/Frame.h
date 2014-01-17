#ifndef FRAME_H
#define FRAME_H

#include "Transform.h"

namespace akin {

/*!
 * \class Frame
 * \brief Foundation of akin's kinematic trees
 * 
 * The Frame class is the foundation of akin, because it is the cornerstone of
 * the kinematic tree structures. The frame class keeps track of its relationship
 * to its parent frame and child frames, as well as its own pose in the world
 * frame. This class keeps track of when updates are needed and provides
 * convenience functions for finding relationships between arbitrary frames.
 */

class Frame : public KinObject
{
    friend class KinObject;

public:

    KinCustomMacro( Frame )

    /*!
      * \fn Frame(Frame& referenceFrame, std::string frameName, verbosity::verbosity_level_t report_level)
      * \brief Constructor for frames
      * 
      */
    Frame(Frame& referenceFrame,
          std::string frameName,
          verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    Frame(const Transform& relativeTf,
          Frame& referenceFrame = World(),
          std::string frameName = "arbitrary_frame",
          verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    ~Frame();

    /*!
      * \fn World()
      * \brief Returns the World Frame
      * 
      * The World Frame is a key concept in akin, because it allows arbitrary
      * kinematic trees to understand their relationships with each other. The
      * World Frame is static in the sense that none of its features may be
      * altered by the user at any time. It simply exists, and always has a
      * world transform of identity. It is important to note that unlike all
      * other frames, the World Frame does NOT keep track of its child frames.
      */
    static Frame& World();
    
    /*!
      * \fn childFrame()
      * \brief Returns this frame's child frame, corresponding to childFrameNum
      *
      * Any given frame can have any number of children. A child frame is defined as
      * any frame whose transformation is maintained directly with respect to the
      * current frame. Therefore this does not include grandchildren or any deeper
      * lineage.
      *
      * If a non-existent child is requested, this function returns the World Frame.
      * If verbosity is set to brief or higher, a warning will be printed out. If
      * assertiveness is set to ASSERT_CASUAL, the program will abort.
      */
    Frame& childFrame(size_t childFrameNum);
    
    /*!
      * \fn numChildFrames()
      * \brief Returns the frame's current number of child frames
      */
    size_t numChildFrames() const;

    /*!
      * \fn childObject()
      * \brief Returns this frame's child KinObject, corresponding to childObjNum
      *
      * Besides child frames, a Frame can also have any number of KinObject children.
      * These may include transformations, translations, or any other arbitrary kind
      * of data which expresses itself relative to this frame. This function returns
      * a KinObject reference, which means it will grant access to the meta-information
      * of the object. This will allow you to print out information about this frame's
      * children which might be useful for investigation or debugging. However, this
      * does not allow you to directly manipulate the object's actual data.
      *
      * Note that the child frames are also included as child KinObjects.
      */
    KinObject &childObject(size_t childObjNum);
    
    /*!
      * \fn numChildObjects()
      * \brief Returns the frame's current number of child KinObjects
      */
    size_t numChildObjects() const;

    virtual bool changeRefFrame(Frame& newRefFrame);
    
    /*!
      * \fn isWorld()
      * \brief Returns true if the frame is the World Frame
      *
      * This can be useful for terminating a search through a kinematic tree.
      * Every search is guaranteed to terminate at the World Frame if it is
      * strictly moving toward the parents.
      *
      * This can also be useful for validity checks, because the World Frame
      * is returned whenever an invalid frame is requested.
      */
    bool isWorld() const;
    
    /*!
      * \fn notifyUpdate()
      * \brief Notify this frame and its children that an update is necessary
      *
      * This function is used to efficiently handle kinematic updates. Kinematic
      * computations are only performed when needed, and they are never performed
      * more often than necessary. The update notification system is what ensures
      * these things.
      * 
      * As a user, you should never have to explicitly call this function, because
      * any other function which warrants an update should already be calling it.
      * If you find that this is not the case, please report it as an issue on Github.
      */
    virtual void notifyUpdate();
    
    /*!
      * \fn respectToRef(const Transform& newTf)
      * \brief Updates the relative transform of this frame
      * 
      * This function is used to modify this frame's transformation with respect
      * to its parent.
      */
    virtual void respectToRef(const Transform& newTf);
    
    /*!
      * \fn respectToRef()
      * \brief Returns this frame's transformation with respect to its parent
      * 
      * This returns a reference, but it is const, so you cannot use this function
      * to modify this frame's transform with respect to its parent. Instead, you
      * should use the function respectToRef(const Transform& newTf).
      *
      * Note that calling this function automatically performs all necessary
      * updates to the kinematic tree.
      */
    const Transform& respectToRef() const;
    
    /*!
      * \fn respectToWorld()
      * \brief Returns this frame's transformation with respect to the world
      *
      * This returns a reference, but it is const, so you cannot use this function
      * to modify this frame's transform with respect to the world. Instead, you
      * should use the function respectToRef(const Transform& newTf) and indicate
      * the desired transformation in this frame's reference frame. If you want to
      * change the reference frame, you must use the function
      * changeRefFrame(Frame& newRefFrame).
      *
      * Note that calling this function automatically performs all necessary
      * updates to the kinematic tree.
      */
    const Transform& respectToWorld();
    
    /*!
      * \fn withRespectTo()
      * \brief Returns this frame's transformation with respect to otherFrame
      * 
      * Note: If otherFrame is the reference frame, then it is more efficient
      * to use the function respectToRef(). If otherFrame is the World Frame,
      * it is more efficient to use the function respectToWorld().
      *
      * Also note that calling this function automatically performs all necessary
      * updates to any relevant kinematic trees.
      */
    Transform withRespectTo(Frame& otherFrame);

    /*!
     * \fn forceUpdate();
     * \brief Forces the frame to update itself
     *
     * This function is the same as _update(). The reason its name differs is to
     * reinforce the idea that you should not ever have to instruct the kinematic
     * model to update. It should do so automatically as necessary. Also, any use
     * of this function is guaranteed to reduce the efficiency of operation. However,
     * if for some reason you want the updates to happen according to a certain time
     * schedule, then this function could be used for that purpose.
     *
     * If you ever find that using this function changes the numerical results that
     * akin produces, please report this as an issue on Github, and try to provide
     * enough detail to recreate the problem.
     */
    void forceUpdate();

protected:
    
    void _update();

    Transform _respectToRef;
    Transform _respectToWorld;

    std::vector<Frame*> _childFrames;
    std::vector<KinObject*> _childObjects;

    void _gainChildFrame(Frame* child);
    void _loseChildFrame(Frame* child);

    void _gainChildObject(KinObject* child);
    void _loseChildObject(KinObject* child);

private:

    Frame(bool createWorld);

    bool _isWorld;
};

typedef std::vector<akin::Frame> FrameArray;
typedef std::vector<akin::Frame*> FramePtrArray;

} // namespace akin

inline std::ostream& operator<<(std::ostream& oStrStream, akin::Frame& mFrame)
{
    oStrStream << (akin::KinObject&)mFrame << " has relative transform:\n" << mFrame.respectToRef()
               << "\nAnd World transform:\n" << mFrame.respectToWorld() << std::endl;
    return oStrStream;
}

#endif // FRAME_H
