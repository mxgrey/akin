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
    
    /*!
      * \fn Frame(Frame& referenceFrame, std::string frameName, verbosity::verbosity_level_t report_level)
      * \brief Constructor for frames
      *
      * 
      */
    Frame(Frame& referenceFrame = World(),
          std::string frameName = "arbitrary_frame",
          verbosity::verbosity_level_t report_level = verbosity::INHERIT);

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

    bool isWorld() const;

protected:

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

} // namespace akin

#endif // FRAME_H
