/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AKIN_FRAME_H
#define AKIN_FRAME_H

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
public:

    friend class KinObject;

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
     * \fn isLink()
     * \brief Return true if the frame is also a robot's link
     * \return 
     */
    bool isLink() const;
    
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
    const Transform& respectToWorld() const;
    
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
    Transform withRespectTo(const Frame& otherFrame) const;

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
    
    void _update() const;

    Transform _respectToRef;
    mutable Transform _respectToWorld;

    std::vector<Frame*> _childFrames;

    void _gainChildFrame(Frame* child);
    void _loseChildFrame(Frame* child);
    
    bool _isLink;

private:

    Frame(bool);

    bool _isWorld;
};

typedef std::vector<akin::Frame> FrameArray;
typedef std::vector<akin::Frame*> FramePtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::Frame& mFrame);

#endif // AKIN_FRAME_H
