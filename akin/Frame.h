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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /*!
      * \fn Frame(Frame& referenceFrame, std::string frameName, verbosity::verbosity_level_t report_level)
      * \brief Constructor for frames
      * 
      */
    Frame(Frame& referenceFrame = World(),
          std::string frameName = "arbitrary_frame",
          verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    Frame(const Transform& relativeTf,
          Frame& referenceFrame = World(),
          std::string frameName = "arbitrary_frame",
          verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    virtual ~Frame();
    
    typedef enum {

        LINEAR,
        ANGULAR

    } coord_t;
    
    /*!
      * \fn World()
      * \brief Returns the World Frame
      * 
      * The World Frame is a key concept in akin, because it allows arbitrary
      * kinematic trees to understand their relationships with each other. The
      * World Frame is static in the sense that none of its features may be
      * altered by the user at any time. It simply exists, and always has a
      * world transform of identity.
      */
    static Frame& World();
    // TODO: Consider reworking mutability in order to allow World() to return a const reference
    
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
    virtual const Transform& respectToRef() const;
    
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
      * Note: If otherFrame is the reference frame, then this function will call
      * respectToRef(). If otherFrame is the World Frame, then it will call the
      * function respectToWorld().
      *
      * Also note that calling this function automatically performs all necessary
      * updates to any relevant kinematic trees.
      */
    Transform withRespectTo(const Frame& otherFrame) const;
    
    // Velocity API ---------------------------------------------------------
    const Velocity& linearVelocity() const;
    Velocity linearVelocity(const Frame& withRespectToFrame) const;
    virtual const Velocity& relativeLinearVelocity() const;
    virtual void relativeLinearVelocity(const Velocity& v);

    const Velocity& angularVelocity() const;
    Velocity angularVelocity(const Frame& withRespectToFrame) const;
    virtual const Velocity& relativeAngularVelocity() const;
    virtual void relativeAngularVelocity(const Velocity& w);

    const Velocity& velocity(coord_t type) const;
    Velocity velocity(coord_t type, const Frame& withRespectToFrame) const;
    virtual void relativeVelocity(const Velocity& v, coord_t type);

    Screw velocity(const Frame& withRespectToFrame=Frame::World()) const;
    Screw relativeVelocity() const;
    void relativeVelocity(const Screw& v_w);

    // Acceleration API ------------------------------------------------------
    const Acceleration& linearAcceleration() const;
    Acceleration linearAcceleration(const Frame& withRespectToFrame) const;
    virtual const Acceleration& relativeLinearAcceleration() const;
    virtual void relativeLinearAcceleration(const Acceleration& a);

    const Acceleration& angularAcceleration() const;
    Acceleration angularAcceleration(const Frame& withRespectToFrame) const;
    virtual const Acceleration& relativeAngularAcceleration() const;
    virtual void relativeAngularAcceleration(const Acceleration& w_dot);

    const Acceleration& acceleration(coord_t type) const;
    Acceleration acceleration(coord_t type, const Frame& withRespectToFrame) const;
    void relativeAcceleration(const Acceleration& a, coord_t type);

    Screw acceleration(const Frame& withRespectToFrame=Frame::World()) const;
    Screw relativeAcceleration() const;
    void relativeAcceleration(const Screw& a_wdot);
    
    const KinAcceleration& gravity() const;
    void gravity(const KinAcceleration& g, bool recursive=true);
    void gravity(const Eigen::Vector3d& g, Frame& inFrame, bool recursive=true);

    // Update functions -------------------------------------------------------
    virtual void notifyPosUpdate();
    
    /*!
     * \fn demandPoseUpdate();
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
    void demandPosUpdate() const;
    
    virtual void notifyVelUpdate();
    bool needsVelUpdate() const;
    void demandVelUpdate() const;

    virtual void notifyAccUpdate();
    bool needsAccUpdate() const;
    void demandAccUpdate() const;

protected:
    
    void _posUpdate() const;
    void _velUpdate() const;
    void _accUpdate() const;

    mutable bool _needsVelUpdate;
    mutable bool _needsAccUpdate;

    Transform _respectToRef;
    mutable Transform _respectToWorld;
    
    Velocity _relativeLinearVel;
    Velocity _relativeAngularVel;
    mutable Velocity _linearVel_wrtWorld;
    mutable Velocity _angularVel_wrtWorld;

    mutable Acceleration _relativeLinearAcc;
    mutable Acceleration _relativeAngularAcc;
    mutable Acceleration _linearAcc_wrtWorld;
    mutable Acceleration _angularAcc_wrtWorld;
    
    KinAcceleration _gravity;
    
    std::vector<Frame*> _childFrames;

    void _gainChildFrame(Frame* child);
    void _loseChildFrame(Frame* child);
    
    bool _isLink;

private:

    explicit Frame(bool);

};

Translation computeLocation(const Translation& ofPoint,
                            const Frame& inFrame,
                            const Frame& withRespectToFrame = Frame::World());

Velocity computeVelocity(const Translation& ofPoint,
                         const Velocity& withVelocity,
                         const Frame& inFrame,
                         const Frame& withRespectToFrame = Frame::World());

Velocity computeVelocity(const Translation& ofPoint,
                         const Frame& inFrame,
                         const Frame& withRespectToFrame = Frame::World());

Acceleration computeAcceleration(const Translation& ofPoint,
                                 const Velocity& withVelocity,
                                 const Acceleration& withAcceleration,
                                 const Frame& inFrame,
                                 const Frame& withRespectToFrame = Frame::World());

Acceleration computeAcceleration(const Translation& ofPoint,
                                 const Frame& inFrame,
                                 const Frame& withRespectToFrame = Frame::World());

typedef std::vector<akin::Frame> FrameArray;
typedef std::vector<akin::Frame*> FramePtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::Frame& mFrame);

#endif // AKIN_FRAME_H
