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

#ifndef AKIN_KINOBJECT_H
#define AKIN_KINOBJECT_H

#include "verbosity.h"
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Geometry.h"

namespace akin {

const double DEG = M_PI/180.0;
double mod(double x, double y);
double wrapToPi(double angle);

/*!
  * \def KinMacro( KinDerivedClass , DerivedClass )
  *
  * This macro is for customizing data types to place them into the kinematic
  * trees of akin. It is for classes which look like the following:
  *
  * class KinDerivedClass : public DerivedClass, public KinObject
  *
  * This macro should be placed inside of the class definition with the public:
  * field members. This macro will generate appropriate copy constructors and
  * assignment operators for you at runtime. Without this macro, a KinObject
  * cannot manage itself correctly, so it is crucial to have this included.
  *
  * This macro is most appropriate when all data members of your class are inherited
  * from DerivedClass and KinObject.
  * If your custom KinObject needs some custom initialization, use KinCustomMacro or
  * KinCustomMacro2.
  */

#define KinMacro(Y,X) inline Y ( const Y & copy ## Y ) :                                           \
                          X ( copy ## Y ),                                                         \
                          akin::KinObject( copy ## Y ) { }                                         \
                                                                                                   \
                      inline Y ( akin::Frame & referenceFrame, const std::string& X ## Name = #X , \
                        verbosity::verbosity_level_t report_level = verbosity::INHERIT ) :         \
                        X (), akin::KinObject( referenceFrame, X ## Name , report_level, #X ) { }  \
                                                                                                   \
                      inline Y ( const X & relative ## X , akin::Frame & referenceFrame,           \
                        const std::string& X ## Name = #X,                                         \
                        verbosity::verbosity_level_t report_level = verbosity::INHERIT ) :         \
                        X ( relative ## X ),                                                       \
                        akin::KinObject( referenceFrame, X ## Name , report_level, #X ) { }        \
                                                                                                   \
                      inline Y & operator=( const Y & copy ## Y )                                  \
                      {   (akin::KinObject&)(*this) = (akin::KinObject&)( copy ## Y );             \
                          ( X & )(*this) = ( X & )( copy ## Y );                                   \
                          notifyPosUpdate(); return *this; }                                          \
                                                                                                   \
                      inline Y & operator=( const X & copy ## X )                                  \
                      {   ( X & )(*this) = copy ## X ;                                             \
                          notifyPosUpdate(); return *this; }                                          \
                                                                                                   \
                      inline const X & respectToRef() const { return ( X & )(*this); }             

/*!
 * \fn _kinitialize( const DerivedClass& otherDerivedClass );
 * \brief Customizable initializer for classes derived from KinObject
 * \param other
 *
 * Use the macro KinCustomMacro to customize your KinObject to declare this function
 * and call it whenever a copy construction or assignment operation is performed.
 * This function is generated by KinCustomMacro and KinCustomMacro2
 */

/*!
  * \def KinCustomMacro2
  *
  * This macro is a more advanced version of KinMacro. Unlike KinMacro, this
  * exposes a function _kinitialize() which will be called by your class's
  * copy constructor and assignment operator.
  *
  * You need to provide a definition to _kinitialize(), because this function
  * only generates a declaration. If you do not want to define _kinitialize(),
  * then simply use KinMacro.
  *
  * If your class inherits more or less than one class (besides KinObject),
  * then you should use KinCustomMacro.
  */

#define KinCustomMacro2(Y,X) inline Y ( const Y & copy ## Y ) :                                    \
                                  akin::KinObject( copy ## Y ),                                    \
                                  X ( copy ## Y )                                                  \
                                  { _kinitialize( copy ## Y ); }                                   \
                              inline Y & operator=( const Y & copy ## Y )                          \
                              {   (akin::KinObject&)(*this) = (akin::KinObject&)( copy ## Y );     \
                                  ( X & )(*this) = ( X & )( copy ## Y );                           \
                                  _kinitialize( copy ## Y );                                       \
                                  notifyPosUpdate(); return *this; }                                  \
                              inline Y & operator=( const X & copy ## X )                          \
                              {   ( X & )(*this) = copy ## X ;                                     \
                                  notifyPosUpdate(); return *this; }                                  \
                              virtual void _kinitialize( const Y & copy );

/*!
  * \def KinCustomMacro
  *
  * This macro is an alternative to KinMacro, intended for classes which
  * have more data members than what they inherit, or which inherit more than
  * one class (besides KinObject).
  */

#define KinCustomMacro(Y) inline Y ( const Y & copy ## Y ) :                                       \
                            akin::KinObject( copy ## Y ) { _kinitialize( copy ## Y ); }            \
                          inline Y & operator=( const Y & copy ## Y )                              \
                            {   (akin::KinObject&)(*this) = (akin::KinObject&)( copy ## Y );       \
                                _kinitialize( copy ## Y );                                         \
                                notifyPosUpdate(); return *this; }                                    \
                          virtual void _kinitialize( const Y & copy );


class Frame; // Declaring Frame here so I can have a pointer to it in KinObject
class MultiTracker;

/*!
 * \class KinObject
 * \brief Metaclass for akin objects
 *
 * The KinObject parent class handles meta features which are common to all
 * classes/objects in the akin library, such as an object's name, its
 * parent frame, its verbosity level, and assertiveness level. This also
 * handles cleanly removing objects from a kinematic tree.
 */

class KinObject
{
public:

    friend class Frame;
    friend class MultiTracker;

    KinObject(Frame &referenceFrame,
              const std::string& myName,
              verbosity::verbosity_level_t report_level,
              const std::string& myType,
              bool thisIsTheWorld = false);

    KinObject(const KinObject& other);
    KinObject& operator=(const KinObject& other);

    virtual ~KinObject();

    /*!
      * \fn Generic()
      * \brief Returns a generic (empty) KinObject
      *
      * This function is often returned when there is an attempt to access a non-existent
      * KinObject in an array. The user might find this preferable to throwing a segfault.
      */
    static KinObject& Generic();

    /*!
      * \fn refFrame()
      * \brief Returns the reference frame of this KinObject
      *
      * The reference frame is a key concept in akin. Every KinObject (including
      * frames themselves) have a reference frame. Having a reference frame allows
      * KinObjects to express their relevant information in terms of whatever is
      * most convenient. The default reference frame for a KinObject is the World Frame.
      */
    Frame& refFrame() const;
//    const Frame& refFrame() const;

    /*!
      * \fn name()
      * \brief Returns the name given to this KinObject
      */
    virtual const std::string& name() const;
    
    /*!
      * \fn name(const std::string& newName)
      * \brief Changes the name of this KinObject
      *
      * Note that some implementations of KinObjects might change the
      * specific way that renaming works. Please check with the particular
      * KinObject that you are dealing with in order to know what to expect.
      */
    virtual bool name(const std::string& newName);
    
    /*!
      * \fn type()
      * \brief Returns a string containing the object type
      * 
      * Note that some implementations of KinObjects might return these
      * strings formatted in various ways. To know exactly what kind of
      * information to expect, please check with the particular KinObject
      * that you are dealing with.
      */
    virtual const std::string& type() const;
    
    /*!
      * \fn changeRefFrame()
      * \brief Changes the reference frame of this KinObject
      *
      * In most cases, this function will alter the relative value of the
      * KinObject so that it does not move in the world frame. But implementations
      * may vary, so consult the particular KinObject that you are dealing with
      * in order to know what to expect.
      *
      * Returns false if the requested change of reference frame was not valid for any reason.
      */
    virtual bool changeRefFrame(Frame& newRefFrame); // TODO: Make a bitflag of options for this
                                                     // such as retaining the relative value

    /*!
      * \fn descendsFrom()
      * \brief Returns true if and only if this KinObject is kinematically downstream from someFrame
      */
    bool descendsFrom(const Frame& someFrame) const;
    
    

    
    mutable verbosity verb;

    /*!
      * \fn notifyPosUpdate()
      * \brief Notify this object that it needs to update itself
      *
      * This function is triggered when a change occurs anywhere upstream on the
      * kinematic tree. It indicates to the KinObject that its location in the
      * world has changed.
      */
    virtual void notifyPosUpdate();
    
    /*!
      * \fn needsUpdate()
      * \brief Returns true if this object needs to update itself
      */
    bool needsUpdate() const;
    
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
    KinObject& registeredObject(size_t objNum);
    
    /*!
      * \fn numChildObjects()
      * \brief Returns the frame's current number of child KinObjects
      */
    size_t numRegisteredObjects() const;
    
    /*!
      * \fn addVisual(const Geometry& visual_geometry)
      * \brief Add a geometry to visualize this KinObject
      * 
      * Visuals are geometries that are intended to be rendered for visualization.
      * These are usually of higher resolution than colliders because they are
      * meant to be visually appealing and their data does not need to be handled
      * regularly.
      *
      * To add geometries that are intended for collision-checking, see addCollider().
      */
    size_t addVisual(const Geometry& visual_geometry);
    
    /*!
      * \fn removeVisual(size_t num)
      * \brief Removes a stored visual
      *
      * Important note: Using this function will reduce the index values of
      * all visuals which follow it in the array by one.
      *
      * Returns false iff num is out of bounds
      */
    bool removeVisual(size_t num);
    
    /*!
      * \fn clearVisuals()
      * \brief Removes all visuals that are being kept
      */
    void clearVisuals();
    
    /*!
      * \fn peekVisual(size_t num)
      * \brief Check a visual geometry object without touching the 'updated' flag
      *
      * To make data handling as efficient and easy as possible, KinObjects do internal
      * updating management. This depends on flags being set and unset at the
      * appropriate times. As the end user, you should use this function or the
      * peekVisuals() function if you find it necessary to inspect this KinObject's
      * visual array members.
      *
      * Returns the last visual in the array if num is out of bounds.
      */
    const Geometry& peekVisual(size_t num) const;
    /*!
      * \fn peekVisuals()
      * \brief Similar to peekVisual() but returns the full array by reference
      */
    const GeometryArray& peekVisuals() const;
    
    /*!
      * \fn addCollider(size_t num)
      * \brief Add a geometry intended for collision checking
      *
      * Colliders are geometries that are intended to be collision checked.
      * Ideally these are convex primitives, but sometimes simple meshes may
      * be needed.
      */
    size_t addCollider(const Geometry& colliding_geometry);
    
    /*!
      * \fn removeCollider(size_t num)
      * \brief Removes a stored collider
      *
      * Important note: Using this function will reduce the index values of
      * all visuals which follow it in the array by one.
      */
    bool removeCollider(size_t num);
    
    /*!
      * \fn clearColliders()
      * \brief Removes all stored colliders
      */
    void clearColliders();
    
    /*!
      * \fn peekCollider(size_t num)
      * \brief Check a collision geometry object without touching the 'updated' flag
      *
      * To make data handling as efficient and easy as possible, KinObjects do internal
      * updating management. This depends on flags being set and unset at the appropriate
      * times. As the end user, you should use this function or the peekColliders() 
      * function if you find it necessary to inspect this KinObject's collision array
      * members.
      */
    const Geometry& peekCollider(size_t num) const;
    
    /*!
      * \fn peekColliders()
      * \brief Similar to peekCollider() but returns the full array by reference
      */
    const GeometryArray& peekColliders() const;
    
    /*!
      * \fn visualsChanged()
      * \brief true iff something has changed with the visuals since the last call to grabVisualsAndReset()
      */
    bool visualsChanged() const;
    
    /*!
      * \fn collidersChanged()
      * \brief true iff something has changed with the colliders since the last call to grabCollidersAndReset()
      */
    bool collidersChanged() const;
    
    /*!
      * \fn grabVisualsAndReset()
      * \brief Similar to peekVisuals() but resets the visualsChanged() flag
      *
      * Note: Meant only for internal use. Using this function inappropriately
      * could negatively impact visualization. If you are implementing a custom visualization
      * interface then using this function might be appropriate.
      */
    const GeometryArray& grabVisualsAndReset();
    
    /*!
      * \fn grabCollidersAndReset()
      * \brief Similar to peekColliders() but resets the collidersChanged() flag
      *
      * Note: Meant only for internal use. Using this function inappropriately
      * could negatively impact collision checking. If you are implementing a custom
      * collision checker then using this function might be appropriate.
      */
    const GeometryArray& grabCollidersAndReset();

    bool isFrame() const;

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
    
protected:
    
    std::vector<KinObject*> _registeredObjects;
    
    void _registerObject(KinObject* child);
    void _unregisterObject(KinObject* child);
    void _setReferenceFrame(Frame& newRefFrame);

    void _copyValues(const KinObject& other);
    
    GeometryArray _colliders;
    bool _collidersUpdate;
    GeometryArray _visuals;
    bool _visualsUpdate;

    bool _isFrame;

    /*!
     * \fn _loseParent();
     * \brief This function is called when this object's parent's destructor is called
     *
     * Overload this function if you want a particular behavior when the parent is
     * destroyed. Default is to simply change the child's parent class to the World.
     * This means that whatever relationship the class had with its parent frame, it
     * will then have with the World Frame.
     */
    virtual void _loseParent(KinObject*);

    std::string _name;
    std::string _type;

    Frame* _referenceFrame;
    
    mutable bool _needsPosUpdate;

private:

    bool _isWorld;

};

typedef std::vector<KinObject> KinObjectArray;
typedef std::vector<KinObject*> KinObjectPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::KinObject& mObject);

#endif // AKIN_KINOBJECT_H
