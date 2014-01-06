#ifndef KINOBJECT_H
#define KINOBJECT_H

#include "verbosity.h"
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace akin {



class Frame; // Declaring Frame here so I can have a pointer to it in KinObject

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
    KinObject(Frame &referenceFrame,
              std::string myName,
              verbosity::verbosity_level_t report_level,
              std::string myType,
              bool thisIsTheWorld = false);

    ~KinObject();

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

    /*!
      * \fn name()
      * \brief Returns the name given to this KinObject
      */
    std::string name() const;
    
    /*!
      * \fn name(std::string newName)
      * \brief Changes the name of this KinObject
      *
      * Note that some implementations of KinObjects might change the
      * specific way that renaming works. Please check with the particular
      * KinObject that you are dealing with in order to know what to expect.
      */
    virtual void name(std::string newName);
    
    /*!
      * \fn type()
      * \brief Returns a string containing the object type
      * 
      * Note that some implementations of KinObjects might return these
      * strings formatted in various ways. To know exactly what kind of
      * information to expect, please check with the particular KinObject
      * that you are dealing with.
      */
    virtual std::string type() const;
    
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
    bool descendsFrom(const Frame& someFrame);

    
    verbosity verb;

    /*!
      * \fn notifyUpdate()
      * \brief Notify this object that it needs to update itself
      *
      * This function is triggered when a change occurs anywhere upstream on the
      * kinematic tree. It indicates to the KinObject that its location in the
      * world has changed.
      */
    virtual void notifyUpdate();
    
    /*!
      * \fn needsUpdate()
      * \brief Returns true if this object needs to update itself
      */
    bool needsUpdate();
    
protected:

    std::string _name;
    std::string _type;

    Frame* _referenceFrame;
    
    bool _needsUpdate;

private:

};

} // namespace akin


std::ostream& operator<<(std::ostream& oStrStream, const akin::KinObject& mObject);

#endif // KINOBJECT_H
