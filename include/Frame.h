#ifndef FRAME_H
#define FRAME_H

#include "verbosity.h"

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
    KinObject();

    ~KinObject();

    Frame& refFrame();

    std::string name();
    virtual void name(std::string newName);

    verbosity verb;

protected:

    std::string _name;

    Frame* _referenceFrame;

private:


};

class Frame : protected KinObject
{
public:
    Frame();
};

} // namespace akin

#endif // FRAME_H
