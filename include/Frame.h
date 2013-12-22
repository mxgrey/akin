#ifndef FRAME_H
#define FRAME_H

#include "verbosity.h"
#include <vector>

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

    Frame& refFrame();

    std::string name();
    virtual void name(std::string newName);

    verbosity verb;

protected:

    std::string _name;
    std::string _type;

    Frame* _referenceFrame;

};

class Frame : public KinObject
{
public:
    Frame(Frame& referenceFrame = World(),
          std::string frameName = "arbitrary frame",
          verbosity::verbosity_level_t report_level = verbosity::LOG);

    static Frame& World();

    Frame* childFrame(size_t childNum);
    Frame* childObject(size_t childNum);

    bool isWorld();

protected:

    std::vector<Frame*> _childFrames;
    std::vector<KinObject*> _childObjects;

private:

    Frame(bool createWorld);

    bool _isWorld;
};

} // namespace akin

#endif // FRAME_H
