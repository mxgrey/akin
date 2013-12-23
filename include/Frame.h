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

    static KinObject& Generic();

    Frame& refFrame();

    std::string name();
    virtual void name(std::string newName);
    virtual bool changeRefFrame(Frame& newRefFrame);

    bool descendsFrom(const Frame& someFrame);

    verbosity verb;



protected:

    std::string _name;
    std::string _type;

    Frame* _referenceFrame;

private:

};

class Frame : public KinObject
{
    friend class KinObject;

public:
    Frame(Frame& referenceFrame = World(),
          std::string frameName = "arbitrary frame",
          verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    static Frame& World();

    Frame& childFrame(size_t childFrameNum);
    size_t numChildFrames() const;

    KinObject &childObject(size_t childObjNum);
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
