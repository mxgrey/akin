#include "Frame.h"

using namespace akin;
using namespace std;

Frame::Frame(Frame& referenceFrame, std::string frameName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, frameName, report_level, "Frame"),
    _isWorld(false)
{
}

Frame::Frame(bool createWorld) :
    _isWorld(true),
    KinObject(*this, "World", verbosity::LOG, "World Frame", true)
{

}

Frame& Frame::World()
{
    static Frame world(true);
    return world;
}

bool Frame::isWorld() const { return _isWorld; }



bool Frame::changeRefFrame(Frame &newRefFrame)
{
    if(!verb.assert(this != &newRefFrame,
                    verbosity::ASSERT_CASUAL,
                    "You requested to make frame " + name() + " into its own reference"
                    + " frame, which is not permitted."))
        return false;

    if(!verb.assert(!newRefFrame.descendsFrom(*this),
                verbosity::ASSERT_CASUAL,
                "Cannot change the reference frame of " + name() + " to "
                + newRefFrame.name() + " because it creates a circular kinematic chain!",
                " We will leave " + name() + " in the frame of " + refFrame().name()))
        return false;

    KinObject::changeRefFrame(newRefFrame);
}

void Frame::_gainChildFrame(Frame *child)
{
    if(_isWorld)
        return;

    verb.debug() << "Adding " << child->name() << " into the Frame " << name();
    verb.end();

    _childFrames.push_back(child);
}

void Frame::_loseChildFrame(Frame *child)
{
    if(_isWorld)
        return;

    verb.debug() << "Removing " << child->name() << " from the Frame " << name();
    verb.end();
    int childIndex = -1;
    for(size_t i=0; i<_childFrames.size(); i++)
    {
        if(_childFrames[i] == child)
        {
            childIndex = i;
            break;
        }
    }

    if(childIndex == -1)
    {
        verb.brief() << "Trying to remove frame " << child->name() << " from the parentage of "
                     << name() << ", that is not its parent!";
        verb.desc() << " Child frames of " << name() << " include: ";
        for(int i=0; i<_childObjects.size(); i++)
            verb.desc() << " -- " << childFrame(i).name() << "\n";
        verb.end();

        verb.assert(false, verbosity::ASSERT_CASUAL, "");
    }
    else
    {
        _childFrames.erase(_childFrames.begin()+childIndex);
    }
}

void Frame::_gainChildObject(KinObject *child)
{
    if(_isWorld)
        return;

    verb.debug() << "Adding " << child->name() << " to the Frame " << name();
    verb.end();

    _childObjects.push_back(child);
}

void Frame::_loseChildObject(KinObject *child)
{
    if(_isWorld)
        return;

    verb.debug() << "Removing " << child->name() << " from the Frame " << name();
    verb.end();

    int childIndex = -1;
    for(size_t i=0; i<_childObjects.size(); i++)
    {
        if(_childObjects[i] == child)
        {
            childIndex = i;
            break;
        }
    }

    if(childIndex == -1)
    {
        verb.brief() << "Trying to remove " << child->name() << " from the parentage of "
                     << name() << ", but they are not related!";
        verb.desc() << " Children of " << name() << " include: ";
        for(int i=0; i<_childObjects.size(); i++)
            verb.desc() << " -- " << childObject(i).name() << "\n";
        verb.end();

        verb.assert(false, verbosity::ASSERT_CASUAL, "");
    }
    else
    {
        _childObjects.erase(_childObjects.begin()+childIndex);
    }
}

Frame& Frame::childFrame(size_t childFrameNum)
{
    if(_isWorld)
    {
        verb.brief() << "The World Frame does not keep track of its children!";
        verb.desc() << " Returning the World Frame instead.";
        verb.end();

        return World();
    }

    if(verb.assert(childFrameNum < _childFrames.size(),
                   verbosity::ASSERT_CASUAL,
                   "Requested non-existent child frame index in Frame "+name()))
        return *_childFrames[childFrameNum];
    else
    {
        verb.brief() << "Requested a child of Frame " << name()
                     << " which does not exist.";
        verb.desc() << " Returning the World Frame instead.";
        verb.end();
        return Frame::World();
    }
}
size_t Frame::numChildFrames() const { return _childFrames.size(); }

KinObject& Frame::childObject(size_t childObjNum)
{
    if(_isWorld)
    {
        verb.brief() << "The World Frame does not keep track of its children!";
        verb.desc() << " Returning a generic object instead.";
        verb.end();

        return KinObject::Generic();
    }

    if(verb.assert(childObjNum < _childFrames.size(),
                   verbosity::ASSERT_CASUAL,
                   "Requested non-existent child object index in Frame "+name()))
        return *_childObjects[childObjNum];
    else
    {
        verb.brief() << "Requested a child object of Frame " << name()
                     << " which does not exist."
                     << " Returning a generic object instead";
        verb.end();
        return KinObject::Generic();
    }
}
size_t Frame::numChildObjects() const { return _childObjects.size(); }
