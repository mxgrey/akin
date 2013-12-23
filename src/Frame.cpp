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

void Frame::_gainChildFrame(Frame *child) { _childFrames.push_back(child); }

void Frame::_loseChildFrame(Frame *child)
{
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

void Frame::_gainChildObject(KinObject *child) { _childObjects.push_back(child); }

void Frame::_loseChildObject(KinObject *child)
{
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
    if(verb.assert(childFrameNum < _childFrames.size(),
                   verbosity::ASSERT_CASUAL,
                   "Requested non-existent child frame index in Frame "+name()))
        return *_childFrames[childFrameNum];
    else
    {
        verb.brief() << "Requesting child number " << string::childFrameNum
                     << " of frame " << name() << ", but it does not exist."
                     << " Returning the World Frame instead";
        verb.end();
        return Frame::World();
    }
}

