
#include "AkinIncludes.h"

using namespace akin;
using namespace std;


const Transform& KinTransform::respectToWorld()
{
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

Transform KinTransform::withRespectTo(Frame &someFrame)
{
    return someFrame.respectToWorld().inverse() * respectToWorld();
}

void KinTransform::_update()
{
    verb.debug() << "Updating transform '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * (Transform&)(*this);

    _needsUpdate = false;
}


