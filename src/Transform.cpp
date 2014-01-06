
#include "Transform.h"
#include "Frame.h"

using namespace akin;
using namespace std;

KinTransform::KinTransform(Frame &referenceFrame, string tfName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, tfName, report_level, "Transform"),
    Transform()
{
    
}

KinTransform::KinTransform(const Transform &relativeTf, Frame &referenceFrame, string tfName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, tfName, report_level, "Transform"),
    Transform(relativeTf)
{
    
}

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
}

std::ostream& operator<<(std::ostream& oStrStream, const KinTransform& mTransform)
{
    oStrStream << (KinObject&)mTransform << " has matrix:\n" << (Transform&)mTransform << endl;
}
