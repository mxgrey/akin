
#include "Transform.h"

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


std::ostream& operator<<(std::ostream& oStrStream, const KinTransform& mTransform)
{
    oStrStream << (KinObject&)mTransform << " has matrix:\n" << (Transform&)mTransform << endl;
}
