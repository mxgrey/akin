
#include "akin/Body.h"

using namespace akin;
using namespace std;

Body::Body(Frame &referenceFrame, const string &bodyName) :
    Frame(referenceFrame, bodyName),
    com(*this, bodyName+"_com"),
    mass(0)
{
    com.setZero();
}
