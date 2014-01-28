
#include "Link.h"

using namespace akin;
using namespace std;

Link::Link(Robot *mRobot, Frame &referenceFrame, string linkName, bool root) :
    Frame(referenceFrame, linkName),
    _myRobot(mRobot),
    _isRoot(root),
    _isAnchor(root),
    _isDummy(false)
{
    _needsUpdate = true;
}

Link::belongsTo(const Robot &someRobot) const
{
    if(_myRobot == &someRobot)
        return true;

    return false;
}
