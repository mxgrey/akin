#ifndef ROBOTNODE_H
#define ROBOTNODE_H

#include "Link.h"
#include "AkinNode.h"

namespace osgAkin {

typedef std::map<akin::Link*,osg::MatrixTransform*> LinkMtfMap;

class RobotNode : public AkinNode
{
public:
    virtual void update();

    void addRobot(akin::Robot& new_robot);

protected:

    akin::RobotPtrArray _robots;

};

} // namespace osgAkin

#endif // ROBOTNODE_H
