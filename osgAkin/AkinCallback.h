#ifndef AKINCALLBACK_H
#define AKINCALLBACK_H

#include "AkinIncludes.h"
#include "AkinData.h"
#include "osg/NodeCallback"
#include "osg/NodeVisitor"

namespace osgAkin {

class AkinCallback : public osg::NodeCallback
{
public:

    AkinCallback() : time(0) { }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::ref_ptr<AkinData> incomingData =
                dynamic_cast<AkinData*>(node->getUserData());
        if(incomingData)
        {
            for(size_t i=0; i<incomingData->geomArray.size(); ++i)
            {
                incomingData->geomArray[i]->updateFrames();
            }
        }
        traverse(node, nv);
    }
protected:
    float time;
};

class SpinCallback : public osg::NodeCallback
{
public:

    SpinCallback() : time(0) { }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::ref_ptr<osg::Vec3Array> lineVerts =
                dynamic_cast<osg::Vec3Array*>(node->getUserData());
        if(lineVerts)
        {
            std::cout << "We have received line verts: " << (*lineVerts)[1].x() << " " << (*lineVerts)[1].z() << std::endl;
            float r = sqrt((*lineVerts)[1].x() * (*lineVerts)[1].x() + (*lineVerts)[1].z() * (*lineVerts)[1].z());
            time += 0.05;
            (*lineVerts)[1].x() = r * cos(time);
            (*lineVerts)[1].z() = r * sin(time);
        }



        traverse(node, nv);
    }

protected:
    float time;
};


} // namespace osgAkin

#endif // AKINCALLBACK_H
