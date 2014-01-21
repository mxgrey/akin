#ifndef AKINCALLBACK_H
#define AKINCALLBACK_H

#include "AkinIncludes.h"
#include "AkinData.h"
#include "AkinNode.h"
#include "osg/NodeCallback"
#include "osg/NodeVisitor"

namespace osgAkin {

class AkinCallback : public osg::NodeCallback
{
public:

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::ref_ptr<AkinNode> currentNode =
                dynamic_cast<AkinNode*>(node);
        
        currentNode->update();

        traverse(node, nv);
    }
};

class SpinData : public osg::Referenced
{
public:
    osg::Vec3Array* lineVerts;
    osg::Geometry* geom;
};

class SpinCallback : public osg::NodeCallback
{
public:

    SpinCallback() : time(0) { }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
//        osg::ref_ptr<osg::Vec3Array> lineVerts =
//                dynamic_cast<osg::Vec3Array*>(node->getUserData());
        osg::ref_ptr<SpinData> data =
                dynamic_cast<SpinData*>(node->getUserData());
        
        if(data)
        {
//            std::cout << "We have received line verts: " << (*lineVerts)[1].x() << " " << (*lineVerts)[1].z() << std::endl;
//            float r = sqrt((*lineVerts)[1].x() * (*lineVerts)[1].x() + (*lineVerts)[1].z() * (*lineVerts)[1].z());
//            time += 0.05;
//            (*lineVerts)[1].x() = r * cos(time);
//            (*lineVerts)[1].z() = r * sin(time);
            osg::Vec3& vec = (*data->lineVerts)[1];
//            std::cout << "We have received line verts: " << vec.x() << ", " << vec.z() << std::endl;
            float r = sqrt(vec.x()*vec.x() + vec.z()*vec.z());
            time += 0.05;
            vec.x() = r * cos(time);
            vec.z() = r * sin(time);
            
            data->geom->setVertexArray(data->lineVerts);
        }

        traverse(node, nv);
    }

protected:
    float time;
};


} // namespace osgAkin

#endif // AKINCALLBACK_H
