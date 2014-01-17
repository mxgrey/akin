
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>

#include "AkinCallback.h"


void pyramid_test()
{
    osg::Group* root = new osg::Group();
    osg::Geode* pyramidGeode = new osg::Geode();
    osg::Geometry* pyramidGeometry = new osg::Geometry();
    
    pyramidGeode->addDrawable(pyramidGeometry);
    root->addChild(pyramidGeode);
    
    osg::Vec3Array* pyramidVertices = new osg::Vec3Array;
    pyramidVertices->push_back(osg::Vec3(0,0,0));
    pyramidVertices->push_back(osg::Vec3(10,0,0));
    pyramidVertices->push_back(osg::Vec3(10,10,0));
    pyramidVertices->push_back(osg::Vec3(0,10,0));
    pyramidVertices->push_back(osg::Vec3(5,5,10));
    
    pyramidGeometry->setVertexArray(pyramidVertices);
    osg::DrawElementsUInt* pyramidBase =
            new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    pyramidBase->push_back(3);
    pyramidBase->push_back(2);
    pyramidBase->push_back(1);
    pyramidBase->push_back(0);
    pyramidGeometry->addPrimitiveSet(pyramidBase);
    
    osg::DrawElementsUInt* pyramidFaceOne =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceOne->push_back(0);
    pyramidFaceOne->push_back(1);
    pyramidFaceOne->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceOne);
    
    osg::DrawElementsUInt* pyramidFaceTwo =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceTwo->push_back(1);
    pyramidFaceTwo->push_back(2);
    pyramidFaceTwo->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceTwo);
    
    osg::DrawElementsUInt* pyramidFaceThree =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceThree->push_back(2);
    pyramidFaceThree->push_back(3);
    pyramidFaceThree->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceThree);
    
    osg::DrawElementsUInt* pyramidFaceFour =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceFour->push_back(3);
    pyramidFaceFour->push_back(0);
    pyramidFaceFour->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceFour);
    
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    
    pyramidGeometry->setColorArray(colors);
    pyramidGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
//    osg::PositionAttitudeTransform* pyramidTwoXForm =
//            new osg::PositionAttitudeTransform();
    
//    root->addChild(pyramidTwoXForm);
//    pyramidTwoXForm->addChild(pyramidGeode);
    
//    osg::Vec3 pyramidTwoPosition(15, 0, 0);
//    pyramidTwoXForm->setPosition(pyramidTwoPosition);
    
    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.run();
    
}


void line_test()
{
    osg::Group* root = new osg::Group;
    osg::Geode* lineGeode = new osg::Geode;
    root->addChild(lineGeode);

    osg::Geometry* lineGeom = new osg::Geometry;
    lineGeode->addDrawable(lineGeom);

    osg::Vec3Array* lineVerts = new osg::Vec3Array;
    lineVerts->push_back(osg::Vec3(0,0,0));
    lineVerts->push_back(osg::Vec3(1,0,2));
    lineGeom->setVertexArray(lineVerts);

    osgAkin::AkinData* data = new osgAkin::AkinData;
//    data->lineVerts = lineVerts;
//    data->lineGeom = lineGeom;
    lineGeode->setUserData(data);
    lineGeom->setDataVariance(osg::Object::DYNAMIC);


    osg::Vec4Array* color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    lineGeom->setColorArray(color);
    lineGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::DrawElementsUShort* lineElem = new osg::DrawElementsUShort(osg::PrimitiveSet::LINES,0);
    lineElem->push_back(0);
    lineElem->push_back(1);
    lineGeom->addPrimitiveSet(lineElem);

    osg::Geometry* greenGeom = new osg::Geometry;
    lineGeode->addDrawable(greenGeom);

    osg::Vec3Array* greenVerts = new osg::Vec3Array;
    greenVerts->push_back(osg::Vec3(0, 0, 0));
    greenVerts->push_back(osg::Vec3(0.5, 0, 0));
    greenGeom->setVertexArray(greenVerts);

    greenGeom->addPrimitiveSet(lineElem);


    osg::Vec4Array* green = new osg::Vec4Array;
    green->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));

    greenGeom->setColorArray(green);
    greenGeom->setColorBinding(osg::Geometry::BIND_OVERALL);


    osg::LineWidth* linewidth = new osg::LineWidth;
    linewidth->setWidth(5.0f);
    lineGeode->getOrCreateStateSet()->setAttributeAndModes(linewidth);
    lineGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


    lineGeode->setUpdateCallback(new osgAkin::AkinCallback);
    
    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.run();
}



int main(int argc, char* argv[])
{
    line_test();
//    pyramid_test();
    
    return 0;
}
