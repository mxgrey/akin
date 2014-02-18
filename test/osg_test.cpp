/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/MatrixTransform>

#include "osgAkin/AkinCallback.h"
#include "osgAkin/Axes.h"
#include "osgAkin/KinTree.h"

using namespace std;
using namespace akin;

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

    lineGeom->setDataVariance(osg::Object::DYNAMIC);
    
    osgAkin::SpinData* spinData = new osgAkin::SpinData;
    spinData->lineVerts = lineVerts;
    spinData->geom = lineGeom;
    lineGeode->setUserData(spinData);
    lineGeode->setUpdateCallback(new osgAkin::SpinCallback);


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
    linewidth->setWidth(2.0f);
    lineGeode->getOrCreateStateSet()->setAttributeAndModes(linewidth);
    lineGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    
    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.run();
}

FramePtrArray createTrees()
{
    Frame* rootFrame = new Frame(Transform(Translation(1,0,0)), akin::Frame::World(), "rootFrame");
    Frame* secondFrame = new Frame(Transform(Translation(1,0,1),
                                Rotation(90*M_PI/180, Axis(0,0,1))), *rootFrame, "secondFrame");
    Frame* thirdFrame = new Frame(Transform(Translation(0,0,1)), *secondFrame, "thirdFrame");
    Frame* fourthFrame = new Frame(Transform(Translation(0, 1, 0.5)), *thirdFrame, "thirdFrame");

    Frame* newBranch = new Frame(Transform(Translation(-1,0,0),
                              Rotation(45*M_PI/180, Axis(0,1,0))), *secondFrame, "newBranch");
    Frame* more = new Frame(Transform(Translation(0.1, 0.5, -0.3)), *newBranch, "more");

    Frame* otherRoot = new Frame(Transform(Translation(-1,0,0)), akin::Frame::World(), "otherRoot");
    Frame* otherSecond = new Frame(Transform(Translation(0,1,1)), *otherRoot, "otherSecond");
    Frame* otherThird = new Frame(Transform(Translation(-1,0,0)), *otherSecond, "otherThird");

    FramePtrArray trees;
    trees.push_back(rootFrame);
    trees.push_back(otherRoot);

    return trees;
}


void spin_test(FramePtrArray trees)
{
    osg::Group* root = new osg::Group;
    osgAkin::SpinNode* akinNode = new osgAkin::SpinNode;
    root->addChild(akinNode);

    for(size_t i=0; i<trees.size(); ++i)
    {
        akinNode->addRootFrame(*(trees[i]));
    }
    
    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3f,0.3f,0.3f,1.0f));
    viewer.setSceneData(root);
    viewer.run();
}

void akin_test(FramePtrArray trees)
{
    osg::Group* root = new osg::Group;
    osgAkin::AkinNode* akinNode = new osgAkin::AkinNode;
    root->addChild(akinNode);

    for(size_t i=0; i<trees.size(); ++i)
    {
        akinNode->addRootFrame(*(trees[i]));
    }

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3f,0.3f,0.3f,1.0f));
    viewer.setSceneData(root);
    viewer.run();
}

int main(int argc, char* argv[])
{
//    akin_test(createTrees());
    spin_test(createTrees());

    return 0;
}
