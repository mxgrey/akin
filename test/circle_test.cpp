
#include "osgAkin/Line.h"
#include "osgAkin/AkinNode.h"

using namespace osgAkin;

void make_circle(Line* circle, const akin::Translation& center, double radius)
{
    size_t res = 1000;
    for(size_t i=0; i<=res; ++i)
    {
        double x = center.x() + radius*cos(2*M_PI*i/res);
        double y = /*center.y() + radius*sin(2*M_PI*i/res);*/ 0;
        double z = center.z() + radius*sin(2*M_PI*i/res);
        circle->addVertex(akin::Translation(x,y,z));
    }
    circle->updateVertices();
}

int main(int argc, char* argv[])
{
    osg::Group* root = new osg::Group;
    osg::Geode* geode = new osg::Geode;
    root->addChild(geode);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    
    osgAkin::Line* circle = new Line;
    make_circle(circle, akin::Translation(0,0), 1);
    circle->setColor(osg::Vec4(0.1f,0.1f,0.1f,1.0f));
    
    
    geode->addDrawable(circle);
    
    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.9f,0.9f,0.9f,1.0f));
    viewer.setSceneData(root);
    viewer.run();
    
    return 0;
}
