
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/Viewer>
#include <osgGA/GUIEventAdapter>

void pick(osgViewer::Viewer& viewer, const osgGA::GUIEventAdapter& ea)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;

    if(viewer.computeIntersections(ea.getX(), ea.getY(), intersections))
    {
        for(osgUtil::LineSegmentIntersector::Intersections::iterator hit = intersections.begin();
            hit != intersections.end(); ++hit)
        {
            const osgUtil::LineSegmentIntersector::Intersection& h = *hit;
        }
    }
}

int main(int, char* [])
{



    return 0;
}
