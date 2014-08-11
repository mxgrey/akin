
#include "../Geometry.h"
#include <algorithm>
#include <iostream>

class HullAngle
{
public:

    HullAngle() : angle(INFINITY), index(-1) { }
    HullAngle(double angle_, size_t index_) : angle(angle_), index(index_) { }

    double angle;
    size_t index;
};

bool HullAngleComparison(const HullAngle& a, const HullAngle& b)
{
    return a.angle < b.angle;
}

std::vector<Eigen::Vector2d> akin::computeConvexHull(std::vector<Eigen::Vector2d>& points)
{
    std::vector<Eigen::Vector2d> hull;

    size_t lowest = (size_t)(-1); double y = INFINITY;
    for(size_t i=0; i<points.size(); ++i)
    {
        if(points[i][1] < y)
        {
            lowest = i;
            y = points[i][1];
        }
        else if(points[i][1] == y)
        {
            if(points[i][0] < points[lowest][0])
            {
                lowest = i;
                y = points[i][0];
            }
            else if(points[i][0] == points[lowest][0])
            {
                std::cout << "Found repeated point while computing convex hull!" << std::endl;
                points.erase(points.begin()+i);
                --i;
            }
        }
    }

    std::vector<HullAngle> angles;
    for(size_t i=0; i<points.size(); ++i)
    {
        if(i != lowest)
            angles.push_back(HullAngle(atan2(points[i][1]-points[lowest][1], 
                                             points[i][0]-points[lowest][0]), i));
    }

    std::sort(angles.begin(), angles.end(), HullAngleComparison);
    
    std::vector<size_t> added;
    size_t last = lowest;
    size_t second_to_last = angles[0].index;
    added.push_back(last);
    for(size_t i=1; i<angles.size(); ++i)
    {
        size_t current = angles[i].index;
        const Eigen::Vector2d& p1 = points[last];
        const Eigen::Vector2d& p2 = points[second_to_last];
        const Eigen::Vector2d& p3 = points[current];
        
        bool left_turn = ( (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]) > 0 );
        
        if(left_turn)
        {
            added.push_back(second_to_last);
            last = second_to_last;
            second_to_last = current;
        }
        else
        {
            second_to_last = added.back();
            added.pop_back();
            last = added.back();
            --i;
        }
    }
    
    const Eigen::Vector2d& p1 = points[added.back()];
    const Eigen::Vector2d& p2 = points[angles.back().index];
    const Eigen::Vector2d& p3 = points[lowest];
    bool left_turn = ( (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]) > 0 );
    if(left_turn)
        added.push_back(angles.back().index);
    
    for(size_t i=0; i<added.size(); ++i)
        hull.push_back(points[added[i]]);
    
    return hull;
}

Eigen::Vector2d akin::computeCentroid(const std::vector<Eigen::Vector2d>& convexHull)
{
    if(convexHull.size()==0)
        return Eigen::Vector2d::Zero();
    
    if(convexHull.size()==1)
        return convexHull.front();
    
    if(convexHull.size()==2)
        return (convexHull[0]+convexHull[1])/2;
    
    Eigen::Vector2d c(0,0);
    Eigen::Vector2d intersect;
    double area = 0;
    double area_i;
    Eigen::Vector2d midp12, midp01;
    for(size_t i=2; i<convexHull.size(); ++i)
    {
        const Eigen::Vector2d& p0 = convexHull[0];
        const Eigen::Vector2d& p1 = convexHull[i-1];
        const Eigen::Vector2d& p2 = convexHull[i];
        
        area_i = 0.5*((p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0]));
        area += area_i;
        
        midp12 = 0.5*(p1+p2);
        midp01 = 0.5*(p0+p1);
        
        intersection_t result = computeIntersection(intersect, p0, midp12, p2, midp01);
//        intersection_t result = computeIntersection(intersect, p0, midp12, midp01, p2);
        if( result != INTERSECTING )
        {
            std::cout << "Trying to find centroid of poorly conditioned convex hull (" 
                      << result << ")" << std::endl;
        }
        
        c += area_i*intersect;
    }
    
    c = c/area;
    
    return c;
}

akin::intersection_t akin::computeIntersection(Eigen::Vector2d& intersection, 
                               const Eigen::Vector2d& a1,
                               const Eigen::Vector2d& a2, 
                               const Eigen::Vector2d& b1, 
                               const Eigen::Vector2d& b2)
{
    double dx_a = a2[0] - a1[0];
    double dy_a = a2[1] - a1[1];
    
    double dx_b = b2[0] - b1[0];
    double dy_b = b2[1] - b1[1];
    
    if( dx_b*dy_a - dx_a*dy_b == 0 )
    {
        intersection = (a1+a2+b1+b2)/4;
        return PARALLEL;
    }
    
    intersection[0] = (dx_b*dy_a*a1[0] - dx_a*dy_b*b1[0] + dx_a*dx_b*(b1[1]-a1[1]))
                            /(dx_b*dy_a - dx_a*dy_b);
    
    if( dx_a != 0 )
        intersection[1] = dy_a/dx_a*(intersection[0]-a1[0]) + a1[1];
    else
        intersection[1] = dy_b/dx_b*(intersection[0]-b1[0]) + b1[1];
    
    for(size_t i=0; i<2; ++i)
    {
        if( (intersection[i] < std::min(a1[i],a2[i])) || std::max(a1[i],a2[i]) < intersection[i] )
        {
            std::cout << "violation A " << i << std::endl;
            return BEYOND_ENDPOINTS;
        }
        
        if( (intersection[i] < std::min(b1[i],b2[i])) || std::max(b1[i],b2[i]) < intersection[i] )
        {
            std::cout << "violation B " << i << std::endl;
            return BEYOND_ENDPOINTS;
        }
    }
    
    return INTERSECTING;
}





