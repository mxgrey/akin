
#include "../ConvexHull.h"
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
        angles.push_back(HullAngle(atan2(points[i][1]-hull[0][1], points[i][0]-hull[0][0]), i));
    }

    std::sort(angles.begin(), angles.end(), HullAngleComparison);
    
    std::vector<size_t> added;
    size_t last = lowest;
    size_t second_to_last = angles[1].index;
    added.push_back(last);
    for(size_t i=2; i<angles.size()-1; ++i)
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
    added.push_back(angles.back().index);
    
    for(size_t i=0; i<added.size(); ++i)
        hull.push_back(points[added[i]]);
    

    return hull;
}
