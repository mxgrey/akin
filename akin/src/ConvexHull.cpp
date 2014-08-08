
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

    hull.push_back(points[lowest]);

    std::vector<HullAngle> angles;
    for(size_t i=0; i<points.size(); ++i)
    {
        angles.push_back(HullAngle(atan2(points[i][1]-hull[0][1], points[i][0]-hull[0][0]), i));
    }

    std::sort(angles.begin(), angles.end(), HullAngleComparison);

    // TODO: Fill in the hull

    return hull;
}
