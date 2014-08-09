#ifndef AKIN_CONVEXHULL_H
#define AKIN_CONVEXHULL_H

#include <vector>
#include <Eigen/Core>

namespace akin {

std::vector<Eigen::Vector2d> computeConvexHull(std::vector<Eigen::Vector2d>& points);


typedef enum {
    
    PARALLEL=0,
    INTERSECTING,
    BEYOND_ENDPOINTS
    
} intersection_t;

Eigen::Vector2d computeCentroid(const std::vector<Eigen::Vector2d>& convexHull);

intersection_t computeIntersection(Eigen::Vector2d& intersection, 
                                   const Eigen::Vector2d& a1,
                                   const Eigen::Vector2d& a2,
                                   const Eigen::Vector2d& b1,
                                   const Eigen::Vector2d& b2);

} // namespace akin

#endif // AKIN_CONVEXHULL_H
