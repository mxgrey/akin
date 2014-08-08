#ifndef AKIN_CONVEXHULL_H
#define AKIN_CONVEXHULL_H

#include <vector>
#include <Eigen/Core>

namespace akin {

std::vector<Eigen::Vector2d> computeConvexHull(std::vector<Eigen::Vector2d>& points);

} // namespace akin

#endif // AKIN_CONVEXHULL_H
