#ifndef URDFPARSING_H
#define URDFPARSING_H

#include "Link.h"

namespace akinUtils {

bool loadURDF(akin::Robot& robot, const std::string& filename, const std::string &package_directory, akin::Frame& referenceFrame);
bool loadURDFstring(akin::Robot& robot, const std::string& urdf_string, akin::Frame& referenceFrame);

} // namespace akinUtils

#endif // URDFPARSING_H
