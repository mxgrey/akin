#ifndef URDFPARSING_H
#define URDFPARSING_H

#include "akin/Link.h"

namespace akinUtils {

bool loadURDF(akin::Robot& robot, 
              const std::string& filename, 
              const std::string &package_directory, 
              akin::Frame& referenceFrame = akin::Frame::World());

bool loadURDFstring(akin::Robot& robot, 
                    const std::string& urdf_string, 
                    akin::Frame& referenceFrame = akin::Frame::World());

} // namespace akinUtils

#endif // URDFPARSING_H
