
#include "urdfParsing.h"

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <fstream>
#include <boost/shared_ptr.hpp>

namespace akinUtils {

bool exploreLink(akin::Robot& robot, boost::shared_ptr<urdf::ModelInterface> model,
                 boost::shared_ptr<urdf::Link> link, akin::Link& parentLink);



} // namespace akinUtils

bool akinUtils::loadURDF(akin::Robot &robot, const std::string &filename,
                         akin::Frame& referenceFrame)
{
    std::string xml_model_string;
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    
    if(!xml_file.good())
    {
        std::cerr << "Could not find file '" << filename << "' to parse!'" << std::endl;
        robot.name("invalid");
        return false;
    }
    
    while(xml_file.good())
    {
        std::string line;
        std::getline( xml_file, line );
        xml_model_string += line + "\n";
    }
    
    xml_file.close();
    
    return akinUtils::loadURDFstring(robot, xml_model_string, referenceFrame);
}


bool akinUtils::loadURDFstring(akin::Robot &robot, const std::string &urdf_string,
                               akin::Frame& referenceFrame)
{
    boost::shared_ptr<urdf::ModelInterface> model;
    
    model = urdf::parseURDF( urdf_string );
    
    std::vector< boost::shared_ptr<urdf::Link> > links;
    model->getLinks( links );
    
    boost::shared_ptr<urdf::Link> rootLink = model->root_link_;
    if(!robot.createRootLink(rootLink->name, referenceFrame))
    {
        std::cout << "Root link already existed for the robot '" << robot.name()
                  << "'. Cannot load the URDF into this robot!" << std::endl;
        return false;
    }
    
    robot.name(model->getName());
    
    bool result = akinUtils::exploreLink(robot, model, rootLink, robot.link(0));
    
    return result;
}

bool akinUtils::exploreLink(akin::Robot &robot,
                            boost::shared_ptr<urdf::ModelInterface> model,
                            boost::shared_ptr<urdf::Link> link,
                            akin::Link &parentLink)
{
    bool success = true;
    
    for(size_t i=0; i<link->child_joints.size(); ++i)
    {
        boost::shared_ptr<urdf::Joint> ujoint = link->child_joints[i];
        akin::Transform baseTransform(akin::Translation(
                                          ujoint->parent_to_joint_origin_transform.position.x,
                                          ujoint->parent_to_joint_origin_transform.position.y,
                                          ujoint->parent_to_joint_origin_transform.position.z),
                                      akin::Rotation(
                                          ujoint->parent_to_joint_origin_transform.rotation.w,
                                          ujoint->parent_to_joint_origin_transform.rotation.x,
                                          ujoint->parent_to_joint_origin_transform.rotation.y,
                                          ujoint->parent_to_joint_origin_transform.rotation.z)
                                      );
        akin::Joint::Type jt = akin::Joint::DUMMY;
        if(ujoint->type == urdf::Joint::REVOLUTE)
            jt = akin::Joint::REVOLUTE;
        else if(ujoint->type == urdf::Joint::PRISMATIC)
            jt = akin::Joint::PRISMATIC;

        akin::Axis jointAxis(ujoint->axis.x, ujoint->axis.y, ujoint->axis.z);
        if(jointAxis.norm() == 0)
            jointAxis = akin::Axis::UnitZ();
            
        boost::shared_ptr<urdf::Link> ulink;
        model->getLink(ujoint->child_link_name, ulink);
        
        double min=0;
        double max=0;
        if(ujoint->limits)
        {
            min = ujoint->limits->lower;
            max = ujoint->limits->upper;
        }
        
        int newID = robot.createJointLinkPair(parentLink, ulink->name, ujoint->name,
                                              baseTransform, jointAxis, jt,
                                              min, max);
        if(newID <= 0)
        {
            std::cerr << "Could not create joint/link pair for URDF joint '" << ujoint->name
                      << "' and link '" << ulink->name << "'. Error Code: " << newID;
            return false;
        }
        
        success = success && exploreLink(robot, model, ulink, robot.link(newID));
        if(!success)
            return false;
    }
    
    return success;
}



