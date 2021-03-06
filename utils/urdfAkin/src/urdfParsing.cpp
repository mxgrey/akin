
#include "urdfAkin/urdfParsing.h"

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <fstream>
#include <boost/shared_ptr.hpp>

namespace urdfAkin {

bool exploreLink(akin::Robot& robot, boost::shared_ptr<urdf::ModelInterface> model,
                 boost::shared_ptr<urdf::Link> link, akin::Link& parentLink);


bool linkProperties(akin::Link& link, boost::shared_ptr<urdf::Link> ulink);


} // namespace urdfAkin

bool urdfAkin::loadURDF(akin::Robot &robot, const std::string &filename,
                         const std::string& package_directory)
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

    robot.robotPackageDirectory = package_directory;
    
    return urdfAkin::loadURDFstring(robot, xml_model_string);
}


bool urdfAkin::loadURDFstring(akin::Robot &robot, const std::string &urdf_string)
{
    boost::shared_ptr<urdf::ModelInterface> model;
    
    model = urdf::parseURDF( urdf_string );
    
    boost::shared_ptr<urdf::Link> rootLink = model->root_link_;
    if(robot.numLinks() > 1)
    {
        std::cout << "Links already existed for the robot '" << robot.name()
                  << "'. Cannot load the URDF into this robot!" << std::endl;
        return false;
    }
    robot.link(0).name(model->root_link_->name);
    linkProperties(robot.link(0), model->root_link_);
    
    robot.name(model->getName());
    
    bool result = urdfAkin::exploreLink(robot, model, rootLink, robot.link(0));

    robot.setDefaultRobotConstraints();

    return result;
}

bool urdfAkin::exploreLink(akin::Robot &robot,
                            boost::shared_ptr<urdf::ModelInterface> model,
                            boost::shared_ptr<urdf::Link> link,
                            akin::Link &parentLink)
{
    bool success = true;
    
    for(size_t i=0; i<link->child_joints.size(); ++i)
    {
        boost::shared_ptr<urdf::Joint> ujoint = link->child_joints[i];
        akin::ProtectedJointProperties joint_prop;
        akin::DofProperties dof_prop;
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
        joint_prop._baseTransform = baseTransform;
        joint_prop._type = akin::Joint::FIXED;
        if(ujoint->type == urdf::Joint::REVOLUTE)
            joint_prop._type = akin::Joint::REVOLUTE;
        else if(ujoint->type == urdf::Joint::PRISMATIC)
            joint_prop._type = akin::Joint::PRISMATIC;
        else if(ujoint->type == urdf::Joint::FLOATING)
            joint_prop._type = akin::Joint::FLOATING;
        else if(ujoint->type == urdf::Joint::FIXED || ujoint->type == urdf::Joint::CONTINUOUS)
            joint_prop._type = akin::Joint::FIXED;

        joint_prop._axis = akin::Vec3(ujoint->axis.x, ujoint->axis.y, ujoint->axis.z);
        if(joint_prop._axis.norm() == 0)
            joint_prop._axis = akin::Vec3::UnitZ();

        boost::shared_ptr<urdf::Link> ulink;
        model->getLink(ujoint->child_link_name, ulink);

        if(ujoint->limits)
        {
            dof_prop._minValue = ujoint->limits->lower;
            dof_prop._maxValue = ujoint->limits->upper;
            dof_prop._maxSpeed = ujoint->limits->velocity;
            dof_prop._maxEffort = ujoint->limits->effort;
        }

        joint_prop._name = ujoint->name;

        int newID = robot.createJointLinkPair(parentLink, ulink->name, joint_prop, dof_prop);
        if(newID <= 0)
        {
            std::cerr << "Could not create joint/link pair for URDF joint '" << ujoint->name
                      << "' and link '" << ulink->name << "'. Error Code: " << newID;
            return false;
        }

        success = success && linkProperties(robot.link(newID), ulink);
        
        success = success && exploreLink(robot, model, ulink, robot.link(newID));
        if(!success)
            return false;
    }
    
    return success;
}

bool urdfAkin::linkProperties(akin::Link &link, boost::shared_ptr<urdf::Link> ulink)
{
    akin::Geometry visual;
    if(ulink->visual)
    {
        visual.hint = akin::Geometry::STATIC;
        if(ulink->visual->geometry->type == urdf::Geometry::MESH)
        {

            urdf::Mesh* mesh =
                    dynamic_cast<urdf::Mesh*>(ulink->visual->geometry.get());
            if(mesh)
            {
                visual.type = akin::Geometry::MESH_FILE;
                std::string mesh_filename = mesh->filename;
                size_t num = mesh_filename.find("package://");
                if(num < std::string::npos)
                    mesh_filename.erase(num, num+9);

                mesh_filename = link.robot().robotPackageDirectory + mesh_filename;

                visual.mesh_filename = mesh_filename;
            }
            else
            {
                std::cout << "The URDF for link named '" << link.name() << "' claims to be a mesh, but "
                          << "cannot be dynamically cast as one!" << std::endl;
                visual.type = akin::Geometry::NONE;
            }
        }
    }
    else
    {
        std::cout << "No visual for the link named '" << link.name() << "'" << std::endl;
    }
    link.addVisual(visual);
    
    if(ulink->inertial)
    {
        akin::StandardInertiaParameters inertia;
        inertia.mass = ulink->inertial->mass;
        inertia.centerOfMass = akin::Translation(ulink->inertial->origin.position.x,
                                                 ulink->inertial->origin.position.y,
                                                 ulink->inertial->origin.position.z);
        inertia.tensor(0,0) = ulink->inertial->ixx;
        inertia.tensor(0,1) = ulink->inertial->ixy;
        inertia.tensor(0,2) = ulink->inertial->ixz;
        inertia.tensor(1,1) = ulink->inertial->iyy;
        inertia.tensor(1,2) = ulink->inertial->iyz;
        inertia.tensor(2,2) = ulink->inertial->izz;
        inertia.mirrorTheCurrentTensorValues();

        link.inertia(inertia);
    }

    return true;
}

