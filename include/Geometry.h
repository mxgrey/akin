#ifndef VISUAL_H
#define VISUAL_H

#include <eigen3/Eigen/Core>
#include <string>
#include <vector>

namespace akin {

class Geometry
{
public:
    
    typedef enum {
        
        STATIC=0,
        DYNAMIC,
        
        MAX_RENDER_HINT
        
    } render_hint_t;
    
    typedef enum {
        
        NONE=0,
        POINT,
        POINTS,
        SPHERE,
        BOX,
        CYLINDER,
        CAPSULE,
        LINE,
        AXES,
        ARROW,
        BI_ARROW,
        MESH_FILE,
        TRI_MESH,
        QUAD_MESH,
        
        MAX_GEOMETRY_TYPE
        
    } geometry_t;
    
    inline Geometry() { clear(); }
    
    geometry_t type;
    
    Eigen::Vector3d scale;
    Eigen::Isometry3d relative_pose;
    
    std::vector<Eigen::Vector3d> vertices;
    std::vector<ushort> element_indices;
    
    std::string mesh_filename;
    
    inline void clear()
    {
        type = POINT;
        scale = Eigen::Vector3d::Ones();
        relative_pose = Eigen::Isometry3d::Identity();
        vertices.clear();
        element_indices.clear();
        mesh_filename.clear();
    }
};

typedef std::vector<Geometry> GeometryArray;

} // namespace akin

#endif // VISUAL_H
