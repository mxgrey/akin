#ifndef AKIN_VISUAL_H
#define AKIN_VISUAL_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

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

double cross2d(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

bool isLeftTurn(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);

bool isRightTurn(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);

bool isInsideConvexHull(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& convexHull, 
                        bool exclude_edge = false);

Eigen::Vector2d closestPointToLineSegment(const Eigen::Vector2d& p, 
                                          const Eigen::Vector2d& s1, const Eigen::Vector2d& s2);

Eigen::Vector2d closestPointOnHull(const Eigen::Vector2d& p, 
                                   const std::vector<Eigen::Vector2d>& convexHull);

class ColorSpec
{
public:
    
    inline ColorSpec(float red=1.0f, float green=1.0f, float blue=1.0f, float alpha=1.0f)
    {
        array[0] = red;
        array[1] = green;
        array[2] = blue;
        array[3] = alpha;
    }
    
    Eigen::Vector4f array;
    
    static inline ColorSpec Red()
    {
        ColorSpec red(1.0f, 0.0f, 0.0f);
        return red;
    }
    
    static inline ColorSpec Green()
    {
        ColorSpec green(0.0f, 1.0f, 0.0f);
        return green;
    }
    
    static inline ColorSpec Blue()
    {
        ColorSpec blue(0.0f, 0.0f, 1.0f);
        return blue;
    }
    
    static inline ColorSpec Black()
    {
        ColorSpec black(0.0f, 0.0f, 0.0f);
        return black;
    }
    
    static inline ColorSpec White()
    {
        ColorSpec white(1.0f, 1.0f, 1.0f);
        return white;
    }
    
    static inline ColorSpec Gray()
    {
        ColorSpec gray(0.6f, 0.6f, 0.6f);
        return gray;
    }
};

typedef std::vector<ColorSpec> ColorArray;

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
        POINT, // Why not just POINTS? Having POINT seems redundant
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
    render_hint_t hint;
    
    Eigen::Vector3d scale;
    Eigen::Isometry3d relative_pose;
    
    ColorArray colors;
    
    std::vector<Eigen::Vector3d> vertices;
    std::vector<ushort> element_indices;
    
    std::string mesh_filename;

    static inline const Geometry& Empty()
    {
        static Geometry emptyGeometry;
        return emptyGeometry;
    }

    static inline const Geometry& Axes()
    {
        static Geometry axesGeometry;
        axesGeometry.type = AXES;
        axesGeometry.scale = Eigen::Vector3d::Ones()*0.2;
        return axesGeometry;
    }
    
    inline void clear()
    {
        type = NONE;
        scale = Eigen::Vector3d::Ones();
        relative_pose = Eigen::Isometry3d::Identity();
        vertices.clear();
        element_indices.clear();
        mesh_filename.clear();
    }
};

typedef std::vector<Geometry> GeometryArray;

// TODO: Make a stream operator for geometries

} // namespace akin

#endif // AKIN_VISUAL_H
