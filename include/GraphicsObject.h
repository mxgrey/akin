#ifndef GRAPHICSOBJECT_H
#define GRAPHICSOBJECT_H

#include "IncludeGL.h"
#include "AkinIncludes.h"

namespace akin {

class GraphicsBuffer;

typedef struct
{
    float XYZW[4];
    float RGBA[4];
} CheapVertex;

class Vertex
{
public:

    inline Vertex()
    {
        memset(XYZW, 0, sizeof(XYZW));
        XYZW[3] = 1.0f;
        memset(RGBA, 0, sizeof(RGBA));
        RGBA[3] = 1.0f;
    }

    inline Vertex(const Vertex& otherVertex)
    {
        memcpy(XYZW, otherVertex.XYZW, sizeof(XYZW));
        memcpy(RGBA, otherVertex.RGBA, sizeof(RGBA));
    }

    inline Vertex(const Eigen::Vector3d& vec, const float (&rgba)[4])
    {
        XYZW[0] = (float)vec[0];
        XYZW[1] = (float)vec[1];
        XYZW[2] = (float)vec[2];
        XYZW[3] = 1.0f;
        memcpy(RGBA, rgba, sizeof(RGBA));
    }
    
    inline Vertex(float x, float y, float z)
    {
        XYZW[0] = x; XYZW[1] = y; XYZW[2] = z; XYZW[3] = 1.0f;
        memset(RGBA, 0, sizeof(RGBA)); RGBA[3] = 1.0f;
    }
    
    inline Vertex(float x, float y, float z, float r, float g, float b, float a)
    {
        XYZW[0] = x; XYZW[1] = y; XYZW[2] = z; XYZW[3] = 1.0f;
        RGBA[0] = r; RGBA[1] = g; RGBA[2] = b; RGBA[3] = 1.0f;
    }
    
    inline Vertex(const float (&xyz)[3], const float (&rgba)[4])
    {
        for(size_t i=0; i<3; ++i)
            XYZW[i] = xyz[i];
        XYZW[3] = 1.0f;
        for(size_t i=0; i<4; ++i)
            RGBA[i] = rgba[i];
    }
    
    inline Vertex(const Eigen::Vector3d& vec)
    {
        XYZW[0] = (float)vec[0];
        XYZW[1] = (float)vec[1];
        XYZW[2] = (float)vec[2];
        XYZW[3] = 1.0f;
        memset(RGBA, 0, sizeof(RGBA)); RGBA[3] = 1.0f;
    }

    inline Vertex& operator=(const Translation& vec)
    {
        XYZW[0] = (float)vec[0];
        XYZW[1] = (float)vec[1];
        XYZW[2] = (float)vec[2];
        XYZW[3] = 1.0f;

        return *this;
    }

    float XYZW[4];
    float RGBA[4];
};

typedef struct
{
    GLushort index[3];
} Face;

inline Face createFace(GLushort v1, GLushort v2, GLushort v3)
{
    Face newFace;
    newFace.index[0] = v1;
    newFace.index[1] = v2;
    newFace.index[2] = v3;
    return newFace;
}

typedef std::vector<Vertex> VertexArray;
typedef std::vector<Face> FaceArray;

inline GLuint numFaceElements(const FaceArray& array)
{
    return array.size()*3;
}

class GraphicsObject : public KinObject
{
public:

    friend class GraphicsBuffer;

    KinCustomMacro( GraphicsObject )

    GraphicsObject(Frame& referenceFrame = Frame::World(),
                   std::string graphicName="graphic",
                   verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    GraphicsObject(const VertexArray& graphicVertexArray, const FaceArray& graphicFaceArray,
                   Frame& referenceFrame = Frame::World(),
                   std::string graphicName="graphic",
                   verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    ~GraphicsObject();

    uint addVertex(const Vertex& new_vertex);
    bool removeVertex(uint index);
    uint addFace(const Face& new_face);
    bool removeFace(uint index);

    const VertexArray& respectToWorld();
    const VertexArray& respectToCamera(Frame& cameraFrame);
    VertexArray withRespectTo(Frame& someFrame);

protected:

    GLuint _vertexBufferAddress;
    GLuint _vertexArrayAddress;
    GLuint _faceBufferAddress;
    GLuint _elementSize;

    VertexArray _vertices;
    FaceArray _faces;

    VertexArray _respectToWorld;
    VertexArray _respectToCamera;

    void _updateWorld();
    void _updateCamera(Frame& cameraFrame);
};

typedef std::vector<GraphicsObject> GraphicsArray;
typedef std::vector<GraphicsObject*> GraphicsPointerArray;

class Box : public GraphicsObject
{
public:
    Box(Frame& referenceFrame = Frame::World(),
         std::string boxName="Box",
         verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    Box(float width, float length, float height,
         Frame& referenceFrame = Frame::World(),
         std::string boxName="Box",
         verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    typedef enum {
        FRONT=0,
        BACK,
        LEFT,
        RIGHT,
        TOP,
        BOTTOM
    } box_side_t;

    float width();
    void width(float new_width);

    float length();
    void length(float new_length);

    float height();
    void height(float new_height);

    void dimensions(float new_width=1, float new_length=1, float new_height=1);
    void color(float red, float green, float blue, float alpha);
    void color(const float (&rgba)[4]);
    void sideColor(box_side_t side, float red, float green, float blue, float alpha);
    void sideColor(box_side_t side, const float (&rgba)[4]);

protected:
    float _width;
    float _height;
    float _length;
};

inline akin::Vertex operator*(const akin::Transform& tf, const akin::Vertex& vertex)
{
    return Vertex(tf * Eigen::Vector3d(vertex.XYZW[0],
                                       vertex.XYZW[1],
                                       vertex.XYZW[2]),
            vertex.RGBA);
}

inline akin::VertexArray operator*(const akin::Transform& tf, const akin::VertexArray& array)
{
    akin::VertexArray newArray(array);
    for(size_t i=0; i<array.size(); i++)
    {
        newArray[i] = tf * array[i];
    }
    return newArray;
}

} // namespace akin


inline akin::Vertex operator*(const akin::Transform& tf, const akin::Vertex& vertex)
{
    return akin::operator *(tf, vertex);
}

inline akin::VertexArray operator*(const akin::Transform& tf, const akin::VertexArray& array)
{
    return akin::operator *(tf, array);
}


#endif // GRAPHICSOBJECT_H
