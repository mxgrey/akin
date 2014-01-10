#ifndef GRAPHICSOBJECT_H
#define GRAPHICSOBJECT_H

#include "IncludeGL.h"
#include "Frame.h"

namespace akin {

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
        memset(RGBA, 0, sizeof(RGBA));
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
    
    inline Vertex(const Eigen::Vector3d& vec)
    {
        XYZW[0] = (float)vec[0];
        XYZW[1] = (float)vec[1];
        XYZW[2] = (float)vec[2];
        XYZW[3] = 1.0f;
        memset(RGBA, 0, sizeof(RGBA));
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
    uint index[3];
} Face;

inline Face createFace(uint v1, uint v2, uint v3)
{
    Face newFace;
    newFace.index[0] = v1;
    newFace.index[1] = v2;
    newFace.index[2] = v3;
    return newFace;
}

typedef std::vector<Vertex> VertexArray;
typedef std::vector<Face> FaceArray;

class GraphicsObject : public KinObject
{
public:

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

    float width();
    void width(float new_width);

    float length();
    void length(float new_length);

    float height();
    void height(float new_height);

    void dimensions(float new_width=1, float new_length=1, float new_height=1);

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
