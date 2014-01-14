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

class ColorSpec
{
public:

    float RGBA[4];

    inline ColorSpec()
    {
        memset(RGBA, 0, sizeof(RGBA));
    }

    inline ColorSpec(float r, float g, float b, float a=1.0f)
    {
        set(r, g, b, a);
    }

    inline void set(float r, float g, float b, float a=1.0f)
    {
        RGBA[0] = r; RGBA[1] = g; RGBA[2] = b; RGBA[3] = a;
    }

    inline static ColorSpec black(float alpha=1.0f)
    {
        ColorSpec result; memset(result.RGBA, 0, sizeof(result.RGBA));
        result.RGBA[3] = alpha;
        return result;
    }

    inline static ColorSpec white(float alpha=1.0f)
    {
        ColorSpec result;
        for(size_t i=0; i<3; ++i)
            result.RGBA[i] = 1.0f;
        result.RGBA[3] = alpha;
        return result;
    }

    inline static ColorSpec red(float alpha=1.0f)
    {
        ColorSpec result; memset(result.RGBA, 0, sizeof(result.RGBA));
        result.RGBA[0] = 1.0f;
        result.RGBA[3] = alpha;
        return result;
    }

    inline static ColorSpec green(float alpha=1.0f)
    {
        ColorSpec result; memset(result.RGBA, 0, sizeof(result.RGBA));
        result.RGBA[1] = 1.0f;
        result.RGBA[3] = alpha;
        return result;
    }

    inline static ColorSpec blue(float alpha=1.0f)
    {
        ColorSpec result; memset(result.RGBA, 0, sizeof(result.RGBA));
        result.RGBA[2] = 1.0f;
        result.RGBA[3] = alpha;
        return result;
    }

    inline ColorSpec(const float (&rgba)[4])
    {
        memcpy(RGBA, rgba, sizeof(RGBA));
    }
};

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

    inline Vertex(const Eigen::Vector3d& vec, const ColorSpec& rgba)
    {
        XYZW[0] = (float)vec[0];
        XYZW[1] = (float)vec[1];
        XYZW[2] = (float)vec[2];
        XYZW[3] = 1.0f;
        setColor(rgba);
    }
    
    inline Vertex(float x, float y, float z)
    {
        XYZW[0] = x; XYZW[1] = y; XYZW[2] = z; XYZW[3] = 1.0f;
        memset(RGBA, 0, sizeof(RGBA)); RGBA[3] = 1.0f;
    }
    
    inline Vertex(float x, float y, float z, float r, float g, float b, float a)
    {
        XYZW[0] = x; XYZW[1] = y; XYZW[2] = z; XYZW[3] = 1.0f;
        RGBA[0] = r; RGBA[1] = g; RGBA[2] = b; RGBA[3] = a;
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

    inline void setColor(const ColorSpec& rgba)
    {
        memcpy(RGBA, rgba.RGBA, sizeof(RGBA));
    }

    float XYZW[4];
    float RGBA[4];
};

typedef std::vector<Vertex> VertexArray;

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

typedef std::vector<Face> FaceArray;

inline GLuint numFaceElements(const FaceArray& array)
{
    return array.size()*3;
}

typedef struct
{
    GLushort index[2];
} LineElem;

typedef std::vector<LineElem> LineElemArray;

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

    inline void showFilled(bool show) { _showFilled = show; }
    bool showingFilled() { return _showFilled; }

    void showOutline(bool show) { _showOutline = show; }
    bool showingOutline() { return _showOutline; }
    void lineWidth(GLfloat width) { _lineWidth = width; }
    GLfloat lineWidth() { return _lineWidth; }

    uint addVertex(const Vertex& new_vertex);
    bool removeVertex(uint index);
    uint addFace(const Face& new_face);
    bool removeFace(uint index);

    const VertexArray& respectToWorld();
    const VertexArray& respectToCamera(Frame& cameraFrame);
    VertexArray withRespectTo(Frame& someFrame);

protected:

    bool _showFilled;
    bool _showOutline;
    GLfloat _lineWidth;

    GLuint _vertexBufferAddress;
    GLuint _vertexArrayAddress;
    GLuint _faceBufferAddress;
    GLuint _faceElementSize;

    GLuint _outlineVertexBufferAddress;
    GLuint _outlineVertexArrayAddress;
    GLuint _outlineIndexBufferAddress;
    GLuint _outlineElementSize;

    VertexArray _vertices;
    FaceArray _faces;
    LineElemArray _outline;

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
    void color(float red, float green, float blue, float alpha=1.0f);
    void color(const ColorSpec& rgba);
    void sideColor(box_side_t side, float red, float green, float blue, float alpha=1.0f);
    void sideColor(box_side_t side, const ColorSpec& rgba);
    // TODO: Consider implementing vertex coloring options
//    void vertexColor(box_side_t front_back, box_side_t left_right, box_side_t top_bottom,
//                     float red, float green, float blue, float alpha=1.0f);
//    void vertexColor(box_side_t front_back, box_side_t left_right, box_side_t top_bottom,
//                     const ColorSpec& rgba);

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
