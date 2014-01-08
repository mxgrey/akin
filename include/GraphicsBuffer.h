#ifndef GRAPHICSBUFFER_H
#define GRAPHICSBUFFER_H

#include "IncludeGL.h"
#include "GraphicsObject.h"

namespace akin {


typedef struct
{
    float m[16];
} FloatMatrix;

inline FloatMatrix convertToFloat(const Eigen::Isometry3d& other)
{
    FloatMatrix M;
    for(size_t i=0; i<4; ++i)
        for(size_t j=0; j<4; ++j)
            M.m[4*i+j] = other(j,i);

    return M;
}

inline FloatMatrix FloatIdentity()
{
    return convertToFloat(Eigen::Isometry3d::Identity());
}

typedef std::vector<GLuint> IdArray;

class GraphicsBuffer
{
public:

    GraphicsBuffer(verbosity::verbosity_level_t report_level = verbosity::LOG);

    static void Cleanup();
    static void CreateVBO();
    static void DestroyVBO();
    static void CreateShaders();
    static void DestroyShaders();
    
    static uint32_t getActiveIndexBuffer();
    static void setActiveIndexBuffer(uint32_t active);
    
    static void drawElements();

    static uint addGraphic(GraphicsObject& object);
    static void removeGraphic(GraphicsObject& object);
    static void removeGraphic(uint index);



    verbosity verb;

protected:
    GLuint _VertexShaderId;
    GLuint _FragmentShaderId;
    GLuint _ProgramId;
    GLuint _VaoId;
    GLuint _VboId;
    GLuint _IndexBufferId[2];
    GLuint _ActiveIndexBuffer;

    IdArray _BufferIds;
    IdArray _ShaderIds;

    GraphicsArray* _graphics;
    IdArray _graphicOffset;

    VertexArray _globalVertexArray;
    FaceArray _globalFaceArray;


    FloatMatrix _ProjectionMatrix;
    FloatMatrix _ViewMatrix;
    FloatMatrix _ModelMatrix;

    const static GLchar* _VertexShader;
    const static GLchar* _FragmentShader;

    static GraphicsBuffer* _buffer;

    void makeBuffer(verbosity::verbosity_level_t report_level);

private:

    GraphicsBuffer(bool create);

};


} // namespace akin


#endif // GRAPHICSBUFFER_H
