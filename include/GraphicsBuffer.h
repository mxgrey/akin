#ifndef GRAPHICSBUFFER_H
#define GRAPHICSBUFFER_H

#include "IncludeGraphics.h"


namespace akin {

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

    verbosity verb;

protected:
    GLuint _VertexShaderId;
    GLuint _FragmentShaderId;
    GLuint _ProgramId;
    GLuint _VaoId;
    GLuint _VboId;
    GLuint _IndexBufferId[2];
    GLuint _ActiveIndexBuffer;

    const static GLchar* _VertexShader;
    const static GLchar* _FragmentShader;

    static GraphicsBuffer* _buffer;

    void makeBuffer(verbosity::verbosity_level_t report_level);

private:

    GraphicsBuffer(bool create);

};

typedef struct
{
    float XYZW[4];
    float RGBA[4];
} Vertex;

} // namespace akin

#endif // GRAPHICSBUFFER_H
