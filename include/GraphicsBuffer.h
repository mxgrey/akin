#ifndef GRAPHICSBUFFER_H
#define GRAPHICSBUFFER_H

#include "GlIncludes.h"


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

    verbosity verb;

protected:
    GLuint _VertexShaderId;
    GLuint _FragmentShaderId;
    GLuint _ProgramId;
    GLuint _VaoId;
    GLuint _VboId;
    GLuint _ColorBufferId;

    const static GLchar* _VertexShader;
    const static GLchar* _FragmentShader;

    static GraphicsBuffer* _buffer;

    void makeBuffer(verbosity::verbosity_level_t report_level);

private:

    GraphicsBuffer(bool create);

};

} // namespace akin

#endif // GRAPHICSBUFFER_H
