#ifndef GRAPHICSSHADER_H
#define GRAPHICSSHADER_H

#include "IncludeGL.h"

namespace akin {

class GraphicsShader
{
public:

    GraphicsShader(std::string myLabel="shader",
                   verbosity::verbosity_level_t report_level=verbosity::LOG);
    GraphicsShader(std::string filename, GLenum shader_type, std::string myLabel="shader",
                   verbosity::verbosity_level_t report_level=verbosity::LOG);

    GLuint load(std::string filename, GLenum shader_type);

    std::string label;

    GLuint id() const;

    verbosity verb;

protected:

    GLuint _id;
};

} // namespace akin

#endif // GRAPHICSSHADER_H
