#ifndef GRAPHICSSHADER_H
#define GRAPHICSSHADER_H

#include "IncludeGL.h"
#include <vector>

namespace akin {

class GraphicsShader
{
public:

    GraphicsShader(std::string myLabel="shader",
                   verbosity::verbosity_level_t report_level=verbosity::LOG);
    GraphicsShader(std::string filename, GLenum shader_type, std::string myLabel="shader",
                   verbosity::verbosity_level_t report_level=verbosity::LOG);

    /*!
     * \fn load()
     * \brief Loads a shader program from a file
     * \param filename
     * \param shader_type
     * \return Shader identifier
     */
    GLuint load(std::string filename, GLenum shader_type);

    std::string label;

    GLuint id() const;

    verbosity verb;

protected:

    GLuint _id;
};

typedef std::vector<GraphicsShader> GraphicsShaderArray;

} // namespace akin

#endif // GRAPHICSSHADER_H
