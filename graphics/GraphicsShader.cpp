
#include "GraphicsShader.h"

using namespace akin;

GraphicsShader::GraphicsShader(std::string myLabel, verbosity::verbosity_level_t report_level) :
    label(myLabel)
{
    verb.level = report_level;
}

GraphicsShader::GraphicsShader(std::string filename, GLenum shader_type, std::string myLabel,
                               verbosity::verbosity_level_t report_level) :
    label(myLabel),
    _id(0)
{
    verb.level = report_level;

    load(filename, shader_type);
}

GLuint GraphicsShader::id() const { return _id; }

GLuint GraphicsShader::load(std::string filename, GLenum shader_type)
{
    GLuint shader_id = 0;
    FILE* file;
    long file_size = -1;
    char* glsl_source;

    if(verb.Assert(NULL != (file = fopen(filename.c_str(), "rb"))
            && 0 == fseek(file, 0, SEEK_END)
            && -1 != (file_size = ftell(file)),
                verbosity::ASSERT_CASUAL,
                "Could not open '"+filename+"'' to load shader '"+label+"'"))
    {
        rewind(file);

        if(verb.Assert(NULL != (glsl_source = (char*)malloc(file_size+1)),
                       verbosity::ASSERT_CASUAL,
                       "Could not allocate enough bytes for file '"+filename+"' to be loaded for shader '"+label+"'"))
        {
            if(verb.Assert(file_size == (long)fread(glsl_source, sizeof(char), file_size, file),
                           verbosity::ASSERT_CASUAL,
                           "Could not read file '"+filename+"' to laod shader '"+label+"'"))
            {
                glsl_source[file_size] = '\0';

                if(verb.Assert(0 != (shader_id = glCreateShader(shader_type)),
                               verbosity::ASSERT_CASUAL,
                               "Could not create a shader for '"+label+"' from file '"+filename+"'"))
                {
                    glShaderSource(shader_id, 1, (const GLchar**)&glsl_source, NULL);
                    glCompileShader(shader_id);

                    GLenum ErrorCheckValue = glGetError();
                    verb.Assert(ErrorCheckValue == GL_NO_ERROR,
                                verbosity::ASSERT_CASUAL,
                                "Could not compile shader for '"+label+"' from the file '"+filename+"'",
                                ": "+std::string((char*)gluErrorString(ErrorCheckValue)));
                }
            }
        }

        fclose(file);
    }

    _id = shader_id;
    return shader_id;
}
