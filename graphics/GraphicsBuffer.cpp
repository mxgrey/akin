
#include "GraphicsBuffer.h"

using namespace akin;

const GLchar* GraphicsBuffer::_VertexShader =
{
    "#version 400\n"

    "layout (location=0) in vec4 in_Position;\n"
    "layout (location=1) in vec4 in_Color;\n"
    "out vec4 ex_Color;\n"

    "void main(void)\n"
    "{\n"
    "    gl_Position = in_Position;\n"
    "    ex_Color = in_Color;\n"
    "}\n"
};

const GLchar* GraphicsBuffer::_FragmentShader =
{
    "#version 400\n"

    "in vec4 ex_Color;\n"
    "out vec4 out_Color;\n"

    "void main(void)\n"
    "{\n"
    "    out_Color = ex_Color;\n"
    "}\n"
};


GraphicsBuffer* GraphicsBuffer::_buffer = 0;

GraphicsBuffer::GraphicsBuffer(verbosity::verbosity_level_t report_level)
{
    verb.level = report_level;
    if(verb.level == verbosity::INHERIT)
        verb.level = verbosity::LOG;

    makeBuffer(verb.level);
}

GraphicsBuffer::GraphicsBuffer(bool create)
{

}

void GraphicsBuffer::makeBuffer(verbosity::verbosity_level_t report_level)
{
    if(_buffer==0)
    {
        verb.debug() << "Buffer does not exist yet. Creating one!"; verb.end();
        _buffer = new GraphicsBuffer(true);
        _buffer->verb.level = report_level;
        verb.debug() << "Static buffer created"; verb.end();
    }
}

void GraphicsBuffer::Cleanup()
{
    DestroyShaders();
    DestroyVBO();
}

void GraphicsBuffer::CreateVBO()
{
    GLfloat Vertices[] = {
        -0.8f, -0.8f, 0.0f, 1.0f,
         0.0f,  0.8f, 0.0f, 1.0f,
         0.8f, -0.8f, 0.0f, 1.0f
    };

    GLfloat Colors[] = {
        1.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 1.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f, 1.0f
    };

    GLenum ErrorCheckValue = glGetError();

    _buffer->verb.debug() << "Generating vertex arrays"; _buffer->verb.end();
    glGenVertexArrays(1, &(_buffer->_VaoId));
    _buffer->verb.debug() << "Binding vertex array"; _buffer->verb.end();
    glBindVertexArray(_buffer->_VaoId);

    _buffer->verb.debug() << "Generating vertex buffer"; _buffer->verb.end();
    glGenBuffers(1, &(_buffer->_VboId));
    _buffer->verb.debug() << "Binding buffer"; _buffer->verb.end();
    glBindBuffer(GL_ARRAY_BUFFER, _buffer->_VboId);
    _buffer->verb.debug() << "Loading buffer data"; _buffer->verb.end();
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
    _buffer->verb.debug() << "Setting vertex attributes"; _buffer->verb.end();
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
    _buffer->verb.debug() << "Enabling vertex attribute array"; _buffer->verb.end();
    glEnableVertexAttribArray(0);

    _buffer->verb.debug() << "Generating color buffer"; _buffer->verb.end();
    glGenBuffers(1, &(_buffer->_ColorBufferId));
    _buffer->verb.debug() << "Binding buffer"; _buffer->verb.end();
    glBindBuffer(GL_ARRAY_BUFFER, _buffer->_ColorBufferId);
    _buffer->verb.debug() << "Loading buffer data"; _buffer->verb.end();
    glBufferData(GL_ARRAY_BUFFER, sizeof(Colors), Colors, GL_STATIC_DRAW);
    _buffer->verb.debug() << "Setting color (vertex?) attributes"; _buffer->verb.end();
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
    _buffer->verb.debug() << "Enabling color (vertex?) attribute array"; _buffer->verb.end();
    glEnableVertexAttribArray(1);

    ErrorCheckValue = glGetError();
    _buffer->verb.Assert(ErrorCheckValue == GL_NO_ERROR, verbosity::ASSERT_CRITICAL,
                "Error while trying to initialize the buffers",
                ": "+std::string((char*)gluErrorString(ErrorCheckValue)));
}

void GraphicsBuffer::DestroyVBO()
{
    GLenum ErrorCheckValue = glGetError();

    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glDeleteBuffers(1, &(_buffer->_ColorBufferId));
    glDeleteBuffers(1, &(_buffer->_VboId));

    glBindVertexArray(0);
    glDeleteVertexArrays(1, &(_buffer->_VaoId));

    ErrorCheckValue = glGetError();
    _buffer->verb.Assert(ErrorCheckValue != GL_NO_ERROR, verbosity::ASSERT_CASUAL,
                         "Error while trying to destroy buffers",
                         ": "+std::string((char*)gluErrorString(ErrorCheckValue)));
}

void GraphicsBuffer::CreateShaders()
{
    GLenum ErrorCheckValue = glGetError();

    _buffer->verb.debug() << "Creating vertex shader"; _buffer->verb.end();
    _buffer->_VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(_buffer->_VertexShaderId, 1, &_buffer->_VertexShader, NULL);
    glCompileShader(_buffer->_VertexShaderId);

    _buffer->verb.debug() << "Creating fragment shader"; _buffer->verb.end();
    _buffer->_FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(_buffer->_FragmentShaderId, 1, &(_buffer->_FragmentShader), NULL);
    glCompileShader(_buffer->_FragmentShaderId);

    _buffer->verb.debug() << "Creating program"; _buffer->verb.end();
    _buffer->_ProgramId = glCreateProgram();
    _buffer->verb.debug() << "Attaching shaders"; _buffer->verb.end();
        glAttachShader(_buffer->_ProgramId, _buffer->_VertexShaderId);
        glAttachShader(_buffer->_ProgramId, _buffer->_FragmentShaderId);
    _buffer->verb.debug() << "Linking program"; _buffer->verb.end();
    glLinkProgram(_buffer->_ProgramId);
    glUseProgram(_buffer->_ProgramId);

    ErrorCheckValue = glGetError();
    _buffer->verb.Assert(ErrorCheckValue == GL_NO_ERROR, verbosity::ASSERT_CRITICAL,
                "Error while trying to create the shaders",
                ": "+std::string((char*)gluErrorString(ErrorCheckValue)));
}

void GraphicsBuffer::DestroyShaders()
{
    GLenum ErrorCheckValue = glGetError();

    glUseProgram(0);

    glDetachShader(_buffer->_ProgramId, _buffer->_VertexShaderId);
    glDetachShader(_buffer->_ProgramId, _buffer->_FragmentShaderId);

    glDeleteShader(_buffer->_FragmentShaderId);
    glDeleteShader(_buffer->_VertexShaderId);

    glDeleteProgram(_buffer->_ProgramId);

    ErrorCheckValue = glGetError();
    _buffer->verb.Assert(ErrorCheckValue == GL_NO_ERROR, verbosity::ASSERT_CASUAL,
                "Error while trying to destroy the shaders",
                ": "+std::string((char*)gluErrorString(ErrorCheckValue)));
}
