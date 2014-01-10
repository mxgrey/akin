
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

void GraphicsBuffer::displayGraphic(GraphicsObject &object)
{
    _buffer->_graphics.push_back(&object);
}

void GraphicsBuffer::unstageGraphic(GraphicsObject &object)
{
    for(size_t i=0; i<_buffer->_graphics.size(); ++i)
    {
        if(_buffer->_graphics[i] == &object)
        {
            _buffer->_graphics.erase(_buffer->_graphics.begin()+i);
            break;
        }
    }
}

GraphicsBuffer::GraphicsBuffer(verbosity::verbosity_level_t report_level)
{
    verb.level = report_level;
    if(verb.level == verbosity::INHERIT)
        verb.level = verbosity::LOG;

    _makeBuffer(verb.level);
}

GraphicsBuffer::GraphicsBuffer(bool create) :
    _ProjectionMatrix(FloatIdentity()),
    _ViewMatrix(FloatIdentity()),
    _ModelMatrix(FloatIdentity()),
    _ActiveIndexBuffer(0)
{
    Box dummyBox(Frame::World(), "dummy", verbosity::DEBUG);
    _graphics.push_back(&dummyBox);
}

uint32_t GraphicsBuffer::getActiveIndexBuffer() { return _buffer->_ActiveIndexBuffer; }
void GraphicsBuffer::setActiveIndexBuffer(uint32_t active)
{
    _buffer->_ActiveIndexBuffer = active;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer->_IndexBufferId[_buffer->_ActiveIndexBuffer]);
}

void GraphicsBuffer::drawElements()
{
    if(_buffer->_ActiveIndexBuffer==0)
        glDrawElements(GL_TRIANGLES, 48, GL_UNSIGNED_BYTE, NULL);
    else
        glDrawElements(GL_TRIANGLE_FAN, 36, GL_UNSIGNED_BYTE, NULL);
}

void GraphicsBuffer::_makeBuffer(verbosity::verbosity_level_t report_level)
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
//    GLfloat Vertices[] = {
//        -0.8f, -0.8f, 0.0f, 1.0f,
//         0.0f,  0.8f, 0.0f, 1.0f,
//         0.8f, -0.8f, 0.0f, 1.0f
//    };
    
    CheapVertex Vertices[] =
    {
        { {  0.0f, 0.0f, 0.0f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f } },
        
        { { -0.2f, 0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
        { {  0.2f, 0.8f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
        { {  0.0f, 0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
        { {  0.0f, 1.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },
        
        { { -0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
        { {  0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
        { {  0.0f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
        { {  0.0f, -1.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },
        
        { { -0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
        { { -0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
        { { -0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
        { { -1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },
        
        { { 0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
        { { 0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
        { { 0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
        { { 1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } }
    };
    
    GLubyte Indices[] =
    {
        0, 1, 3,
        0, 3, 2,
        3, 1, 4,
        3, 4, 2,
        
        0, 5, 7,
        0, 7, 6,
        7, 5, 8,
        7, 8, 6,
        
        0, 9, 11,
        0, 11, 10,
        11, 9, 12,
        11, 12, 10,
        
        0, 13, 15,
        0, 15, 14,
        15, 13, 16,
        15, 16, 14
    };
    
    GLubyte AlternateIndices[] =
    {
        3, 4, 16,
        3, 15, 16,
        15, 16, 8,
        15, 7, 8,
        7, 8, 12,
        7, 11, 12,
        11, 12, 4,
        11, 3, 4,
        
        0, 11, 3,
        0, 3, 15,
        0, 15, 7,
        0, 7, 11
    };

    GLenum ErrorCheckValue = glGetError();
    const size_t BufferSize = sizeof(Vertices);
    const size_t VertexSize = sizeof(Vertices[0]);
    const size_t RgbaOffset = sizeof(Vertices[0].XYZW);

    glGenBuffers(1, &(_buffer->_VboId));
    
    glGenVertexArrays(1, &(_buffer->_VaoId));
    glBindVertexArray(_buffer->_VaoId);
    
    glBindBuffer(GL_ARRAY_BUFFER, _buffer->_VboId);
    glBufferData(GL_ARRAY_BUFFER, BufferSize, Vertices, GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, VertexSize, 0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, VertexSize, (GLvoid*)RgbaOffset);
    
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    
    glGenBuffers(2, _buffer->_IndexBufferId);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer->_IndexBufferId[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices), Indices, GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer->_IndexBufferId[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(AlternateIndices), AlternateIndices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer->_IndexBufferId[0]);

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

    glDeleteBuffers(1, &(_buffer->_VboId));
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDeleteBuffers(2, _buffer->_IndexBufferId);

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
