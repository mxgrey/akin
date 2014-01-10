
#include "GraphicsBuffer.h"

using namespace akin;

//const GLchar* GraphicsBuffer::_VertexShader =
//{
//    "#version 400\n"

//    "layout (location=0) in vec4 in_Position;\n"
//    "layout (location=1) in vec4 in_Color;\n"
//    "out vec4 ex_Color;\n"

//    "void main(void)\n"
//    "{\n"
//    "    gl_Position = in_Position;\n"
//    "    ex_Color = in_Color;\n"
//    "}\n"
//};

//const GLchar* GraphicsBuffer::_FragmentShader =
//{
//    "#version 400\n"

//    "in vec4 ex_Color;\n"
//    "out vec4 out_Color;\n"

//    "void main(void)\n"
//    "{\n"
//    "    out_Color = ex_Color;\n"
//    "}\n"
//};


GraphicsBuffer* GraphicsBuffer::_buffer = 0;

void GraphicsBuffer::displayGraphic(GraphicsObject &object)
{
    for(size_t i=0; i<_buffer->_graphics.size(); ++i)
        if(_buffer->_graphics[i] == &object)
            return;

    _buffer->verb.debug() << "Generating vertex buffer for GraphicsObject '" << object.name() << "'"; _buffer->verb.end();
    glGenBuffersARB( 1, &(object._vertexBufferAddress) );
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, object._vertexBufferAddress );
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, object._vertices.size()*sizeof(Vertex),
                     &(object._vertices), GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error passing vertex data to graphics memory");

    _buffer->verb.debug() << "Generating index buffer for GraphicsObject '" << object.name() << "'"; _buffer->verb.end();
    glGenBuffersARB( 1, &(object._faceBufferAddress) );
    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, object._faceBufferAddress );
    glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB, object._faces.size()*sizeof(Face),
                     &(object._faces), GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error passing face data to graphics memory");

    glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );
    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, 0 );
    CheckGLError(_buffer->verb, "Error reseting the buffer binding");

    _buffer->verb.debug() << "Adding GraphicsObject '" << object.name() << "' to the render list."; _buffer->verb.end();
    _buffer->_graphics.push_back(&object);

    _buffer->verb.debug() << "Finished adding Graphics Object '" << object.name() << "' to the display."; _buffer->verb.end();
}

void GraphicsBuffer::unstageGraphic(GraphicsObject &object)
{
    for(size_t i=0; i<_buffer->_graphics.size(); ++i)
    {
        if(_buffer->_graphics[i] == &object)
        {
            _buffer->_graphics.erase(_buffer->_graphics.begin()+i);
            --i;

            glDeleteBuffersARB( 1, &(object._vertexBufferAddress) );
            glDeleteBuffersARB( 1, &(object._faceBufferAddress) );

            break;
        }
    }
}

uint GraphicsBuffer::storeGraphic(const GraphicsObject &object)
{
    GraphicsObject* copiedGraphic = new GraphicsObject(object);
    _buffer->_storedGraphics[_buffer->_nextStorageIndex] = copiedGraphic;
    ++_buffer->_nextStorageIndex;

    displayGraphic(*copiedGraphic);

    return _buffer->_nextStorageIndex-1;
}

GraphicsObject& GraphicsBuffer::retrieveGraphic(uint graphicId)
{
    GraphicsPointerMap::iterator g = _buffer->_storedGraphics.find(graphicId);
    if( _buffer->verb.Assert( g != _buffer->_storedGraphics.end(),
                              verbosity::ASSERT_CASUAL,
                              "Trying to retrieve non-existent (or deleted) graphic") )
        return *(g->second);

    return _buffer->_emptyGraphic;
}

void GraphicsBuffer::deleteGrahpic(uint graphicId)
{
    GraphicsPointerMap::iterator g = _buffer->_storedGraphics.find(graphicId);
    if( _buffer->verb.Assert( g != _buffer->_storedGraphics.end(),
                              verbosity::ASSERT_CASUAL,
                              "Trying to delete non-existent (or deleted) graphic") )
        _buffer->_storedGraphics.erase(g);
}

void GraphicsBuffer::displayStoredGraphic(uint graphicId)
{
    displayGraphic(retrieveGraphic(graphicId));
}

void GraphicsBuffer::unstageStoredGraphic(uint graphicId)
{
    unstageGraphic(retrieveGraphic(graphicId));
}

void GraphicsBuffer::setProjectionMatrix(float field_of_view, float aspect_ratio, float near_plane, float far_plane)
{
    _buffer->_ProjectionMatrix = FloatZeros();

    const float
            y_scale = 1.0f/tan(field_of_view*M_PI/180 / 2),
            x_scale = y_scale / aspect_ratio,
            frustrum_length = far_plane - near_plane;

    _buffer->_ProjectionMatrix.m[0] = x_scale;
    _buffer->_ProjectionMatrix.m[5] = y_scale;
    _buffer->_ProjectionMatrix.m[10] = -((far_plane+near_plane)/frustrum_length);
    _buffer->_ProjectionMatrix.m[11] = -1;
    _buffer->_ProjectionMatrix.m[14] = -((2*near_plane*far_plane)/frustrum_length);

    // TODO: Remove this when done debugging
    _buffer->_ProjectionMatrix = FloatIdentity();

    glUseProgram(_buffer->_shaderProgramId);
    glUniformMatrix4fv(_buffer->_ProjectionMatrixId, 1, GL_FALSE, _buffer->_ProjectionMatrix.m);
    glUseProgram(0);
}

GraphicsBuffer::GraphicsBuffer(verbosity::verbosity_level_t report_level)
{
    verb.level = report_level;
    if(verb.level == verbosity::INHERIT)
        verb.level = verbosity::LOG;

    _makeBuffer(verb.level);
}

GraphicsBuffer::GraphicsBuffer(bool create, verbosity::verbosity_level_t report_level) :
    _ProjectionMatrix(FloatIdentity()),
    _ViewMatrix(FloatIdentity()),
    _ModelMatrix(FloatIdentity()),
    _emptyGraphic(Frame::World(), "empty_graphic"),
    _nextStorageIndex(0)
{
    verb.level = report_level;
}

void GraphicsBuffer::drawElements()
{
//    CheckGLError(_buffer->verb, "About to draw elements");
    glUseProgram(_buffer->_shaderProgramId);
//    CheckGLError(_buffer->verb, "Could not use shader program");

    glUniformMatrix4fvARB(_buffer->_ViewMatrixId, 1, GL_FALSE, _buffer->_ViewMatrix.m);
    glUniformMatrix4fvARB(_buffer->_ProjectionMatrixId, 1, GL_FALSE, _buffer->_ProjectionMatrix.m);

    for(size_t i=0; i<_buffer->_graphics.size(); i++)
    {
        glUniformMatrix4fvARB(_buffer->_ModelMatrixId, 1, GL_FALSE, _buffer->_ModelMatrix.m);

        GraphicsObject& graphic = *(_buffer->_graphics[i]);
        glBindBufferARB( GL_ARRAY_BUFFER_ARB, graphic._vertexBufferAddress);
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, graphic._faceBufferAddress);

        // TODO: Check if the next two lines can live outside of the for loop
        glEnableClientState( GL_VERTEX_ARRAY );
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glDrawElements( GL_TRIANGLES, graphic._faces.size(), GL_UNSIGNED_INT, 0 );

        // TODO: check if the following lines can live otuside of the for loop
        glDisableClientState( GL_VERTEX_ARRAY );
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
    }

    glUseProgram(0);
}

void GraphicsBuffer::_makeBuffer(verbosity::verbosity_level_t report_level)
{
    if(_buffer==0)
    {
        verb.debug() << "Buffer does not exist yet. Creating one!"; verb.end();
        _buffer = new GraphicsBuffer(true, report_level);
        verb.debug() << "Static buffer created"; verb.end();

        CreateShaders();

        Box dummyBox(Frame::World(), "dummy", verbosity::DEBUG);
        storeGraphic(dummyBox);
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
    
//    CheapVertex Vertices[] =
//    {
//        { {  0.0f, 0.0f, 0.0f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f } },
        
//        { { -0.2f, 0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
//        { {  0.2f, 0.8f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
//        { {  0.0f, 0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
//        { {  0.0f, 1.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },
        
//        { { -0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
//        { {  0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
//        { {  0.0f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
//        { {  0.0f, -1.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },
        
//        { { -0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
//        { { -0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
//        { { -0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
//        { { -1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },
        
//        { { 0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },
//        { { 0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },
//        { { 0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },
//        { { 1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } }
//    };
    
//    GLubyte Indices[] =
//    {
//        0, 1, 3,
//        0, 3, 2,
//        3, 1, 4,
//        3, 4, 2,
        
//        0, 5, 7,
//        0, 7, 6,
//        7, 5, 8,
//        7, 8, 6,
        
//        0, 9, 11,
//        0, 11, 10,
//        11, 9, 12,
//        11, 12, 10,
        
//        0, 13, 15,
//        0, 15, 14,
//        15, 13, 16,
//        15, 16, 14
//    };
    
//    GLubyte AlternateIndices[] =
//    {
//        3, 4, 16,
//        3, 15, 16,
//        15, 16, 8,
//        15, 7, 8,
//        7, 8, 12,
//        7, 11, 12,
//        11, 12, 4,
//        11, 3, 4,
        
//        0, 11, 3,
//        0, 3, 15,
//        0, 15, 7,
//        0, 7, 11
//    };

//    GLenum ErrorCheckValue = glGetError();
//    const size_t BufferSize = sizeof(Vertices);
//    const size_t VertexSize = sizeof(Vertices[0]);
//    const size_t RgbaOffset = sizeof(Vertices[0].XYZW);

//    glGenBuffers(1, &(_buffer->_VboId));
    
//    glGenVertexArrays(1, &(_buffer->_VaoId));
//    glBindVertexArray(_buffer->_VaoId);
    
//    glBindBuffer(GL_ARRAY_BUFFER, _buffer->_VboId);
//    glBufferData(GL_ARRAY_BUFFER, BufferSize, Vertices, GL_STATIC_DRAW);
    
//    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, VertexSize, 0);
//    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, VertexSize, (GLvoid*)RgbaOffset);
    
//    glEnableVertexAttribArray(0);
//    glEnableVertexAttribArray(1);
    
//    glGenBuffers(2, _buffer->_IndexBufferId);
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer->_IndexBufferId[0]);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices), Indices, GL_STATIC_DRAW);
    
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer->_IndexBufferId[1]);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(AlternateIndices), AlternateIndices, GL_STATIC_DRAW);
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer->_IndexBufferId[0]);

//    ErrorCheckValue = glGetError();
//    _buffer->verb.Assert(ErrorCheckValue == GL_NO_ERROR, verbosity::ASSERT_CRITICAL,
//                "Error while trying to initialize the buffers",
//                ": "+std::string((char*)gluErrorString(ErrorCheckValue)));
}

void GraphicsBuffer::DestroyVBO()
{
//    GLenum ErrorCheckValue = glGetError();

//    glDisableVertexAttribArray(1);
//    glDisableVertexAttribArray(0);

//    glBindBuffer(GL_ARRAY_BUFFER, 0);

//    glDeleteBuffers(1, &(_buffer->_VboId));
    
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
//    glDeleteBuffers(2, _buffer->_IndexBufferId);

//    glBindVertexArray(0);
//    glDeleteVertexArrays(1, &(_buffer->_VaoId));

//    ErrorCheckValue = glGetError();
//    _buffer->verb.Assert(ErrorCheckValue != GL_NO_ERROR, verbosity::ASSERT_CASUAL,
//                         "Error while trying to destroy buffers",
//                         ": "+std::string((char*)gluErrorString(ErrorCheckValue)));
}

void GraphicsBuffer::CreateShaders()
{
    _buffer->verb.debug() << "Creating shaders"; _buffer->verb.end();
    _buffer->_shaderProgramId = glCreateProgram();
    _buffer->verb.debug() << "Created glProgram"; _buffer->verb.end();
    CheckGLError(_buffer->verb, "Could not create the shader program");

    _buffer->verb.debug() << "Loading fragment shader"; _buffer->verb.end();
    _buffer->_fragmentShader.load("../shaders/SimpleShader.fragment.glsl", GL_FRAGMENT_SHADER);
    _buffer->verb.debug() << "Loading vertex shader"; _buffer->verb.end();
    _buffer->_vertexShader.load("../shaders/SimpleShader.vertex.glsl", GL_VERTEX_SHADER);

    _buffer->verb.debug() << "Attaching shaders to the shading program"; _buffer->verb.end();
    glAttachShader(_buffer->_shaderProgramId, _buffer->_fragmentShader.id());
    glAttachShader(_buffer->_shaderProgramId, _buffer->_vertexShader.id());

    glLinkProgram(_buffer->_shaderProgramId);
    CheckGLError(_buffer->verb, "Could not link the shader program");

    _buffer->_ModelMatrixId = glGetUniformLocationARB( _buffer->_shaderProgramId, "ModelMatrix");
    _buffer->_ViewMatrixId = glGetUniformLocationARB( _buffer->_shaderProgramId, "ViewMatrix");
    _buffer->_ProjectionMatrixId = glGetUniformLocationARB( _buffer->_shaderProgramId, "ProjectionMatrix");

    _buffer->verb.debug() << "Finished creating shaders"; _buffer->verb.end();
}

void GraphicsBuffer::DestroyShaders()
{
    _buffer->verb.debug() << "Destroying shaders"; _buffer->verb.end();

    glUseProgram(0);

    glDetachShader(_buffer->_shaderProgramId, _buffer->_fragmentShader.id());
    glDetachShader(_buffer->_shaderProgramId, _buffer->_vertexShader.id());

    glDeleteShader(_buffer->_fragmentShader.id());
    glDeleteShader(_buffer->_vertexShader.id());

    CheckGLError(_buffer->verb, "Error while trying to destroy the shaders", verbosity::ASSERT_CASUAL);
}
