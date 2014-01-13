
#include "GraphicsBuffer.h"

using namespace akin;

const GLchar* _VertexShader =
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

const GLchar* _FragmentShader =
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

void GraphicsBuffer::setProjectionMatrix(float field_of_view, float aspect_ratio, float near_plane, float far_plane, bool perspective)
{
    if(perspective)
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
    }
    else
    {
        _buffer->_ProjectionMatrix = FloatIdentity();

        _buffer->_ProjectionMatrix.m[0] = 1/aspect_ratio;
    }

    // TODO: Remove this when done debugging
//    _buffer->_ProjectionMatrix = FloatIdentity();

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
//    CheckGLError(_buffer->verb, "About to use gluLookAt");
//    gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
//    CheckGLError(_buffer->verb, "Attempted to use gluLookAt");

    glUseProgram(_buffer->_shaderProgramId);
    CheckGLError(_buffer->verb, "Attempted to use shading program");

//    _buffer->_ViewMatrix = FloatIdentity(); // TODO: Remove later
    glUniformMatrix4fvARB(_buffer->_ViewMatrixId, 1, GL_FALSE, _buffer->_ViewMatrix.m);
//    _buffer->_ProjectionMatrix = FloatIdentity(); // TODO: Remove later
    glUniformMatrix4fvARB(_buffer->_ProjectionMatrixId, 1, GL_FALSE, _buffer->_ProjectionMatrix.m);
    CheckGLError(_buffer->verb, "Attempted to set view matrices");
    
    /////////////////// TEST OBJECT /////////////////////
    
//    _buffer->_ModelMatrix = FloatIdentity(); // TODO: Remove later
    glUniformMatrix4fvARB(_buffer->_ModelMatrixId, 1, GL_FALSE, _buffer->_ModelMatrix.m);
    CheckGLError(_buffer->verb, "Attempted to set model matrix");
    
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, _buffer->_testVertexBufferAddress);
    CheckGLError(_buffer->verb, "Attempted to bind test vertex buffer");
    
    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, _buffer->_testFaceAddress);
    CheckGLError(_buffer->verb, "Attempted to bind test face buffer");

    glDrawElements( GL_TRIANGLES, _buffer->_testSize, GL_UNSIGNED_INT, 0);
    CheckGLError(_buffer->verb, "Attempted to draw test element");
    
    glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
    glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
    CheckGLError(_buffer->verb, "Attempted to unbind the vertex and element buffers");
    
    ///////////////// END TEST OBJECT ///////////////////
    for(size_t i=0; i<_buffer->_graphics.size(); i++)
    {   
        GraphicsObject& graphic = *(_buffer->_graphics[i]);
        _buffer->verb.debug() << "Rendering object '" << graphic.name() << "'"; _buffer->verb.end();
        std::ostringstream data;
        data << _buffer->_ViewMatrixId << ", " << _buffer->_ProjectionMatrixId << ", " << _buffer->_ModelMatrixId;
        _buffer->verb.debug() << "Buffer Addresses: " << data.str(); _buffer->verb.end();
        
        glUniformMatrix4fvARB(_buffer->_ModelMatrixId, 1, GL_FALSE, _buffer->_ModelMatrix.m);
        CheckGLError(_buffer->verb, "Attempted to set model matrix");
    
        glBindBufferARB( GL_ARRAY_BUFFER_ARB, graphic._vertexBufferAddress);
        CheckGLError(_buffer->verb, "Attempted to bind vertex buffer");
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, graphic._faceBufferAddress);
        CheckGLError(_buffer->verb, "Attempted to bind face buffer");

//        // TODO: Check if the next two lines can live outside of the for loop
//        glEnableClientState( GL_VERTEX_ARRAY );
//        CheckGLError(_buffer->verb, "Attempted to enable vertex array");
//        glVertexPointer(3, GL_FLOAT, 0, 0);
//        CheckGLError(_buffer->verb, "Something with the vertex pointer...");
        
//        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex::XYZW), 0);
//        CheckGLError(_buffer->verb, "Attempted to use glVertexAttribPointer for the vertices");
//        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex::RGBA), (GLvoid*)sizeof(Vertex::XYZW));
//        CheckGLError(_buffer->verb, "Attempted to use glVertexAttribPointer for the colors");
        

        glDrawElements( GL_TRIANGLES, graphic._faces.size(), GL_UNSIGNED_INT, 0 );
        CheckGLError(_buffer->verb, "Attempted glDrawElements");

        // TODO: check if the following lines can live otuside of the for loop
//        glDisableClientState( GL_VERTEX_ARRAY );
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
        
        CheckGLError(_buffer->verb, "Attempted to render object "+graphic.name());
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
        _createTestObject();
    }
}

void GraphicsBuffer::_createTestObject()
{
    TestVertex derp[] =
//    {
//        { { 0.0f, 1.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } },
//        { { 1.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } },
//        { { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } }
//    };
    {
        { {  0.0f, 0.0f, 0.0f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f } }, //  0

        { { -0.2f, 0.8f, -1.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } }, //  1
        { {  0.2f, 0.8f, -1.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } }, //  2
        { {  0.0f, 0.8f, -1.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } }, //  3
        { {  0.0f, 1.0f, -1.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } }, //  4

        { { -0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },//  5
        { {  0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },//  6
        { {  0.0f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },//  7
        { {  0.0f, -1.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },//  8

        { { -0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },//  9
        { { -0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },// 10
        { { -0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },// 11
        { { -1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },// 12

        { { 0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } }, // 13
        { { 0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } }, // 14
        { { 0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } }, // 15
        { { 1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } }  // 16
    };
    
    Face herp[] =
//    { 0, 1, 2 };
    {
        { 0, 1, 3 },
        { 0, 3, 2 },
        { 3, 1, 4 },
        { 3, 4, 2 },

        { 0, 5, 7 },
        { 0, 7, 6 },
        { 7, 5, 8 },
        { 7, 8, 6 },

        { 0, 9, 11 },
        { 0, 11, 10 },
        { 11, 9, 12 },
        { 11, 12, 10 },

        { 0, 13, 15 },
        { 0, 15, 14 },
        { 15, 13, 16 },
        { 15, 16, 14 }
    };

    _buffer->_testSize = sizeof(herp)/sizeof(Face::index[0]);
    
    glGenBuffersARB( 1, &(_buffer->_testVertexBufferAddress) );
    CheckGLError(_buffer->verb, "Attempted to generate vertex buffer");

    glGenVertexArrays(1, &(_buffer->_testVertexArrayAddress) );
    CheckGLError(_buffer->verb, "Attempted to generate vertex array");
    glBindVertexArray(_buffer->_testVertexArrayAddress);
    CheckGLError(_buffer->verb, "Attempted to bind vertex array");

    glBindBufferARB( GL_ARRAY_BUFFER_ARB, _buffer->_testVertexBufferAddress );
    CheckGLError(_buffer->verb, "Attempted to bind vertex buffer");
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, sizeof(derp), derp, GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error generating test vertex buffer");

    glVertexAttribPointerARB(0, 4, GL_FLOAT, GL_FALSE, sizeof(derp[0]), 0);
    CheckGLError(_buffer->verb, "Error setting vertex attributes");
    glVertexAttribPointerARB(1, 4, GL_FLOAT, GL_FALSE, sizeof(derp[0]), (GLvoid*)sizeof(derp[0].XYZW));
    CheckGLError(_buffer->verb, "Error setting color attributes");

    glEnableVertexAttribArrayARB(0);
    glEnableVertexAttribArrayARB(1);
    CheckGLError(_buffer->verb, "Error enabling vertices");
    
    glGenBuffersARB( 1, &(_buffer->_testFaceAddress) );
    CheckGLError(_buffer->verb, "Error generating test index buffer");
    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, _buffer->_testFaceAddress );
    CheckGLError(_buffer->verb, "Error binding test index buffer");
    glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB, sizeof(herp), herp, GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error loading test index buffer");


    
    glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
    glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
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
//    GLuint fragShader = glCreateShader(GL_FRAGMENT_SHADER);
//    glShaderSource(fragShader, 1, (const GLchar**)&_FragmentShader, NULL);
//    glCompileShader(fragShader);

    _buffer->verb.debug() << "Loading vertex shader"; _buffer->verb.end();
    _buffer->_vertexShader.load("../shaders/SimpleShader.vertex.glsl", GL_VERTEX_SHADER);
//    GLuint vertShader = glCreateShader(GL_VERTEX_SHADER);
//    glShaderSource(vertShader, 1, (const GLchar**)&_VertexShader, NULL);
//    glCompileShader(vertShader);

    _buffer->verb.debug() << "Attaching shaders to the shading program"; _buffer->verb.end();
    glAttachShader(_buffer->_shaderProgramId, _buffer->_fragmentShader.id());
    glAttachShader(_buffer->_shaderProgramId, _buffer->_vertexShader.id());
//    glAttachShader(_buffer->_shaderProgramId, fragShader);
//    glAttachShader(_buffer->_shaderProgramId, vertShader);

    glLinkProgram(_buffer->_shaderProgramId);
    CheckGLError(_buffer->verb, "Could not link the shader program");
    glUseProgram(_buffer->_shaderProgramId);
    CheckGLError(_buffer->verb, "Could not set up shader program for use");

    _buffer->_ModelMatrixId = glGetUniformLocationARB( _buffer->_shaderProgramId, "ModelMatrix");
    _buffer->_ViewMatrixId = glGetUniformLocationARB( _buffer->_shaderProgramId, "ViewMatrix");
    _buffer->_ProjectionMatrixId = glGetUniformLocationARB( _buffer->_shaderProgramId, "ProjectionMatrix");
    std::ostringstream datastr; datastr << _buffer->_ModelMatrixId << ", " << _buffer->_ViewMatrixId << ", " << _buffer->_ProjectionMatrixId;
    _buffer->verb.debug() << "Matrix ids: " << datastr.str(); _buffer->verb.end();

    _buffer->verb.debug() << "Finished creating shaders"; _buffer->verb.end();
    glUseProgram(0);
    CheckGLError(_buffer->verb, "Attempted to close shader program");
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
