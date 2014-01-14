
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
    {
        if(_buffer->_graphics[i] == &object)
        {
            _buffer->verb.brief() << "Graphical Object '"+object.name()+"' is already being displayed!"; _buffer->verb.end();
            return;
        }
    }

    _buffer->verb.debug() << "Generating vertex buffer for GraphicsObject '" << object.name() << "'"; _buffer->verb.end();
    glGenBuffersARB( 1, &(object._vertexBufferAddress) );
    _buffer->verb.debug() << "Generated vertex buffer"; _buffer->verb.end();
    CheckGLError(_buffer->verb, "Attempted to generate vertex buffer");
    
    _buffer->verb.debug() << "Generating vertex array for '" << object.name() << "'"; _buffer->verb.end();
    glGenVertexArrays(1, &(object._vertexArrayAddress));
    CheckGLError(_buffer->verb, "Attempted to generate vertex array");
    _buffer->verb.debug() << "Generated vertex array"; _buffer->verb.end();
    glBindVertexArray(object._vertexArrayAddress);
    _buffer->verb.debug() << "Bound vertex array"; _buffer->verb.end();
    CheckGLError(_buffer->verb, "Attempted to bind vertex array");
    
    _buffer->verb.debug() << "Binding vertex buffer for '" << object.name() << "'"; _buffer->verb.end();
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, object._vertexBufferAddress );
    CheckGLError(_buffer->verb, "Attempted to bind vertex buffer");
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, object._vertices.size()*sizeof(Vertex),
                     &(object._vertices[0]), GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error passing vertex data to graphics memory");
    _buffer->verb.debug() << "Loaded vertex buffer data for '" << object.name() << "'"; _buffer->verb.end();
    
    glVertexAttribPointerARB(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
    CheckGLError(_buffer->verb, "Error setting vertex attributes");
    glVertexAttribPointerARB(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)sizeof(Vertex::XYZW));
    CheckGLError(_buffer->verb, "Error settings color attributes");
    
    glEnableVertexAttribArrayARB(0);
    glEnableVertexAttribArrayARB(1);
    CheckGLError(_buffer->verb, "Error enable vertex attributes");

    _buffer->verb.debug() << "Generating index buffer for '" << object.name() << "'"; _buffer->verb.end();
    glGenBuffersARB( 1, &(object._faceBufferAddress) );
    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, object._faceBufferAddress );
    glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB, object._faces.size()*sizeof(Face),
                     &(object._faces[0]), GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error passing face data to graphics memory");
    
    object._faceElementSize = 3*object._faces.size();

    _buffer->verb.debug() << "Generating outline buffer for '" << object.name() << "'"; _buffer->verb.end();
    glGenBuffersARB( 1, &(object._outlineIndexBufferAddress) );
    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, object._outlineIndexBufferAddress );
    glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB, object._outline.size()*sizeof(LineElem),
                     &(object._outline[0]), GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error passing outline data to graphics memory");

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
                              "Trying to retrieve non-existent (or deleted) graphic.",
                              " I will give you a reference to an empty graphic.") )
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
    _ViewMatrix.m[14] = -2.0f;
    std::cout << _ProjectionMatrix << std::endl << _ViewMatrix << std::endl << _ModelMatrix << std::endl;
    verb.level = report_level;
}

void GraphicsBuffer::drawElements()
{
//    CheckGLError(_buffer->verb, "About to use gluLookAt");
//    gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
//    CheckGLError(_buffer->verb, "Attempted to use gluLookAt");
    
//    _buffer->verb.debug() << "drawing elements"; _buffer->verb.end();

    glUseProgram(_buffer->_shaderProgramId);
    CheckGLError(_buffer->verb, "Attempted to use shading program");

//    _buffer->_ViewMatrix = FloatIdentity(); // TODO: Remove later
//    _buffer->_ViewMatrix.m[14] = -2;
    glUniformMatrix4fvARB(_buffer->_ViewMatrixId, 1, GL_FALSE, _buffer->_ViewMatrix.m);
//    std::cout << _buffer->_ViewMatrix << std::endl;
//    _buffer->_ProjectionMatrix = FloatIdentity(); // TODO: Remove later
    glUniformMatrix4fvARB(_buffer->_ProjectionMatrixId, 1, GL_FALSE, _buffer->_ProjectionMatrix.m);
//    std::cout << _buffer->_ProjectionMatrix << std::endl;
    CheckGLError(_buffer->verb, "Attempted to set view matrices");
    
    
    /////////////////// TEST OBJECT /////////////////////
    
    glUniformMatrix4fvARB(_buffer->_ModelMatrixId, 1, GL_FALSE, _buffer->_ModelMatrix.m);
    CheckGLError(_buffer->verb, "Attempted to set model matrix");
    
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, _buffer->_testVertexBufferAddress);
    CheckGLError(_buffer->verb, "Attempted to bind test vertex buffer");
    
    glVertexAttribPointerARB(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
    CheckGLError(_buffer->verb, "Error setting vertex attributes");
    glVertexAttribPointerARB(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)sizeof(Vertex::XYZW));
    CheckGLError(_buffer->verb, "Error setting color attributes");
    
    glEnableVertexAttribArrayARB(0);
    glEnableVertexAttribArrayARB(1);
    CheckGLError(_buffer->verb, "Error enabling vertices");
    
    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, _buffer->_testFaceAddress);
    CheckGLError(_buffer->verb, "Attempted to bind test face buffer");

    glDrawElements( GL_TRIANGLES, _buffer->_testSize, GL_UNSIGNED_SHORT, 0);
    CheckGLError(_buffer->verb, "Attempted to draw test element");
    
    ///////////////// END TEST OBJECT ///////////////////
    

    for(size_t i=0; i<_buffer->_graphics.size(); i++)
    {
        GraphicsObject& graphic = *(_buffer->_graphics[i]);
        glUniformMatrix4fvARB(_buffer->_ModelMatrixId, 1, GL_FALSE, _buffer->_ModelMatrix.m);
        CheckGLError(_buffer->verb, "Attempted to set model matrix");

        if(graphic.showingFilled())
        {
            _drawElement(graphic._vertexBufferAddress, graphic._faceBufferAddress,
                         graphic._faceElementSize, GL_TRIANGLES);
        }

        if(graphic.showingOutline())
        {
            glLineWidth(graphic._lineWidth);

            _drawElement(graphic._vertexBufferAddress, graphic._faceBufferAddress,
                         graphic._faceElementSize, GL_LINES);
        }
    }

    glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
    glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
    CheckGLError(_buffer->verb, "Attempted to unbind the vertex and element buffers");

    glUseProgram(0);
}

void GraphicsBuffer::_drawElement(GLuint vertexBufferAddress, GLuint elementBufferAddress, GLuint elementSize, GLuint elementType)
{
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vertexBufferAddress);
    CheckGLError(_buffer->verb, "Attempted to bind vertex buffer");

    glVertexAttribPointerARB(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
    CheckGLError(_buffer->verb, "Error setting vertex attributes");
    glVertexAttribPointerARB(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)sizeof(Vertex::XYZW));
    CheckGLError(_buffer->verb, "Error setting color attributes");

    glEnableVertexAttribArrayARB(0);
    glEnableVertexAttribArrayARB(1);
    CheckGLError(_buffer->verb, "Error enabling vertices");

    glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, elementBufferAddress );
    CheckGLError(_buffer->verb, "Attempted to bind face buffer");
    glDrawElements( elementType, elementSize, GL_UNSIGNED_SHORT, 0);
    CheckGLError(_buffer->verb, "Attempted to draw element");
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
    Vertex vertices[] =
            // BEGIN STAR
//    {
//        { {  0.0f, 0.0f, 0.0f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f } }, //  0

//        { { -0.2f, 0.8f, -1.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } }, //  1
//        { {  0.2f, 0.8f, -1.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } }, //  2
//        { {  0.0f, 0.8f, -1.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } }, //  3
//        { {  0.0f, 1.0f, -1.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } }, //  4

//        { { -0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },//  5
//        { {  0.2f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },//  6
//        { {  0.0f, -0.8f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },//  7
//        { {  0.0f, -1.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },//  8

//        { { -0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } },//  9
//        { { -0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } },// 10
//        { { -0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } },// 11
//        { { -1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } },// 12

//        { { 0.8f, -0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f } }, // 13
//        { { 0.8f,  0.2f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f } }, // 14
//        { { 0.8f,  0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f } }, // 15
//        { { 1.0f,  0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } }  // 16
//    };
            // END STAR
            // BEGIN CUBE
//    {
//        { { -.5f, -.5f,  .5f, 1 }, { 0, 0, 1, 1 } },
//        { { -.5f,  .5f,  .5f, 1 }, { 1, 0, 0, 1 } },
//        { {  .5f,  .5f,  .5f, 1 }, { 0, 1, 0, 1 } },
//        { {  .5f, -.5f,  .5f, 1 }, { 1, 1, 0, 1 } },
//        { { -.5f, -.5f, -.5f, 1 }, { 1, 1, 1, 1 } },
//        { { -.5f,  .5f, -.5f, 1 }, { 1, 0, 0, 1 } },
//        { {  .5f,  .5f, -.5f, 1 }, { 1, 0, 1, 1 } },
//        { {  .5f, -.5f, -.5f, 1 }, { 0, 0, 1, 1 } }
//    };
            // END CUBE
     { 
        Vertex( -.5f, -.5f,  .5f, 0, 0, 1, 1 ),
        Vertex( -.5f,  .5f,  .5f, 1, 0, 0, 1 ),
        Vertex(  .5f,  .5f,  .5f, 0, 1, 0, 1 ),
        Vertex(  .5f, -.5f,  .5f, 1, 1, 0, 1 ),
        Vertex( -.5f, -.5f, -.5f, 1, 1, 1, 1 ),
        Vertex( -.5f,  .5f, -.5f, 1, 0, 0, 1 ),
        Vertex(  .5f,  .5f, -.5f, 1, 0, 1, 1 ),
        Vertex(  .5f, -.5f, -.5f, 0, 0, 1, 1 )
    };
    
    VertexArray derp;
    for(int i=0; i<8; ++i)
    {
        derp.push_back(vertices[i]);
    }
    
    Face herp[] =
            // BEGIN STAR
//    {
//        { 0, 1, 3 },
//        { 0, 3, 2 },
//        { 3, 1, 4 },
//        { 3, 4, 2 },

//        { 0, 5, 7 },
//        { 0, 7, 6 },
//        { 7, 5, 8 },
//        { 7, 8, 6 },

//        { 0, 9, 11 },
//        { 0, 11, 10 },
//        { 11, 9, 12 },
//        { 11, 12, 10 },

//        { 0, 13, 15 },
//        { 0, 15, 14 },
//        { 15, 13, 16 },
//        { 15, 16, 14 }
//    };
            // END STAR
    {
        {0,2,1},  {0,3,2},
        {4,3,0},  {4,7,3},
        {4,1,5},  {4,0,1},
        {3,6,2},  {3,7,6},
        {1,6,5},  {1,2,6},
        {7,5,6},  {7,4,5}
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
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, sizeof(Vertex)*derp.size(), &derp[0], GL_STATIC_DRAW_ARB );
    CheckGLError(_buffer->verb, "Error generating test vertex buffer");

//    glVertexAttribPointerARB(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
//    CheckGLError(_buffer->verb, "Error setting vertex attributes");
//    glVertexAttribPointerARB(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)sizeof(Vertex::XYZW));
//    CheckGLError(_buffer->verb, "Error setting color attributes");

//    glEnableVertexAttribArrayARB(0);
//    glEnableVertexAttribArrayARB(1);
//    CheckGLError(_buffer->verb, "Error enabling vertices");
    
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
