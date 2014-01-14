
#include "GraphicsObject.h"
#include "GraphicsBuffer.h"

using namespace akin;


GraphicsObject::GraphicsObject(Frame &referenceFrame, std::string graphicName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, graphicName, report_level, "Graphic"),
    _lineWidth(1),
    _showFilled(true),
    _showOutline(false)
{

}

GraphicsObject::GraphicsObject(const VertexArray &graphicVertexArray, const FaceArray &graphicFaceArray, Frame &referenceFrame, std::string graphicName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, graphicName, report_level, "Graphic"),
    _vertices(graphicVertexArray),
    _faces(graphicFaceArray),
    _lineWidth(1),
    _showFilled(true),
    _showOutline(false)
{

}

void GraphicsObject::_kinitialize(const GraphicsObject &copy)
{
    _vertices = copy._vertices;
    _faces = copy._faces;
    _lineWidth = copy._lineWidth;
}

GraphicsObject::~GraphicsObject()
{
    GraphicsBuffer::unstageGraphic(*this);
}

const VertexArray& GraphicsObject::respectToWorld()
{
    if(_needsUpdate)
        _updateWorld();

    return _respectToWorld;
}

void GraphicsObject::_updateWorld()
{
    verb.debug() << "Updating GraphicsObject '"+name()+"'"; verb.end();
    _respectToWorld = refFrame().respectToWorld() * _vertices;
}

const VertexArray& GraphicsObject::respectToCamera(Frame &cameraFrame)
{
    if(_needsUpdate)
        _updateCamera(cameraFrame);

    return _respectToCamera;
}

void GraphicsObject::_updateCamera(Frame &cameraFrame)
{
    // TODO: Why does this work? Is it always okay?
    _respectToCamera = cameraFrame.respectToWorld().inverse() * respectToWorld();
}


Box::Box(Frame &referenceFrame, std::string boxName, verbosity::verbosity_level_t report_level) :
    GraphicsObject(referenceFrame, boxName, report_level)
{
    _vertices.resize(24);
    _faces.resize(24);
    _type = "Box";
    dimensions();
}

Box::Box(float width, float length, float height, Frame &referenceFrame, std::string boxName, verbosity::verbosity_level_t report_level) :
    GraphicsObject(referenceFrame, boxName, report_level)
{
    _vertices.resize(24);
    _faces.resize(24);
    _type = "Box";
    dimensions(width, length, height);
}

void Box::color(float red, float green, float blue, float alpha)
{
    color(ColorSpec(red, green, blue, alpha));
}

void Box::color(const ColorSpec &rgba)
{
    for(size_t i=0; i<_vertices.size(); ++i)
    {
        _vertices[i].setColor(rgba);
    }
}

void Box::sideColor(box_side_t side, float red, float green, float blue, float alpha)
{
    sideColor(side, ColorSpec(red, green, blue, alpha));
}

void Box::sideColor(box_side_t side, const ColorSpec &rgba)
{
    for(size_t i=0; i<4; ++i)
    {
        _vertices[4*side+i].setColor(rgba);
    }
}

float Box::width() { return _width; }
void Box::width(float new_width) { dimensions(new_width, _length, _height); }

float Box::length() { return _length; }
void Box::length(float new_length) { dimensions(_width, new_length, _height); }

float Box::height() { return _height; }
void Box::height(float new_height) { dimensions(_width, _length, new_height); }

void Box::dimensions(float new_width, float new_length, float new_height)
{
    _width = new_width;
    _length = new_length;
    _height = new_height;
                            //     x           y           z
    _vertices[0] = Translation( -_width/2, -_length/2,  _height/2); // Top front left
    _vertices[1] = Translation(  _width/2, -_length/2,  _height/2); // Top front right
    _vertices[2] = Translation( -_width/2, -_length/2, -_height/2); // Bottom front left
    _vertices[3] = Translation(  _width/2, -_length/2, -_height/2); // Bottom front right
    _faces[0] = createFace(0, 1, 2);
    _faces[1] = createFace(0, 2, 1);
    _faces[2] = createFace(1, 2, 3);
    _faces[3] = createFace(1, 3, 2);
    // Front face done
    
    _vertices[4] = Translation( -_width/2,  _length/2,  _height/2); // Top back left
    _vertices[5] = Translation(  _width/2,  _length/2,  _height/2); // Top back right
    _vertices[6] = Translation( -_width/2,  _length/2, -_height/2); // Bottom back left
    _vertices[7] = Translation( _width/2,  _length/2, -_height/2);  // Bottom back right
    _faces[4] = createFace(4, 5, 6);
    _faces[5] = createFace(4, 6, 5);
    _faces[6] = createFace(5, 6, 7);
    _faces[7] = createFace(5, 7, 6);
    // Back face done

    _vertices[8] = Translation( -_width/2, -_length/2, -_height/2); // Bottom front left
    _vertices[9] = Translation( -_width/2,  _length/2, -_height/2); // Bottom back left
    _vertices[10]= Translation( -_width/2, -_length/2,  _height/2); // Top front left
    _vertices[11]= Translation( -_width/2,  _length/2,  _height/2); // Top back left
    _faces[8]  = createFace(8, 9, 10);
    _faces[9]  = createFace(8, 10, 9);
    _faces[10] = createFace(9, 10, 11);
    _faces[11] = createFace(9, 11, 10);
    // Left face done
    
    
    _vertices[12] = Translation(  _width/2, -_length/2,  _height/2); // Top front right
    _vertices[13] = Translation(  _width/2,  _length/2,  _height/2); // Top back right
    _vertices[14] = Translation(  _width/2, -_length/2, -_height/2); // Bottom front right
    _vertices[15] = Translation(  _width/2,  _length/2, -_height/2); // Bottom back right
    _faces[12] = createFace(12, 13, 14);
    _faces[13] = createFace(12, 14, 13);
    _faces[14] = createFace(13, 14, 15);
    _faces[15] = createFace(13, 15, 14);
    // Right face done
    
    _vertices[16] = Translation( -_width/2, -_length/2,  _height/2); // Top front left
    _vertices[17] = Translation(  _width/2, -_length/2,  _height/2); // Top front right
    _vertices[18] = Translation( -_width/2,  _length/2,  _height/2); // Top back left
    _vertices[19] = Translation(  _width/2,  _length/2,  _height/2); // Top back right
    _faces[16] = createFace(16, 17, 18);
    _faces[17] = createFace(16, 18, 17);
    _faces[18] = createFace(17, 18, 19);
    _faces[19] = createFace(17, 19, 18);
    // Top face done

    _vertices[20] = Translation( -_width/2, -_length/2, -_height/2); // Bottom front left
    _vertices[21] = Translation(  _width/2, -_length/2, -_height/2); // Bottom front right
    _vertices[22] = Translation( -_width/2,  _length/2, -_height/2); // Bottom back left
    _vertices[23] = Translation(  _width/2,  _length/2, -_height/2); // Bottom back right
    _faces[20] = createFace(20, 21, 22);
    _faces[21] = createFace(20, 22, 21);
    _faces[22] = createFace(21, 22, 23);
    _faces[23] = createFace(21, 23, 22);
    // Bottom face done
}


