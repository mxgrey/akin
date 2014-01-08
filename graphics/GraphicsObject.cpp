
#include "GraphicsObject.h"
#include "GraphicsBuffer.h"

using namespace akin;


GraphicsObject::GraphicsObject(Frame &referenceFrame, std::string graphicName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, graphicName, report_level, "Graphic")
{

}

GraphicsObject::GraphicsObject(const VertexArray &graphicVertexArray, const FaceArray &graphicFaceArray, Frame &referenceFrame, std::string graphicName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, graphicName, report_level, "Graphic"),
    vertices(graphicVertexArray),
    faces(graphicFaceArray)
{

}

GraphicsObject::~GraphicsObject()
{
    GraphicsBuffer::removeGraphic(*this);
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
    _respectToWorld = refFrame().respectToWorld() * vertices;
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
    vertices.resize(8);
    faces.resize(24);
    _type = "Box";
    dimensions();
}

Box::Box(float width, float length, float height, Frame &referenceFrame, std::string boxName, verbosity::verbosity_level_t report_level) :
    GraphicsObject(referenceFrame, boxName, report_level)
{
    vertices.resize(8);
    faces.resize(24);
    _type = "Box";
    dimensions(width, length, height);
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
    vertices[0] = Translation( -_width/2, -_length/2,  _height/2); // Top front left
    vertices[1] = Translation(  _width/2, -_length/2,  _height/2); // Top front right
    vertices[2] = Translation( -_width/2, -_length/2, -_height/2); // Bottom front left
    faces[0] = createFace(0, 1, 2);
    faces[1] = createFace(0, 2, 1);

    vertices[3] = Translation(  _width/2, -_length/2, -_height/2); // Bottom front right
    faces[2] = createFace(1, 2, 3);
    faces[3] = createFace(1, 3, 2);
    // Front face done

    vertices[4] = Translation( -_width/2,  _length/2,  _height/2); // Top back left
    faces[4] = createFace(0, 1, 4);
    faces[5] = createFace(0, 4, 1);

    vertices[5] = Translation(  _width/2,  _length/2,  _height/2); // Top back right
    faces[6] = createFace(4, 6, 1);
    faces[7] = createFace(4, 1, 6);
    // Top face done

    vertices[6] = Translation( -_width/2,  _length/2, -_height/2); // Bottom back left
    faces[8] = createFace(4, 6, 2);
    faces[9] = createFace(4, 2, 6);

    faces[10] = createFace(0, 4, 2);
    faces[11] = createFace(0, 2, 4);
    // Left face done

    vertices[7] = Translation( _width/2,  _length/2, -_height/2); // Bottom back right
    faces[12] = createFace(1, 5, 7);
    faces[13] = createFace(1, 7, 5);

    faces[14] = createFace(1, 3, 7);
    faces[15] = createFace(1, 7, 3);
    // Right face done

    // TBR-TBL-BBL
    faces[16] = createFace(5, 4, 6);
    faces[17] = createFace(5, 6, 4);

    // TBR-BBL-BBR
    faces[18] = createFace(5, 6, 7);
    faces[19] = createFace(5, 7, 6);
    // Back face done

    // BFL-BFR-BBL
    faces[20] = createFace(2, 3, 6);
    faces[21] = createFace(2, 6, 3);

    // BFR-BBL-BBR
    faces[22] = createFace(3, 6, 7);
    faces[23] = createFace(3, 7, 6);
}
