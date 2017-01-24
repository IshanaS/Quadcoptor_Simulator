/* Description:
 * Vertex class is required to create user-defined geometry.
 * An object of this class stores 3d coordinates and color for each vertex
 * of the required geometry/shape. For instance, a quadrotor.
 */

# include "vertex.h"

// Constructor
Vertex::Vertex(const QVector3D &position, const QVector3D &color) : Qvec_position(position), Qvec_color(color) {}

// Destructor
Vertex::~Vertex() {}

// Member functions (Setters and getters)
const QVector3D& Vertex::position() const
{
    return Qvec_position;
}
const QVector3D& Vertex::color() const
{
    return Qvec_color;
}
void Vertex::setPosition(const QVector3D& position)
{
    Qvec_position = position;
}
void Vertex::setColor(const QVector3D& color)
{
    Qvec_color = color;
}

// Member functions (OpenGL Helpers)
int Vertex::positionOffset()
{
    return offsetof(Vertex, Qvec_position);
}
int Vertex::colorOffset()
{
    return offsetof(Vertex, Qvec_color);
}
int Vertex::stride()
{
    return sizeof(Vertex);
}
