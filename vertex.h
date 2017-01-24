/* Description:
 * Vertex class is required to create user-defined geometry.
 * An object of this class stores 3d coordinates and color for each vertex
 * of the required geometry/shape. For instance, a quadrotor.
 */

#ifndef VERTEX_H
#define VERTEX_H

#include <QVector3D>

class Vertex
{
public:
    // Constructor
    Vertex(const QVector3D &position, const QVector3D &color);

    // Destructor
    ~Vertex();

    // Data members and Member functions
    void setPosition(const QVector3D& position);
    void setColor(const QVector3D& color);
    const QVector3D& position() const;
    const QVector3D& color() const;
    static const int PositionTupleSize = 3; // No of elements in the position data (3: x,y,z)
    static const int ColorTupleSize = 3; // No of elements in the color data (3: R,G,B)
    static int positionOffset();
    static int colorOffset();
    static int stride(); // Sizeof(Vertex object)

private:
    QVector3D Qvec_position;
    QVector3D Qvec_color;
};

// Note: Q_MOVABLE_TYPE means it can be memcpy'd.
Q_DECLARE_TYPEINFO(Vertex, Q_MOVABLE_TYPE);

#endif // VERTEX_H
