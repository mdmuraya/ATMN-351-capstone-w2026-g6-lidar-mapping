#include <QDebug>
#include <QRandomGenerator>
#include <QVector3D>

#include "include/LIDARScanPointCloud2Geometry.hpp"

struct Vertex {
    float x, y, z;
    float nx, ny, nz;
};


LIDARScanPointCloud2Geometry::LIDARScanPointCloud2Geometry(QQuick3DObject  *parent)
    : QQuick3DGeometry(parent)
{
    qDebug() << "LIDARScanPointCloud2Geometry::LIDARScanPointCloud2Geometry()";


}

QVector3D LIDARScanPointCloud2Geometry::generateRandomVertex(float min, float max) const
{
    float v1 = QRandomGenerator::global()->generateDouble() * (max - min) + min;
    float v2 = QRandomGenerator::global()->generateDouble() * (max - min) + min;
    float v3 = QRandomGenerator::global()->generateDouble() * (max - min) + min;

    return QVector3D(v1, v2, v3);
}

LIDARScanPointCloud2Geometry::~LIDARScanPointCloud2Geometry()
{
    qDebug() << "LIDARScanPointCloud2Geometry::~LIDARScanPointCloud2Geometry()";
}

void LIDARScanPointCloud2Geometry::updatePoints(const QVector<QVector3D> &points)
{
    // clear();

    // setVertexData(vertexData);
    // setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);
    // setStride(3 * sizeof(float));
    // addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
    //              0,
    //              QQuick3DGeometry::Attribute::F32Type);

    if (points.isEmpty())
        return;

    QByteArray vbuf;
    vbuf.resize(points.size() * sizeof(Vertex));
    auto *v = reinterpret_cast<Vertex*>(vbuf.data());

    for (int i = 0; i < points.size(); ++i) {
        v[i].x = points[i].x();
        v[i].y = points[i].y();
        v[i].z = points[i].z();
        v[i].nx = 0.0f; v[i].ny = 1.0f; v[i].nz = 0.0f;
    }

    clear();
    setVertexData(vbuf);
    setStride(sizeof(Vertex));
    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic, 0, QQuick3DGeometry::Attribute::F32Type);
    addAttribute(QQuick3DGeometry::Attribute::NormalSemantic, offsetof(Vertex, nx), QQuick3DGeometry::Attribute::F32Type);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);


     update();

}
