#include <QDebug>
#include <QRandomGenerator>
#include <QVector3D>

#include "include/LIDARScanPointCloud2Geometry.hpp"


LIDARScanPointCloud2Geometry::LIDARScanPointCloud2Geometry(QObject *parent)
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

void LIDARScanPointCloud2Geometry::updateData(QByteArray &vertexData)
{
    clear();

    setVertexData(vertexData);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);
    setStride(3 * sizeof(float));
    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                 0,
                 QQuick3DGeometry::Attribute::F32Type);

     update();

}
