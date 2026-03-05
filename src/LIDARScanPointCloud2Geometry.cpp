#include <QDebug>
#include <QRandomGenerator>
#include <QVector3D>

#include "include/LIDARScanPointCloud2Geometry.hpp"


LIDARScanPointCloud2Geometry::LIDARScanPointCloud2Geometry(QObject *parent)
{
    qDebug() << "LIDARScanPointCloud2Geometry::LIDARScanPointCloud2Geometry()";

    QByteArray vertexData;
    vertexData.resize(sizeof(float) * 3 * 100);
    float *p = reinterpret_cast<float *>(vertexData.data());

    for (int var = 0; var < 100; ++var)
    {
        const QVector3D vertex = generateRandomVertex(-300.0f, 300.0f);
        *p++ = vertex.x();
        *p++ = vertex.y();
        *p++ = vertex.z();
    }



    updateData(vertexData);
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

    // QByteArray vertexData;
    // vertexData.resize(sizeof(float) * 3 * m_count);
    // float *p = reinterpret_cast<float *>(vertexData.data());

    // for (int var = 0; var < m_count; ++var)
    // {
    //     const QVector3D vertex = generateRandomVertex(-300.0f, 300.0f);
    //     *p++ = vertex.x();
    //     *p++ = vertex.y();
    //     *p++ = vertex.z();
    // }

    setVertexData(vertexData);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);
    setStride(3 * sizeof(float));
    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                 0,
                 QQuick3DGeometry::Attribute::F32Type);

     update();

}
