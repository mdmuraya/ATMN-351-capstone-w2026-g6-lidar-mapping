#ifndef LIDARSCANPOINTCLOUD2GEOMETRY_H
#define LIDARSCANPOINTCLOUD2GEOMETRY_H

#include <QObject>
#include <QQuick3DGeometry>

class LIDARScanPointCloud2Geometry : public QQuick3DGeometry
{
    Q_OBJECT    
    public:
        explicit LIDARScanPointCloud2Geometry(QObject *parent = nullptr);
        ~LIDARScanPointCloud2Geometry();

    signals:

    public slots:
        void updateData(QByteArray &vertexData);

    private:
        QVector3D generateRandomVertex(float min, float max) const;
};

#endif // LIDARSCANPOINTCLOUD2GEOMETRY_H
