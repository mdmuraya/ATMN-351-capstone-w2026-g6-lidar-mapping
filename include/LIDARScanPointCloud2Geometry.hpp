#ifndef LIDARSCANPOINTCLOUD2GEOMETRY_H
#define LIDARSCANPOINTCLOUD2GEOMETRY_H

#include <QObject>
#include <QQuick3DGeometry>
#include <QVector3D>

class LIDARScanPointCloud2Geometry : public QQuick3DGeometry
{
    Q_OBJECT    
    public:
        explicit LIDARScanPointCloud2Geometry(QQuick3DObject *parent = nullptr);
        ~LIDARScanPointCloud2Geometry();

    signals:

    public slots:
        void updatePoints(const QVector<QVector3D> &points);

    private:
        QVector3D generateRandomVertex(float min, float max) const;
};

#endif // LIDARSCANPOINTCLOUD2GEOMETRY_H
