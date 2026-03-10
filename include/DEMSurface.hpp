#ifndef DEMSURFACE_HPP
#define DEMSURFACE_HPP

#include <QObject>
#include <QVector3D>
#include <QVector>
#include <QDebug>
#include <QImage>
#include <QStandardPaths>
#include <QtMath>
#include <limits>
#include <QUrl>


#include "sensor_msgs/msg/laser_scan.hpp"

class DEMSurface : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QUrl heightMap READ getHeightMap NOTIFY heightMapChanged)
    public:
        explicit DEMSurface(QObject *parent = nullptr);
        ~DEMSurface();

        QUrl getHeightMap() const;

    signals:
        void heightMapChanged();

    public slots:
        void updatePoints(const QVector<QVector3D> &points);
    private:
        QUrl _heightMap;
};

#endif // DEMSURFACE_HPP
