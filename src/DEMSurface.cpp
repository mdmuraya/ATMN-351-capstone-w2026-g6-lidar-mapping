
#include "sensor_msgs/msg/laser_scan.hpp"

#include "include/DEMSurface.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

DEMSurface::DEMSurface(QObject *parent)
    : QObject{parent}
{
    qDebug() << "DEMSurface::DEMSurface()";
}

QUrl DEMSurface::getHeightMap() const
{
    return _heightMap;
}

void DEMSurface::updatePoints(const QVector<QVector3D> &points)
{
    if (points.isEmpty())
        return;

    const int W = 512, H = 512;
    float minX=1e9f, maxX=-1e9f, minY=1e9f, maxY=-1e9f, minZ=1e9f, maxZ=-1e9f;

    for (const auto &p : points) {
        minX = std::min(minX, p.x()); maxX = std::max(maxX, p.x());
        minY = std::min(minY, p.y()); maxY = std::max(maxY, p.y());
        minZ = std::min(minZ, p.z()); maxZ = std::max(maxZ, p.z());
    }

    QVector<float> grid(W * H, std::numeric_limits<float>::quiet_NaN());

    for (const auto &p : points) {
        int ix = int((p.x() - minX) / (maxX - minX + 1e-6f) * (W - 1));
        int iy = int((p.y() - minY) / (maxY - minY + 1e-6f) * (H - 1));
        if (ix < 0 || ix >= W || iy < 0 || iy >= H) continue;
        int idx = iy * W + ix;
        if (std::isnan(grid[idx]) || p.z() > grid[idx])
            grid[idx] = p.z();
    }

    QImage img(W, H, QImage::Format_Grayscale8);
    img.fill(0);

    for (int y = 0; y < H; ++y) {
        uchar *line = img.scanLine(y);
        for (int x = 0; x < W; ++x) {
            float z = grid[y * W + x];
            if (std::isnan(z)) {
                line[x] = 0;
            } else {
                float t = (z - minZ) / (maxZ - minZ + 1e-6f);
                line[x] = uchar(qBound(0.0f, t, 1.0f) * 255.0f);
            }
        }
    }

    const QString path =
        QStandardPaths::writableLocation(QStandardPaths::TempLocation)
        + "/lidar_dem.png";
    img.save(path);
    _heightMap = QUrl::fromLocalFile(path);
    emit heightMapChanged();
}

DEMSurface::~DEMSurface()
{
    qDebug() << "DEMSurface::~DEMSurface()";
}
