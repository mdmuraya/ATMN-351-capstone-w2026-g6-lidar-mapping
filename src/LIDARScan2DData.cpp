#include <QDebug>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "include/LIDARScan2DData.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

LIDARScan2DData::LIDARScan2DData(QObject *parent)
    : QObject{parent}
{
    qDebug() << "LIDARScan2DData::LIDARScan2DData()";
}


uint32_t LIDARScan2DData::getNumberOfPoints() const
{
    return _numberOfPoints;
}

float LIDARScan2DData::getAngleInDegrees() const
{
    return _angleInDegrees;
}


float LIDARScan2DData::getAngleInRadians() const
{
    return _angleInRadians;
}

std::vector<float> LIDARScan2DData::getRanges() const
{
    return _ranges;
}


void LIDARScan2DData::setScanData(sensor_msgs::msg::LaserScan::SharedPtr scanData)
{
    qDebug() << "LIDARScan2DData::setScanData()";

    _numberOfPoints = scanData->scan_time / scanData->time_increment;
    _ranges = scanData->ranges;

    //printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scanData->header.frame_id.c_str(), _numberOfPoints);
    //printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scanData->angle_min),
           //RAD2DEG(scanData->angle_max));

    for (uint32_t i = 0; i < _numberOfPoints; i++) {
        _angleInRadians = (scanData->angle_min + scanData->angle_increment * i);
        _angleInDegrees = RAD2DEG(_angleInRadians);

        printf("[SLLIDAR INFO]: angle-distance : [%f, %f, %f]\n", _angleInRadians, _angleInDegrees, _ranges[i]);
    }

    emit scanDataChanged();
}

LIDARScan2DData::~LIDARScan2DData()
{
    qDebug() << "LIDARScan2DData::~LIDARScan2DData()";
}
