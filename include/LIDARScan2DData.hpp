#ifndef LIDARSCAN2DDATA_HPP
#define LIDARSCAN2DDATA_HPP

#include <QObject>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

class LIDARScan2DData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(uint32_t numberOfPoints READ getNumberOfPoints)
    Q_PROPERTY(float angleInDegrees READ getAngleInDegrees)
    Q_PROPERTY(float angleInRadians READ getAngleInRadians)
    Q_PROPERTY(std::vector<float> ranges READ getRanges)
    public:
        explicit LIDARScan2DData(QObject *parent = nullptr);
        ~LIDARScan2DData();

        //sensor_msgs::msg::LaserScan getScanData() const;
        uint32_t getNumberOfPoints() const;
        float getAngleInDegrees() const;
        float getAngleInRadians() const;
        std::vector<float> getRanges() const;
        void setScanData(sensor_msgs::msg::LaserScan::SharedPtr scanData);

    signals:
        void scanDataChanged();
    private:
        sensor_msgs::msg::LaserScan _scanData;
        uint32_t _numberOfPoints = 0;
        float _angleInDegrees = 0.0;
        float _angleInRadians = 0.0;
        std::vector<float> _ranges {};
};

#endif // LIDARSCAN2DDATA_HPP
