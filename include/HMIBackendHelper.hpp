#ifndef HMIBACKENDHELPER_H
#define HMIBACKENDHELPER_H

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QQmlEngine>
#include <QHash>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
//#include "tf2_ros/tf2_ros/transform_listener.hpp"
//sudo nmcli connection modify "Wired connection 1" ipv4.method manual ipv4.address 192.168.0.8/24


#include "include/PLCTag.hpp"
#include "include/LIDARScan2DData.hpp"

class HMIBackendHelper : public QObject
{
    Q_OBJECT
    //QML_ELEMENT
    //QML_SINGLETON    
    public:
        explicit HMIBackendHelper(QObject *parent = nullptr);
        ~HMIBackendHelper();
        bool initialize(QGuiApplication *qGuiApplication);
    signals:
        void timeToPublish();
    public slots:
        void onConnectToPLC();

    private:
        QQmlApplicationEngine _QQmlApplicationEngine;
        QDateTime _dateTimeOnApplicationStart = QDateTime::currentDateTime();
        std::shared_ptr<QTimer> _ros2PublishTimer = nullptr;
        rclcpp::Node::SharedPtr _ros2Node = nullptr;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr _ros2Publisher = nullptr;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _ros2LIDARScannerSubscription = nullptr;
        laser_geometry::LaserProjection _LaserProjection;
        //tf2_ros::TransformListener _TransformListener;
        std::unique_ptr<PLCTag> _PLCTag = nullptr;
        std::unique_ptr<LIDARScan2DData> _LIDARScan2DData = nullptr;
        QString _plcAddress = "192.168.50.102";//"10.111.42.192";//
        QString _plcType = "controllogix";//controllogix //micro800
        QString _plcProgramName = "Program:MainProgram."; //"Program:MainProgram.";

        void initializeROS2();
        void setupConnections();
        void startTimers();
        void publishToROS2();
        void scanCallBack(sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif // HMIBACKENDHELPER_H
