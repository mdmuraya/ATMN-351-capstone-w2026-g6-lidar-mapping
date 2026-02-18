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
//#include "std_msgs/msg/string.hpp"
#include "example_interfaces/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "include/PLCTag.hpp"

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
        std::unique_ptr<PLCTag> _PLCTag = nullptr;
        QString _plcAddress = "192.168.40.62"; //"10.111.42.192";//
        QString _plcType = "micro800";//controllogix //micro800
        QString _plcProgramName = ""; //"Program:MainProgram.";

        void initializeROS2();
        void setupConnections();
        void startTimers();
        void publishToROS2();
        void scanCallBack(sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif // HMIBACKENDHELPER_H
