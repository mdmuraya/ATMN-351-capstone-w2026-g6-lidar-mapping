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
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "laser_geometry/laser_geometry.hpp"
//#include "tf2_ros/tf2_ros/transform_listener.hpp"
//sudo nmcli connection modify "Wired connection 1" ipv4.method manual ipv4.address 192.168.0.8/24


#include "include/PLCTag.hpp"
#include "include/LIDARScan2DData.hpp"
#include "include/LIDARScanPointCloud2Geometry.hpp"

class HMIBackendHelper : public QObject
{
    Q_OBJECT
    //QML_ELEMENT
    //QML_SINGLETON
    Q_PROPERTY(QString plcAddress READ getPLCAddress WRITE setPLCAddress NOTIFY plcAddressChanged)
    Q_PROPERTY(QString plcFamilyId READ getPLCFamilyId WRITE setPLCFamilyId NOTIFY plcFamilyIdChanged)
    Q_PROPERTY(QVariantList listOfPLCFamily READ getListOfPLCFamily NOTIFY listOfPLCFamilyChanged)
    public:
        explicit HMIBackendHelper(QObject *parent = nullptr);
        ~HMIBackendHelper();
        bool initialize(QGuiApplication *qGuiApplication);
        QVariantList  getListOfPLCFamily() const;
        QString getPLCAddress() const;
        QString getPLCFamilyId() const;
        void setPLCAddress(QString newValue);
        void setPLCFamilyId(QString newValue);

    signals:
        void timeToPublish();
        void listOfPLCFamilyChanged();
        void plcAddressChanged(QString newValue);
        void plcFamilyIdChanged(QString newValue);

    public slots:
        void connectToPLC();
        void disconnectFromPLC();

    private:
        QQmlApplicationEngine _QQmlApplicationEngine;
        QDateTime _dateTimeOnApplicationStart = QDateTime::currentDateTime();
        std::shared_ptr<QTimer> _ros2PublishTimer = nullptr;
        rclcpp::Node::SharedPtr _ros2Node = nullptr;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr _ros2Publisher = nullptr;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _ros2LIDARScannerSubscription = nullptr;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _ros2SICKMultiscan100LIDARScannerSubscription = nullptr;
        laser_geometry::LaserProjection _LaserProjection;
        //tf2_ros::TransformListener _TransformListener;
        std::unique_ptr<PLCTag> _PLCTag = nullptr;
        std::unique_ptr<LIDARScan2DData> _LIDARScan2DData = nullptr;
        std::unique_ptr<LIDARScanPointCloud2Geometry> _LIDARScanPointCloud2Geometry = nullptr;

        QVariantList _listOfPLCFamily = {};
        QString _plcAddress = "";//"192.168.50.102";//"10.111.42.192";//
        QString _plcFamilyId = "";//"controllogix";//controllogix //micro800
        QString _plcProgramName = "Program:MainProgram."; //"Program:MainProgram.";

        void initializeROS2();
        void setupConnections();
        void startTimers();
        void publishToROS2();
        void scanCallBack(sensor_msgs::msg::LaserScan::SharedPtr scan);
        void scanSICKMultiscan100CallBack(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg);
};

#endif // HMIBACKENDHELPER_H
