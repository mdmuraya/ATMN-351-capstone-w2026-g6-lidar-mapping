#ifndef MAINBACKENDHELPER_H
#define MAINBACKENDHELPER_H

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QQmlEngine>
#include <QHash>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "include/PLCTag.hpp"

class MainBackendHelper : public QObject
{
    Q_OBJECT
    //QML_ELEMENT
    //QML_SINGLETON    
    public:
        explicit MainBackendHelper(QObject *parent = nullptr);
        ~MainBackendHelper();
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
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _ros2Publisher = nullptr;
        std::unique_ptr<PLCTag> _PLCTag = nullptr;
        QString _plcAddress = "192.168.40.62"; //"10.116.204.159";
        QString _plcType = "micro800";//controllogix //micro800
        QString _plcProgramName = ""; //"Program:MainProgram.";

        void initializeROS2();
        void setupConnections();
        void startTimers();
        void publishToROS2();
};

#endif // MAINBACKENDHELPER_H
