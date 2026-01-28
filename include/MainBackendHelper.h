#ifndef MAINBACKENDHELPER_H
#define MAINBACKENDHELPER_H

#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QQmlEngine>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class MainBackendHelper : public QObject
{
    Q_OBJECT
    //QML_ELEMENT
    //QML_SINGLETON
    public:
        explicit MainBackendHelper(QObject *parent = nullptr);
        ~MainBackendHelper();

    signals:
        void requestPLCStatus();
        void timeToPublish();

    public slots:
        void onConnectToPLC();
        void onStartScan();
        void onResetSystem();
        void onStopScan();
        void onSafetyStop();
        void onMoveToHome();
        void onMoveLeft();
        void onMoveRight();
        void onMoveForward();
        void onMoveBack();
        void onRequestPLCStatus();
        void onTimeToPublish();

    private:
        QDateTime _dateTimeOnApplicationStart = QDateTime::currentDateTime();
        std::unique_ptr<QTimer> _getPLCStatusTimer = nullptr;
        std::shared_ptr<QTimer> _publishTimer = nullptr;
        rclcpp::Node::SharedPtr _ros2Node = nullptr;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher = nullptr;

        void initializeROS2();
        void setupConnections();
        void startTimers();
};

#endif // MAINBACKENDHELPER_H
