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
    Q_PROPERTY(bool plcRunTag READ getPLCRunTag WRITE setPLCRunTag NOTIFY plcRunTagChanged)

    public:
        explicit MainBackendHelper(QObject *parent = nullptr);
        ~MainBackendHelper();
        bool getPLCRunTag() const;
        void setPLCRunTag(bool plcRunTagValue);

    signals:
        void getPLCStatus();
        void timeToPublish();
        void plcRunTagChanged(bool plcRunTagValue);

    public slots:
        void onConnectToPLC();
        void onStartClicked();
        void onStartPressed();
        void onStartReleased();
        void onResetSystem();
        void onStopClicked();
        void onStopPressed();
        void onStopReleased();
        void onSafetyStop();
        void onMoveToHome();
        void onMoveLeft();
        void onMoveRight();
        void onMoveForward();
        void onMoveBack();
        void onGetPLCStatus();
        void onTimeToPublish();

    private:
        QDateTime _dateTimeOnApplicationStart = QDateTime::currentDateTime();
        std::unique_ptr<QTimer> _getPLCStatusTimer = nullptr;
        std::shared_ptr<QTimer> _publishTimer = nullptr;
        rclcpp::Node::SharedPtr _ros2Node = nullptr;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher = nullptr;
        bool _plcRunTagValue = false;

        void initializeROS2();
        void setupConnections();
        void startTimers();
};

#endif // MAINBACKENDHELPER_H
