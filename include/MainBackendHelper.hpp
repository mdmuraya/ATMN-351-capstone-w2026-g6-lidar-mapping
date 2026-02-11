#ifndef MAINBACKENDHELPER_H
#define MAINBACKENDHELPER_H

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
    Q_PROPERTY(bool runState READ getRunState WRITE setRunState NOTIFY runStateChanged)
    Q_PROPERTY(bool runStateAUTO READ getRunStateAUTO WRITE setRunStateAUTO NOTIFY runStateAUTOChanged)

    public:
        explicit MainBackendHelper(QObject *parent = nullptr);
        ~MainBackendHelper();

        bool getRunState() const;
        void setRunState(bool newValue);
        bool getRunStateAUTO() const;
        void setRunStateAUTO(bool newValue);

    signals:
        void getPLCStatus();
        void timeToPublish();
        void runStateChanged(bool newValue);
        void runStateAUTOChanged(bool newValue);

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
        bool _runState = false;
        bool _runStateAUTO = false;
        std::unique_ptr<PLCTag> _PLCTag = nullptr;
        QString _plcAddress = "192.168.40.62";
        QString _plcType = "Micro800";
        QString _plcProgramName = ""; //"Program:Prog1.";


        void initializeROS2();
        void setupConnections();
        void startTimers();
};

#endif // MAINBACKENDHELPER_H
