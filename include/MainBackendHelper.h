#ifndef MAINBACKENDHELPER_H
#define MAINBACKENDHELPER_H

#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QQmlEngine>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

enum HMI_RUN_STATE
{
    HMI_RUN_STATE_OFF =0,
    HMI_RUN_STATE_JOG =1,
    HMI_RUN_STATE_AUTO =2
};


class MainBackendHelper : public QObject
{
    Q_OBJECT
    //QML_ELEMENT
    //QML_SINGLETON
    Q_PROPERTY(bool plcRunTag READ getPLCRunTag WRITE setPLCRunTag NOTIFY plcRunTagChanged)
    Q_PROPERTY(HMI_RUN_STATE hmiRunState READ getHMIRunStateTag WRITE setHMIRunStateTag NOTIFY hmiRunStateTagChanged)
    Q_PROPERTY(bool runStateOFF READ getRunStateOFF WRITE setRunStateOFF NOTIFY runStateOFFChanged)
    Q_PROPERTY(bool runStateJOG READ getRunStateJOG WRITE setRunStateJOG NOTIFY runStateJOGChanged)
    Q_PROPERTY(bool runStateAUTO READ getRunStateAUTO WRITE setRunStateAUTO NOTIFY runStateAUTOChanged)

    public:
        explicit MainBackendHelper(QObject *parent = nullptr);
        ~MainBackendHelper();
        bool getPLCRunTag() const;
        void setPLCRunTag(bool plcRunTagValue);

        bool getRunStateOFF() const;
        void setRunStateOFF(bool runStateOFFValue);

        bool getRunStateJOG() const;
        void setRunStateJOG(bool runStateJOGValue);

        bool getRunStateAUTO() const;
        void setRunStateAUTO(bool runStateAUTOValue);

        HMI_RUN_STATE getHMIRunStateTag() const;
        void setHMIRunStateTag(HMI_RUN_STATE hmiRunStateTagValue);




    signals:
        void getPLCStatus();
        void timeToPublish();
        void plcRunTagChanged(bool plcRunTagValue);
        void hmiRunStateTagChanged(HMI_RUN_STATE hmiRunStateTagValue);
        void runStateOFFChanged(bool runStateOFFValue);
        void runStateJOGChanged(bool runStateJOGValue);
        void runStatAUTOChanged(bool runStateAUTOValue);

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
        HMI_RUN_STATE _hmiRunStateTagValue = HMI_RUN_STATE_OFF;
        bool _runStateOFFValue = false;
        bool _runStateJOGValue = false;
        bool _runStateAUTOValue = false;

        void initializeROS2();
        void setupConnections();
        void startTimers();
};

#endif // MAINBACKENDHELPER_H
