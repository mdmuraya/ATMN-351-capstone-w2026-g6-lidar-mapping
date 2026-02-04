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
    //Q_PROPERTY(bool plcRunTag READ getPLCRunTag WRITE setPLCRunTag NOTIFY plcRunTagChanged)
    Q_PROPERTY(HMI_RUN_STATE hmiRunState READ getHMIRunStateTag WRITE setHMIRunStateTag NOTIFY hmiRunStateTagChanged)
    Q_PROPERTY(bool runState READ getRunState WRITE setRunState NOTIFY runStateChanged)
    Q_PROPERTY(bool runStateJOG READ getRunStateJOG WRITE setRunStateJOG NOTIFY runStateJOGChanged)
    Q_PROPERTY(bool runStateAUTO READ getRunStateAUTO WRITE setRunStateAUTO NOTIFY runStateAUTOChanged)

    public:
        explicit MainBackendHelper(QObject *parent = nullptr);
        ~MainBackendHelper();
        //bool getPLCRunTag() const;
        //void setPLCRunTag(bool newValue);

        bool getRunState() const;
        void setRunState(bool newValue);
        bool getRunStateJOG() const;
        void setRunStateJOG(bool newValue);

        bool getRunStateAUTO() const;
        void setRunStateAUTO(bool newValue);

        HMI_RUN_STATE getHMIRunStateTag() const;
        void setHMIRunStateTag(HMI_RUN_STATE newValue);




    signals:
        void getPLCStatus();
        void timeToPublish();
        //void plcRunTagChanged(bool newValue);
        void hmiRunStateTagChanged(HMI_RUN_STATE newValue);
        void runStateChanged(bool newValue);
        void runStateJOGChanged(bool newValue);
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
        HMI_RUN_STATE _hmiRunStateTag = HMI_RUN_STATE_OFF;
        bool _runState = false;
        bool _runStateJOG = false;
        bool _runStateAUTO = false;

        void initializeROS2();
        void setupConnections();
        void startTimers();

        bool getPLCTag(QString tagName, bool &tagValue);
};

#endif // MAINBACKENDHELPER_H
