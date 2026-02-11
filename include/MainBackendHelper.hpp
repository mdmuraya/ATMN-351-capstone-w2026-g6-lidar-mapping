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
    Q_PROPERTY(bool runState READ getRunState WRITE setRunState NOTIFY runStateChanged)
    Q_PROPERTY(bool runStateAUTO READ getRunStateAUTO WRITE setRunStateAUTO NOTIFY runStateAUTOChanged)
    Q_PROPERTY(bool redPilotLight READ getRedPilotLight WRITE setRedPilotLight NOTIFY redPilotLightChanged)
    Q_PROPERTY(bool amberPilotLight READ getAmberPilotLight WRITE setAmberPilotLight NOTIFY amberPilotLightChanged)
    Q_PROPERTY(bool greenPilotLight READ getGreenPilotLight WRITE setGreenPilotLight NOTIFY greenPilotLightChanged)
    Q_PROPERTY(bool bluePilotLight READ getBluePilotLight WRITE setBluePilotLight NOTIFY bluePilotLightChanged)
    Q_PROPERTY(bool whitePilotLight READ getWhitePilotLight WRITE setWhitePilotLight NOTIFY whitePilotLightChanged)

    public:
        explicit MainBackendHelper(QObject *parent = nullptr);
        ~MainBackendHelper();
        bool initialize(QGuiApplication *qGuiApplication);

        bool getRunState() const;
        void setRunState(bool newValue);
        bool getRunStateAUTO() const;
        void setRunStateAUTO(bool newValue);

        bool getRedPilotLight() const;
        void setRedPilotLight(bool newValue);

        bool getAmberPilotLight() const;
        void setAmberPilotLight(bool newAmberPilotLight);

        bool getGreenPilotLight() const;
        void setGreenPilotLight(bool newGreenPilotLight);

        bool getBluePilotLight() const;
        void setBluePilotLight(bool newBluePilotLight);

        bool getWhitePilotLight() const;
        void setWhitePilotLight(bool newWhitePilotLight);

    signals:
        void getPLCStatus();
        void timeToPublish();
        void runStateChanged(bool newValue);
        void runStateAUTOChanged(bool newValue);
        void redPilotLightChanged(bool newValue);
        void amberPilotLightChanged(bool newValue);
        void greenPilotLightChanged(bool newValue);
        void bluePilotLightChanged(bool newValue);
        void whitePilotLightChanged(bool newValue);

    public slots:
        void onConnectToPLC();
        void startButtonPressedChanged(bool pressed);
        void stopButtonPressedChanged(bool pressed);
        void resetButtonPressedChanged(bool pressed);
        void moveToHomeButtonPressedChanged(bool pressed);
        void moveLeftButtonPressedChanged(bool pressed);
        void moveBackButtonPressedChanged(bool pressed);
        void moveForwardButtonPressedChanged(bool pressed);
        void moveRightButtonPressedChanged(bool pressed);
        /*
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
        */
        void onGetPLCStatus();
        void onTimeToPublish();

    private:
        QQmlApplicationEngine _QQmlApplicationEngine;
        QDateTime _dateTimeOnApplicationStart = QDateTime::currentDateTime();
        std::unique_ptr<QTimer> _getPLCStatusTimer = nullptr;
        std::shared_ptr<QTimer> _publishTimer = nullptr;
        rclcpp::Node::SharedPtr _ros2Node = nullptr;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher = nullptr;
        bool _runState = false;
        bool _runStateAUTO = false;
        bool _redPilotLight = false;
        bool _amberPilotLight = false;
        bool _greenPilotLight = false;
        bool _bluePilotLight = false;
        bool _whitePilotLight = false;

        std::unique_ptr<PLCTag> _PLCTag = nullptr;
        QString _plcAddress = "192.168.40.62";
        QString _plcType = "Micro800";
        QString _plcProgramName = ""; //"Program:Prog1.";


        void initializeROS2();
        void setupConnections();
        void startTimers();


};

#endif // MAINBACKENDHELPER_H
