#include <QCoreApplication>
#include <QQmlContext>
#include <QDebug>
#include <QVariantList>

#include "include/libplctag.h"

#include "include/MainBackendHelper.hpp"


#define REQUIRED_VERSION 2, 4, 0

MainBackendHelper::MainBackendHelper(QObject *parent) : QObject (parent)
{
    qDebug() << "MainBackendHelper::MainBackendHelper()";
    qDebug() << "Total arguments:" << QCoreApplication::arguments().count();

    for (int i = 0; i < QCoreApplication::arguments().count(); ++i) {
        qDebug() << "Argument" << i << ":" << QCoreApplication::arguments().at(i);
    }

    // Access specific arguments (e.g., the second argument if it exists)
    if (QCoreApplication::arguments().count() > 1) {
        qDebug() << "Second argument:" << QCoreApplication::arguments().at(1);
    }
}


MainBackendHelper::~MainBackendHelper()
{
    qDebug() << "MainBackendHelper::~MainBackendHelper()";

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

bool MainBackendHelper::getRunState() const
{
    return _runState;
}

void MainBackendHelper::setRunState(bool newValue)
{
    if (_runState == newValue)
        return;

    _runState = newValue;
    emit runStateChanged(_runState); // Emit signal to trigger QML updates
}

bool MainBackendHelper::getRunStateAUTO() const
{
    return _runStateAUTO;
}

void MainBackendHelper::setRunStateAUTO(bool newValue)
{
    if (_runStateAUTO == newValue)
        return;

    _runStateAUTO = newValue;
    emit runStateAUTOChanged(_runStateAUTO); // Emit signal to trigger QML updates
}

void MainBackendHelper::onConnectToPLC()
{
    qDebug() << "MainBackendHelper::onConnectToPLC()";
}

void MainBackendHelper::startButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::startButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Start_PB",pressed);
}

void MainBackendHelper::stopButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::stopButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Stop_PB",pressed);
}

void MainBackendHelper::resetButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::resetButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Reset_PB",pressed);
}

void MainBackendHelper::moveToHomeButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::moveToHomeButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Home_PB",pressed);
}

void MainBackendHelper::moveLeftButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::moveLeftButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_MoveLeft_PB",pressed);
}

void MainBackendHelper::moveBackButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::moveBackButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_MoveBack_PB",pressed);
}

void MainBackendHelper::moveForwardButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::moveForwardButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_MoveForward_PB",pressed);
}

void MainBackendHelper::moveRightButtonPressedChanged(bool pressed)
{
    qDebug() << "MainBackendHelper::moveRightButtonPressedChanged()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_MoveRight_PB",pressed);
}

void MainBackendHelper::onGetPLCStatus()
{

    qDebug() << "MainBackendHelper::onGetPLCStatus()" << QDateTime::currentDateTime();
    //here we will get all the PLC tags
    _getPLCStatusTimer->stop();

    bool tagValue=false;


    if(_PLCTag->readPLCTag(_plcProgramName + "System_Running", tagValue))
    {
        setRunState(tagValue);
    }    

    if(_PLCTag->readPLCTag(_plcProgramName + "PHY_Selector_Run_AUTO", tagValue))
    {
        setRunStateAUTO(tagValue);
    }

    if(_PLCTag->readPLCTag(_plcProgramName + "Red_Pilot_Light", tagValue))
    {
        setRedPilotLight(tagValue);
    }

    if(_PLCTag->readPLCTag(_plcProgramName + "Amber_Pilot_Light", tagValue))
    {
        setAmberPilotLight(tagValue);
    }

    if(_PLCTag->readPLCTag(_plcProgramName + "Green_Pilot_Light", tagValue))
    {
        setGreenPilotLight(tagValue);
    }

    if(_PLCTag->readPLCTag(_plcProgramName + "Blue_Pilot_Light", tagValue))
    {
        setBluePilotLight(tagValue);
    }


    if(_PLCTag->readPLCTag(_plcProgramName + "White_Pilot_Light", tagValue))
    {
        setWhitePilotLight(tagValue);
    }

    _getPLCStatusTimer->start();
}

void MainBackendHelper::onTimeToPublish()
{
    qDebug() << "MainBackendHelper::onTimeToPublish()" << QDateTime::currentDateTime();

    if (!rclcpp::ok()) {
        qDebug() << "ROS is not running!";
        _publishTimer->stop();
        return;
    }

    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2! ";
    _publisher->publish(message);

    qDebug() << "Published:" <<  message.data;
    rclcpp::spin_some(_ros2Node);
}

bool MainBackendHelper::initialize(QGuiApplication *qGuiApplication)
{
    qDebug() << "MainBackendHelper::initialize()" << QDateTime::currentDateTime();

    QObject::connect(
        &_QQmlApplicationEngine,
        &QQmlApplicationEngine::objectCreationFailed,
        qGuiApplication,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    _QQmlApplicationEngine.rootContext()->setContextProperty("MainBackendHelper", this);
    _QQmlApplicationEngine.loadFromModule("LIDAR_Mapping_HMI", "Main");

    if(_QQmlApplicationEngine.rootObjects().isEmpty())
    {
        return false;
    }

    _getPLCStatusTimer = std::make_unique<QTimer>();
    _publishTimer = std::make_shared<QTimer>();
    _PLCTag = std::make_unique<PLCTag>(this, _plcAddress, _plcType);

    setupConnections();
    initializeROS2();
    startTimers();

    return true;
}

void MainBackendHelper::initializeROS2()
{
    rclcpp::init(0, nullptr);
    _ros2Node = rclcpp::Node::make_shared("LIDARMapping_HMI_App");
    _publisher = _ros2Node->create_publisher<std_msgs::msg::String>("LIDARMapping_HMI_App_topic", 10);

}

void MainBackendHelper::setupConnections()
{
    qDebug() << "MainBackendHelper::setupConnections()";

    connect(_getPLCStatusTimer.get(), &QTimer::timeout, [this](){
        emit getPLCStatus();
    });

    connect(_publishTimer.get(), &QTimer::timeout, [this](){
        emit timeToPublish();
    });

    connect(this, &MainBackendHelper::getPLCStatus, this, &MainBackendHelper::onGetPLCStatus);
    connect(this, &MainBackendHelper::timeToPublish, this, &MainBackendHelper::onTimeToPublish);

}

void MainBackendHelper::startTimers()
{
    qDebug() << "MainBackendHelper::startTimers()";

    int frequency = 5; //number of times per second
    _getPLCStatusTimer->start((1000/frequency));

    frequency= 1; //number of times per second
    //_publishTimer->start((1000/frequency));
}









bool MainBackendHelper::getRedPilotLight() const
{
    return _redPilotLight;
}

void MainBackendHelper::setRedPilotLight(bool newValue)
{
    if (_redPilotLight == newValue)
        return;

    _redPilotLight = newValue;
    emit redPilotLightChanged(newValue);
}

bool MainBackendHelper::getAmberPilotLight() const
{
    return _amberPilotLight;
}

void MainBackendHelper::setAmberPilotLight(bool newValue)
{
    if (_amberPilotLight == newValue)
        return;

    _amberPilotLight = newValue;
    emit amberPilotLightChanged(newValue);
}

bool MainBackendHelper::getGreenPilotLight() const
{
    return _greenPilotLight;
}

void MainBackendHelper::setGreenPilotLight(bool newValue)
{
    if (_greenPilotLight == newValue)
        return;

    _greenPilotLight = newValue;
    emit greenPilotLightChanged(newValue);
}

bool MainBackendHelper::getBluePilotLight() const
{
    return _bluePilotLight;
}

void MainBackendHelper::setBluePilotLight(bool newValue)
{
    if (_bluePilotLight == newValue)
        return;

    _bluePilotLight = newValue;
    emit bluePilotLightChanged(newValue);
}

bool MainBackendHelper::getWhitePilotLight() const
{
    return _whitePilotLight;
}

void MainBackendHelper::setWhitePilotLight(bool newValue)
{
    if (_whitePilotLight == newValue)
        return;

    _whitePilotLight = newValue;
    emit whitePilotLightChanged(newValue);
}
