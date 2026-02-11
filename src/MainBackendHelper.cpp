#include <QCoreApplication>
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

    _getPLCStatusTimer = std::make_unique<QTimer>();
    _publishTimer = std::make_shared<QTimer>();
    _PLCTag = std::make_unique<PLCTag>(parent, _plcAddress, _plcType);

    setupConnections();
    initializeROS2();
    startTimers();
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

void MainBackendHelper::onStartClicked()
{
    qDebug() << "MainBackendHelper::onStartClicked()";
}

void MainBackendHelper::onStartPressed()
{
    qDebug() << "MainBackendHelper::onStartPressed()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Start_PB",true);
}

void MainBackendHelper::onStartReleased()
{
    qDebug() << "MainBackendHelper::onStartReleased()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Start_PB",false);
}

void MainBackendHelper::onResetSystem()
{
    qDebug() << "MainBackendHelper::onResetSystem()";
}

void MainBackendHelper::onStopClicked()
{
    qDebug() << "MainBackendHelper::onStopClicked()";
}

void MainBackendHelper::onStopPressed()
{
    qDebug() << "MainBackendHelper::onStopPressed()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Stop_PB",true);
}

void MainBackendHelper::onStopReleased()
{
    qDebug() << "MainBackendHelper::onStopReleased()";

    _PLCTag->writePLCTag(_plcProgramName + "HMI_Stop_PB",false);
}

void MainBackendHelper::onSafetyStop()
{
    qDebug() << "MainBackendHelper::onSafetyStop()";
}

void MainBackendHelper::onMoveToHome()
{
    qDebug() << "MainBackendHelper::onMoveToHome()";
}

void MainBackendHelper::onMoveLeft()
{
    qDebug() << "MainBackendHelper::onMoveLeft()";
}

void MainBackendHelper::onMoveRight()
{
    qDebug() << "MainBackendHelper::onMoveRight()";
}

void MainBackendHelper::onMoveForward()
{
    qDebug() << "MainBackendHelper::onMoveForward()";
}

void MainBackendHelper::onMoveBack()
{
    qDebug() << "MainBackendHelper::onMoveBack()";
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








