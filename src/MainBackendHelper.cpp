#include <QCoreApplication>
#include <QDebug>
#include <QVariantList>

#include "include/MainBackendHelper.h"


MainBackendHelper::MainBackendHelper(QObject *parent) :
    QObject (parent),
    _getPLCStatusTimer (std::make_unique<QTimer>()),
    _publishTimer (std::make_shared<QTimer>())
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

void MainBackendHelper::onConnectToPLC()
{
    qDebug() << "MainBackendHelper::onConnectToPLC()";
}

void MainBackendHelper::onStartScan()
{
    qDebug() << "MainBackendHelper::onStartScan()";
}

void MainBackendHelper::onResetSystem()
{
    qDebug() << "MainBackendHelper::onResetSystem()";
}

void MainBackendHelper::onStopScan()
{
    qDebug() << "MainBackendHelper::onStopScan()";
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

void MainBackendHelper::onRequestPLCStatus()
{
    qDebug() << "MainBackendHelper::onRequestPLCStatus()" << QDateTime::currentDateTime();
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
    _ros2Node = std::make_shared<rclcpp::Node>("LIDARMapping_HMI_App");
    _publisher = _ros2Node->create_publisher<std_msgs::msg::String>("LIDARMapping_HMI_App_topic", 10);

}

void MainBackendHelper::setupConnections()
{
    qDebug() << "MainBackendHelper::setupConnections()";

    connect(_getPLCStatusTimer.get(), &QTimer::timeout, [this](){
        emit requestPLCStatus();
    });

    connect(_publishTimer.get(), &QTimer::timeout, [this](){
        emit timeToPublish();
    });

    connect(this, &MainBackendHelper::requestPLCStatus, this, &MainBackendHelper::onRequestPLCStatus);
    connect(this, &MainBackendHelper::timeToPublish, this, &MainBackendHelper::onTimeToPublish);

}

void MainBackendHelper::startTimers()
{
    qDebug() << "MainBackendHelper::startTimers()";

    int frequency = 1; //number of times per second
    _getPLCStatusTimer->start((1000/frequency));

    frequency= 1; //number of times per second
    _publishTimer->start((1000/frequency));
}







