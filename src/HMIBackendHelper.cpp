#include <QCoreApplication>
#include <QQmlContext>
#include <QDebug>
#include <QVariantList>

#include "lib/libplctag/include/libplctag.h"
#include "include/HMIBackendHelper.hpp"


#define REQUIRED_VERSION 2, 4, 0

HMIBackendHelper::HMIBackendHelper(QObject *parent) : QObject (parent)
{
    qDebug() << "HMIBackendHelper::HMIBackendHelper()";
    qDebug() << "Total arguments:" << QCoreApplication::arguments().count();

    for (int i = 0; i < QCoreApplication::arguments().count(); ++i) {
        qDebug() << "Argument" << i << ":" << QCoreApplication::arguments().at(i);
    }

    // Access specific arguments (e.g., the second argument if it exists)
    if (QCoreApplication::arguments().count() > 1) {
        qDebug() << "Second argument:" << QCoreApplication::arguments().at(1);
    }
}

HMIBackendHelper::~HMIBackendHelper()
{
    qDebug() << "HMIBackendHelper::~HMIBackendHelper()";

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void HMIBackendHelper::publishToROS2()
{
    qDebug() << "HMIBackendHelper::publishToROS2()" << QDateTime::currentDateTime();

    if (!rclcpp::ok()) {
        qDebug() << "ROS is not running!";
        _ros2PublishTimer->stop();
        return;
    }

    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2! ";
    _ros2Publisher->publish(message);

    qDebug() << "Published:" <<  message.data;
    rclcpp::spin_some(_ros2Node);
}

bool HMIBackendHelper::initialize(QGuiApplication *qGuiApplication)
{
    qDebug() << "HMIBackendHelper::initialize()" << QDateTime::currentDateTime();

    QObject::connect(
        &_QQmlApplicationEngine,
        &QQmlApplicationEngine::objectCreationFailed,
        qGuiApplication,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    _PLCTag = std::make_unique<PLCTag>(this, _plcAddress, _plcType, _plcProgramName);
    _ros2PublishTimer = std::make_shared<QTimer>();

    _QQmlApplicationEngine.rootContext()->setContextProperty("plcTag", _PLCTag.get());
    _QQmlApplicationEngine.rootContext()->setContextProperty("HMIBackendHelper", this);

    _QQmlApplicationEngine.loadFromModule("LIDAR_Mapping", "HMI");

    if(_QQmlApplicationEngine.rootObjects().isEmpty())
    {
        return false;
    }

    setupConnections();
    initializeROS2();
    startTimers();

    return true;
}

void HMIBackendHelper::onConnectToPLC()
{

}

void HMIBackendHelper::initializeROS2()
{
    rclcpp::init(0, nullptr);
    _ros2Node = rclcpp::Node::make_shared("LIDARMapping_HMI");
    _ros2Publisher = _ros2Node->create_publisher<std_msgs::msg::String>("LIDARMapping_HMI_topic", 10);

}

void HMIBackendHelper::setupConnections()
{
    qDebug() << "HMIBackendHelper::setupConnections()";

    connect(_ros2PublishTimer.get(), &QTimer::timeout, [this](){
        publishToROS2();
    });

    //connect(this, &HMIBackendHelper::timeToPublish, this, &HMIBackendHelper::onTimeToPublish);

}

void HMIBackendHelper::startTimers()
{
    qDebug() << "HMIBackendHelper::startTimers()";

    int frequency = 1; //number of times per second

    _ros2PublishTimer->start((1000/frequency));
}









