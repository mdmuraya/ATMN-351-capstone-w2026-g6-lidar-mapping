#include <QCoreApplication>
#include <QQmlContext>
#include <QDebug>
#include <QVariantList>
#include <QtNumeric>

#include "include/HMIBackendHelper.hpp"

#define REQUIRED_VERSION 2, 4, 0
#define RAD2DEG(x) ((x)*180./M_PI)

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

        if (_ros2WorkerThread.joinable())
        {
            _ros2WorkerThread.join();
        }
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
    auto message = example_interfaces::msg::String();
    message.data = QString(QString("Hello, ROS 2! ") + QDateTime::currentDateTime().toString()).toUtf8().constData();
    _ros2Publisher->publish(message);

    qDebug() << "Published:" <<  message.data;
    //rclcpp::spin_some(_ros2Node);
}

bool HMIBackendHelper::initialize(QGuiApplication *qGuiApplication)
{
    qDebug() << "HMIBackendHelper::initialize()" << QDateTime::currentDateTime();

    QObject::connect(
        &_QQmlApplicationEngine,
        &QQmlApplicationEngine::objectCreationFailed,
        qGuiApplication,
        []()
        {
            qDebug() << "HERE QQmlApplicationEngine::objectCreationFailed";
            QCoreApplication::exit(-1);
        },
        Qt::QueuedConnection);

    QMap<QString, QString> plcFamily;
    plcFamily.insert("controllogix", "Control Logix / Compact Logix" );
    plcFamily.insert("micro800", "Micro 800" );

    for (auto it = plcFamily.constBegin(); it != plcFamily.constEnd(); ++it) {
        QVariantMap itemMap;
        itemMap["plcFamilyId"] = it.key();
        itemMap["plcFamilyDescription"] = it.value();
        _listOfPLCFamily.append(itemMap);
    }

    _PLCTag = std::make_unique<PLCTag>(this);
    _LIDARScan2DData = std::make_unique<LIDARScan2DData>(this);
    _LIDARScanPointCloud2Geometry = std::make_unique<LIDARScanPointCloud2Geometry>();
    _DEMSurface = std::make_unique<DEMSurface>();
    _ros2PublishTimer = std::make_shared<QTimer>();

    qmlRegisterSingletonInstance<LIDARScanPointCloud2Geometry>("LIDARScanPointCloud2", 1, 0, "LIDARScanPointCloud2Geometry",_LIDARScanPointCloud2Geometry.get());
    qmlRegisterSingletonInstance<DEMSurface>("LIDARScanPointCloud2", 1, 0, "DEMSurface",_DEMSurface.get());

    _QQmlApplicationEngine.rootContext()->setContextProperty("plcTag", _PLCTag.get());
    _QQmlApplicationEngine.rootContext()->setContextProperty("lidarScan2DData", _LIDARScan2DData.get());
    _QQmlApplicationEngine.rootContext()->setContextProperty("hmiBackendHelper", this);

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

QVariantList HMIBackendHelper::getListOfPLCFamily() const
{
    return _listOfPLCFamily;
}

QString HMIBackendHelper::getPLCAddress() const
{
    return _plcAddress;
}

QString HMIBackendHelper::getPLCFamilyId() const
{
    return _plcFamilyId;
}

void HMIBackendHelper::setPLCAddress(QString newValue)
{
    if (_plcAddress == newValue)
        return;

    _plcAddress = newValue;
    emit plcAddressChanged(_plcAddress); // Emit signal to trigger QML updates
}

void HMIBackendHelper::setPLCFamilyId(QString newValue)
{
    if (_plcFamilyId == newValue)
        return;

    _plcFamilyId = newValue;
    emit plcFamilyIdChanged(_plcFamilyId); // Emit signal to trigger QML updates
}

void HMIBackendHelper::connectToPLC()
{
    qDebug() << "HMIBackendHelper::onConnectToPLC()" << QDateTime::currentDateTime();

    qDebug() << "HMIBackendHelper::onConnectToPLC()" << _plcFamilyId  << _plcAddress;

    _plcAddress = "192.168.1.102";

    if(_plcFamilyId == "controllogix")
    {
        _plcMainProgramName = "Program:MainProgram.";
        _plcSafetyProgramName = "Program:SafetyProgram.";
    }
    else if(_plcFamilyId == "micro800")
    {
        _plcMainProgramName = "";
        _plcSafetyProgramName = "";
    }
    else
    {
        _plcMainProgramName = "";
        _plcSafetyProgramName = "";
    }
    _PLCTag->connectToPLC(_plcAddress, _plcFamilyId, _plcMainProgramName, _plcSafetyProgramName);
}


void HMIBackendHelper::disconnectFromPLC()
{
    qDebug() << "HMIBackendHelper::disconnectFromPLC()" << QDateTime::currentDateTime();

    qDebug() << "HMIBackendHelper::disconnectFromPLC()" << _plcFamilyId  << _plcAddress;


    _PLCTag->disconnectFromPLC();
}



void HMIBackendHelper::initializeROS2()
{
    rclcpp::init(0, nullptr);
    _ros2Node = rclcpp::Node::make_shared("LIDAR_Mapping_HMI");
    _ros2Publisher = _ros2Node->create_publisher<example_interfaces::msg::String>("LIDAR_Mapping_HMI_topic", 10);

    _ros2LIDARScannerSubscription = _ros2Node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&HMIBackendHelper::scanCallBack, this, std::placeholders::_1));

    _ros2SICKMultiscan100LIDARScannerSubscription = _ros2Node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_unstructured_fullframe",
        10,
        std::bind(&HMIBackendHelper::scanSICKMultiscan100CallBack, this, std::placeholders::_1));

    _ros2WorkerThread = std::thread([this]() {
        rclcpp::spin(_ros2Node);
    });

}

void HMIBackendHelper::setupConnections()
{
    qDebug() << "HMIBackendHelper::setupConnections()";

    connect(_ros2PublishTimer.get(), &QTimer::timeout, [this](){
        publishToROS2();
    });

    //connect(this, &HMIBackendHelper::timeToPublish, this, &HMIBackendHelper::onTimeToPublish);

}


void HMIBackendHelper::scanCallBack(sensor_msgs::msg::LaserScan::SharedPtr scan) {

    qDebug() << "HMIBackendHelper::scanCallBack()";

    sensor_msgs::msg::PointCloud2 pointCloud2;

    qInfo() << "START: LaserScan to  PointCloud2";

    _LaserProjection.projectLaser(*scan, pointCloud2);

    qInfo() << "DONE: LaserScan to  PointCloud2";

    qDebug() << "SLLIDAR: PointCloud2 message received, size " << pointCloud2.width << " x " << pointCloud2.height;

    QVector<QVector3D> points;
    points.reserve(pointCloud2.width * pointCloud2.height);

    // Create iterators for x, y, and z fields
    sensor_msgs::PointCloud2ConstIterator<float> iterX(pointCloud2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterY(pointCloud2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iterZ(pointCloud2, "z");

    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ)
    {
        float x = (*iterX) * 20;
        float y = (*iterY) * 20;
        float z = (*iterZ) * 20;
        // Do something with x, y, z
        qDebug() << "SLLIDAR: PointCloud2 message XYZ: x=" << x << ", y=" << y << ", z=" << z;

        points.append(QVector3D(x, y, z));
    }

    emit pointCloudReady(points);
    _DEMSurface->updatePoints(points);
    _LIDARScanPointCloud2Geometry->updatePoints(points);

}

void HMIBackendHelper::scanSICKMultiscan100CallBack(const std::shared_ptr<sensor_msgs::msg::PointCloud2> pointCloud2)
{
    //qDebug() << "HMIBackendHelper::scanSICKMultiscan100CallBack()";


    //qDebug() << "sick_scan_ros2_example: pointcloud message received, size " << pointCloud2->width << " x " << pointCloud2->height;

    if(_PLCTag->getRunStateSCAN() && !(_PLCTag->getEndLimitSwitch()))
    {
        QVector<QVector3D> points;
        points.reserve(pointCloud2->width * pointCloud2->height);

        // Create iterators for x, y, and z fields
        sensor_msgs::PointCloud2ConstIterator<float> iterX(*pointCloud2, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iterY(*pointCloud2, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iterZ(*pointCloud2, "z");

        for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ)
        {
            float x = (*iterX) * 20;
            float y = (*iterY) * 20;
            float z = (*iterZ) * 20;
            // Do something with x, y, z

            // if(((x) < 0) )
            //     continue;

            if((x < 0) || (x > 20) || (y > 20) || (z > 20))
                continue;

            //qDebug() << "sick_scan_ros2_example: PointCloud2 message XYZ: x=" << x << ", y=" << y << ", z=" << z;

            points.append(QVector3D(x, y, z));
            m_points.append(QVector3D(x, y, z));
        }

        emit pointCloudReady(points);
        _DEMSurface->updatePoints(points);
        _LIDARScanPointCloud2Geometry->updatePoints(m_points);
    }



}


void HMIBackendHelper::startDataCaptureButtonClicked()
{
    qDebug() << "HMIBackendHelper::startDataCaptureButtonClicked()";

    clearDataCaptureButtonClicked();
}

void HMIBackendHelper::stopDataCaptureButtonClicked()
{
    qDebug() << "HMIBackendHelper::stopDataCaptureButtonClicked()";

    //_LIDARScanPointCloud2Geometry->updatePoints(m_points);

}

void HMIBackendHelper::clearDataCaptureButtonClicked()
{
    qDebug() << "HMIBackendHelper::clearDataCaptureButtonClicked()";

    m_points.clear();
    //_LIDARScanPointCloud2Geometry->updatePoints(m_points);
}

void HMIBackendHelper::startTimers()
{
    qDebug() << "HMIBackendHelper::startTimers()";

    int frequency = 1; //number of times per second

    //_ros2PublishTimer->start((1000/frequency));
}









