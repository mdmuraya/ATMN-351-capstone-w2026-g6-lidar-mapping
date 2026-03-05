#include <QCoreApplication>
#include <QQmlContext>
#include <QDebug>
#include <QVariantList>




//#include "<pcl_conversions/pcl_conversions.h"
//#include "pcl/point_cloud.h"
//#include "pcl/point_types.h"

#include "lib/libplctag/include/libplctag.h"
#include "include/LIDARScanPointCloud2Geometry.hpp"
#include "mycustompointcloud.h"
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
    rclcpp::spin_some(_ros2Node);
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
    _LIDARScanPointCloud2Geometry = std::make_unique<LIDARScanPointCloud2Geometry>(qGuiApplication);
    _ros2PublishTimer = std::make_shared<QTimer>();

    qmlRegisterSingletonInstance<LIDARScanPointCloud2Geometry>("LIDARScanPointCloud2", 1, 0, "LIDARScanPointCloud2Geometry",_LIDARScanPointCloud2Geometry.get());

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

    if(_plcFamilyId == "controllogix")
    {
        _plcProgramName = "Program:MainProgram.";
    }
    else if(_plcFamilyId == "micro800")
    {
        _plcProgramName = "";
    }
    else
    {
        _plcProgramName = "";
    }
    _PLCTag->connectToPLC(_plcAddress, _plcFamilyId, _plcProgramName);
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

    // Create iterators for x, y, and z fields
    sensor_msgs::PointCloud2ConstIterator<float> iterX(pointCloud2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterY(pointCloud2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iterZ(pointCloud2, "z");

    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ) {
        float x = *iterX;
        float y = *iterY;
        float z = *iterZ;
        // Do something with x, y, z
        qDebug() << "SLLIDAR: PointCloud2 message XYZ: x=" << x << ", y=" << y << ", z=" << z;
    }

     QByteArray vertexData = QByteArray(reinterpret_cast<const char*>(pointCloud2.data.data()),
               static_cast<int>(pointCloud2.data.size()));

    _LIDARScanPointCloud2Geometry->updateData(vertexData);

    // _LIDARScanPointCloud2.clear();

    // // QByteArray vertexData;
    // // vertexData.resize(sizeof(float) * 3 * m_count);
    // // float *p = reinterpret_cast<float *>(vertexData.data());

    // // for (int var = 0; var < m_count; ++var)
    // // {
    // //     const QVector3D vertex = generateRandomVertex(-300.0f, 300.0f);
    // //     *p++ = vertex.x();
    // //     *p++ = vertex.y();
    // //     *p++ = vertex.z();
    // // }

    // _LIDARScanPointCloud2.setVertexData(vertexData);
    // _LIDARScanPointCloud2.setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);
    // _LIDARScanPointCloud2.setStride(3 * sizeof(float));
    // _LIDARScanPointCloud2.addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
    //              0,
    //              QQuick3DGeometry::Attribute::F32Type);

    //_LIDARScan2DData->setScanData(scan);

    // int count = scan->scan_time / scan->time_increment;
    // printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    // printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
    //        RAD2DEG(scan->angle_max));

    // for (int i = 0; i < count; i++) {
    //     float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    //     printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    // }

}

void HMIBackendHelper::scanSICKMultiscan100CallBack(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg)
{
    qDebug() << "HMIBackendHelper::scanSICKMultiscan100CallBack()";


    qDebug() << "sick_scan_ros2_example: pointcloud message received, size " << msg->width << " x " << msg->height;
}

void HMIBackendHelper::startTimers()
{
    qDebug() << "HMIBackendHelper::startTimers()";

    int frequency = 1; //number of times per second

    _ros2PublishTimer->start((1000/frequency));
}









