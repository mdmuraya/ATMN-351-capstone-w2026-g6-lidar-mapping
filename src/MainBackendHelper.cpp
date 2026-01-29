#include <QCoreApplication>
#include <QDebug>
#include <QVariantList>

#include "include/MainBackendHelper.h"
#include "include/libplctag.h"

#define REQUIRED_VERSION 2, 4, 0

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

bool MainBackendHelper::getPLCRunTag() const
{
    return _plcRunTagValue;
}

void MainBackendHelper::setPLCRunTag(bool plcRunTagValue)
{
    if (_plcRunTagValue == plcRunTagValue)
        return;
    _plcRunTagValue = plcRunTagValue;
    emit plcRunTagChanged(_plcRunTagValue); // Emit signal to trigger QML updates
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

    int i = 0, rc = 0, elementCount = 1, elementSize = 1, dataTimeout = 5000;
    QString plcTagPath = "protocol=ab-eip&gateway=192.168.40.62&plc=Micro800&elem_size=1&elem_count=1&name=HMI_Start_PB";
    auto tag = plc_tag_create(plcTagPath.toUtf8().constData(), dataTimeout);

    qDebug() << plcTagPath;

    /* everything OK? */
    if(tag < 0) {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return;
    }

    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK) {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }

    /* get the data */
    rc = plc_tag_read(tag, dataTimeout);
    if(rc != PLCTAG_STATUS_OK) {
        qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }
    qDebug() << QString::number(tag);
    /* print out the data */
    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "BEFORE data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    /* now test a write */
    for(i = 0; i < elementCount; i++) {
        int32_t val = plc_tag_get_bit(tag, (i * elementSize));

        val = 1;

        qDebug() << "Setting element" <<  i << " to" << val;

        plc_tag_set_bit(tag, (i * elementSize), val);
    }

    rc = plc_tag_write(tag, dataTimeout);

    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "AFTER data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    plc_tag_destroy(tag);
}

void MainBackendHelper::onStartReleased()
{
    qDebug() << "MainBackendHelper::onStartReleased()";

    int i = 0, rc = 0, elementCount = 1, elementSize = 1, dataTimeout = 5000;
    QString plcTagPath = "protocol=ab-eip&gateway=192.168.40.62&plc=Micro800&elem_size=1&elem_count=1&name=HMI_Start_PB";
    auto tag = plc_tag_create(plcTagPath.toUtf8().constData(), dataTimeout);

    qDebug() << plcTagPath;

    /* everything OK? */
    if(tag < 0) {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return;
    }

    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK) {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }

    /* get the data */
    rc = plc_tag_read(tag, dataTimeout);
    if(rc != PLCTAG_STATUS_OK) {
        qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }
    qDebug() << QString::number(tag);
    /* print out the data */
    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "BEFORE data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    /* now test a write */
    for(i = 0; i < elementCount; i++) {
        int32_t val = plc_tag_get_bit(tag, (i * elementSize));

        val = 0;

        qDebug() << "Setting element" <<  i << " to" << val;

        plc_tag_set_bit(tag, (i * elementSize), val);
    }

    rc = plc_tag_write(tag, dataTimeout);

    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "AFTER data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    plc_tag_destroy(tag);
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

    int i = 0, rc = 0, elementCount = 1, elementSize = 1, dataTimeout = 5000;
    QString plcTagPath = "protocol=ab-eip&gateway=192.168.40.62&plc=Micro800&elem_size=1&elem_count=1&name=HMI_Stop_PB";
    auto tag = plc_tag_create(plcTagPath.toUtf8().constData(), dataTimeout);

    qDebug() << plcTagPath;

    /* everything OK? */
    if(tag < 0) {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return;
    }

    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK) {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }

    /* get the data */
    rc = plc_tag_read(tag, dataTimeout);
    if(rc != PLCTAG_STATUS_OK) {
        qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }
    qDebug() << QString::number(tag);
    /* print out the data */
    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "BEFORE data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    /* now test a write */
    for(i = 0; i < elementCount; i++) {
        int32_t val = plc_tag_get_bit(tag, (i * elementSize));

        val = 1;

        qDebug() << "Setting element" <<  i << " to" << val;

        plc_tag_set_bit(tag, (i * elementSize), val);
    }

    rc = plc_tag_write(tag, dataTimeout);

    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "AFTER data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    plc_tag_destroy(tag);
}

void MainBackendHelper::onStopReleased()
{
    qDebug() << "MainBackendHelper::onStopReleased()";

    int i = 0, rc = 0, elementCount = 1, elementSize = 1, dataTimeout = 5000;
    QString plcTagPath = "protocol=ab-eip&gateway=192.168.40.62&plc=Micro800&elem_size=1&elem_count=1&name=HMI_Stop_PB";
    auto tag = plc_tag_create(plcTagPath.toUtf8().constData(), dataTimeout);

    qDebug() << plcTagPath;

    /* everything OK? */
    if(tag < 0) {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return;
    }

    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK) {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }

    /* get the data */
    rc = plc_tag_read(tag, dataTimeout);
    if(rc != PLCTAG_STATUS_OK) {
        qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }
    qDebug() << QString::number(tag);
    /* print out the data */
    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "BEFORE data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    /* now test a write */
    for(i = 0; i < elementCount; i++) {
        int32_t val = plc_tag_get_bit(tag, (i * elementSize));

        val = 0;

        qDebug() << "Setting element" <<  i << " to" << val;

        plc_tag_set_bit(tag, (i * elementSize), val);
    }

    rc = plc_tag_write(tag, dataTimeout);

    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "AFTER data[" <<  i << "]=" << plc_tag_get_bit(tag, (i * elementSize));
    }

    plc_tag_destroy(tag);
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

    int i = 0, rc = 0, elementCount = 1, elementSize = 1, dataTimeout = 5000;
    QString plcTagPath = "protocol=ab-eip&gateway=192.168.40.62&plc=Micro800&elem_size=1&elem_count=1&name=PLC_RUN";
    auto tag = plc_tag_create(plcTagPath.toUtf8().constData(), dataTimeout);

    qDebug() << plcTagPath;

    /* everything OK? */
    if(tag < 0) {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return;
    }

    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK) {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }

    /* get the data */
    rc = plc_tag_read(tag, dataTimeout);
    if(rc != PLCTAG_STATUS_OK) {
        qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }
    qDebug() << QString::number(tag);
    int32_t val = plc_tag_get_bit(tag, 0);
    qDebug() << "CURRENT PLC_RUN [" <<  val << "]";

    setPLCRunTag(static_cast<bool>(val));



    /*
    int i = 0, rc = 0, elementCount = 10, elementSize = 4, dataTimeout = 5000;
    QString plcTagPath = "protocol=ab-eip&gateway=192.168.40.62&plc=Micro800&elem_size=1&elem_count=1&name=DENNIS_TAG";
    auto tag = plc_tag_create(plcTagPath.toUtf8().constData(), dataTimeout);

    qDebug() << plcTagPath;
    qDebug() << QString::number(tag);


    if(tag < 0) {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return;
    }

    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK) {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }


    rc = plc_tag_read(tag, dataTimeout);
    if(rc != PLCTAG_STATUS_OK) {
        qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return;
    }
    qDebug() << QString::number(tag);

    for(i = 0; i < elementCount; i++)
    {
        //fprintf(stderr, "data[%d]=%d\n", i, plc_tag_get_int32(tag, (i * ELEM_SIZE)));
        qDebug() << "data[" <<  i << "]=" << plc_tag_get_int32(tag, (i * elementSize));
    }


    for(i = 0; i < elementCount; i++) {
        int32_t val = plc_tag_get_int32(tag, (i * elementSize));

        val = val + 1;

        qDebug() << "Setting element" <<  i << " to" << val;

        plc_tag_set_int32(tag, (i * elementSize), val);
    }

    rc = plc_tag_write(tag, dataTimeout);

    plc_tag_destroy(tag);
*/
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







