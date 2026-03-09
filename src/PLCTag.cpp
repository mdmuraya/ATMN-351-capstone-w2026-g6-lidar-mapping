#include <QDebug>
#include <QFutureSynchronizer>
#include <QtConcurrent>
#include <QTime>
#include <QTimer>

#include "include/PLCTag.hpp"
#include "lib/libplctag/include/libplctag.h"

PLCTag::PLCTag(QObject *parent) : QObject (parent)
{
    qDebug() << "PLCTag::PLCTag()";

    _getPLCStatusTimer = std::make_unique<QTimer>();

    connect(_getPLCStatusTimer.get(), &QTimer::timeout, [this](){
        getPLCStatus();
    });

}

PLCTag::~PLCTag()
{
    qDebug() << "PLCTag::~PLCTag()";

    for (auto [tagName, tag] : _PLCTags.asKeyValueRange())
    {
        qDebug() << "DESTRYOING Tag: Name:" << tagName << "Value:" << tag;
        plc_tag_destroy(tag);
    }
    _PLCTags.clear();
}

void PLCTag::connectToPLC(QString plcAddress, QString plcFamilyId, QString plcMainProgramName, QString plcSafetyProgramName)
{
    _plcAddress = plcAddress;
    _plcFamilyId = plcFamilyId;
    _plcMainProgramName = plcMainProgramName;
    _plcSafetyProgramName = plcSafetyProgramName;

    int frequency = 5; //number of times per second
    _getPLCStatusTimer->start((1000/frequency));
}

void PLCTag::disconnectFromPLC()
{
    setPLCIsConnected(false);
    _getPLCStatusTimer->stop();
}

int32_t PLCTag::getPLCTag(QString tagName)
{
    //qDebug() << "PLCTag::getPLCTag()" << QDateTime::currentDateTime();

    if (_PLCTags.contains(tagName)) {
        //qDebug() << "Key" << tagName << " found.";
        return _PLCTags.value(tagName);
    }

    //qDebug() << "Key " << tagName << " NOT FOUND. Creating...";


    QString plcPath = (QString::compare(_plcFamilyId, "controllogix", Qt::CaseInsensitive) == 0 ) ? QString("&path=1,0") : "";
    QString plcTagPath = QString("protocol=ab-eip&gateway=") + _plcAddress + plcPath + QString("&plc=") + _plcFamilyId + QString("&elem_size=1&elem_count=1&name=") + tagName;

    int32_t tag = plc_tag_create(plcTagPath.toUtf8().constData(), 5000 /*wait for a maximumm of 5 seconds*/);

    qDebug() << plcTagPath;

    /* everything OK? */
    if(tag < 0)
    {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return 0;
    }

    int rc = 0;
    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK)
    {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return 0;
    }

    _PLCTags.insert(tagName, tag);
    return _PLCTags.value(tagName);

}

bool PLCTag::readPLCTag(QString tagName, bool &tagValue)
{
    //qDebug() << "PLCTag::readPLCTag()" << QDateTime::currentDateTime();

    int32_t tag = getPLCTag(tagName);

    if(tag > 0)
    {
        /* get the data */
        int rc = plc_tag_read(tag, _getPLCStatusTimer->interval());
        if(rc != PLCTAG_STATUS_OK)
        {
            qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
            _PLCTags.remove(tagName);
            plc_tag_destroy(tag);
            return false;
        }
        //qDebug() << QString::number(tag);
        tagValue = static_cast<bool>(plc_tag_get_bit(tag, 0));

        return true;
    }

    return false;
}

uint64_t PLCTag::readPLCTag(QString tagName, uint64_t &tagValue)
{
    //qDebug() << "PLCTag::readPLCTag()" << QDateTime::currentDateTime();

    int32_t tag = getPLCTag(tagName);


    if(tag > 0)
    {
        /* get the data */
        int rc = plc_tag_read(tag, _getPLCStatusTimer->interval());
        if(rc != PLCTAG_STATUS_OK)
        {
            qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
            _PLCTags.remove(tagName);
            plc_tag_destroy(tag);
            return false;
        }
        //qDebug() << QString::number(tag);
        tagValue = (plc_tag_get_uint64(tag, 0));

        return true;
    }

    return false;
}

bool PLCTag::writePLCTag(QString tagName, bool tagValue)
{
    qDebug() << "PLCTag::writePLCTag()" << QDateTime::currentDateTime();

    int32_t tag = getPLCTag(tagName);

    if(tag > 0)
    {
        plc_tag_set_bit(tag, 0, static_cast<int32_t>(tagValue));

        if(plc_tag_write(tag, 3000 /*wait for a maximumm of 3 seconds*/) != PLCTAG_STATUS_OK) {
            //plc_tag_destroy(tag);
            qDebug() << "PLCTag::writePLCTag() FAIL";
            return false;
        }
        //plc_tag_destroy(tag);
        qDebug() << "PLCTag::writePLCTag() SUCCESS";

        return true;
    }

    qDebug() << "PLCTag::writePLCTag() FAIL";
    return false;
}

void PLCTag::getPLCStatus()
{
    //qDebug() << "PLCTag::getPLCStatus()" << QDateTime::currentDateTime();
    //here we will get all the PLC tags, in threads (concurrently)
    //Program:SafetyProgram.PHY_ESTOP_ACTIVATED
    _getPLCStatusTimer->stop();

    //bool boolTagValue = false;
    uint64_t uint64TagValue = 0;

    if(readPLCTag(_plcMainProgramName + "PLC_Heart_Beat", uint64TagValue))
    {
        setPLCIsConnected(true);
        qDebug() << "PLC_Heart_Beat" << uint64TagValue;

        QFutureSynchronizer<void> synchronizer;
        QFuture<void> future;

        future = QtConcurrent::run([this]() {
            bool boolTagValue = getEStop1Activated();
            readPLCTag(_plcSafetyProgramName + "PHY_ESTOP_1_ACTIVATED", boolTagValue) ? setEStop1Activated(boolTagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool boolTagValue = getRunState();
            readPLCTag(_plcMainProgramName + "System_Running", boolTagValue) ? setRunState(boolTagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool boolTagValue =getRunStateSCAN();
            readPLCTag(_plcMainProgramName + "PHY_Selector_Run_SCAN", boolTagValue) ? setRunStateSCAN(boolTagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool boolTagValue =getRedPilotLight();
            readPLCTag(_plcMainProgramName + "Red_Pilot_Light", boolTagValue) ? setRedPilotLight(boolTagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool boolTagValue = getAmberPilotLight();
            readPLCTag(_plcMainProgramName + "Amber_Pilot_Light", boolTagValue) ? setAmberPilotLight(boolTagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool boolTagValue = getGreenPilotLight();
            readPLCTag(_plcMainProgramName + "Green_Pilot_Light", boolTagValue) ? setGreenPilotLight(boolTagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool boolTagValue = getBluePilotLight();
            readPLCTag(_plcMainProgramName + "Blue_Pilot_Light", boolTagValue) ? setBluePilotLight(boolTagValue) :  (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool boolTagValue = getWhitePilotLight();
            readPLCTag(_plcMainProgramName + "White_Pilot_Light", boolTagValue) ? setWhitePilotLight(boolTagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

    }
    else
    {
        setPLCIsConnected(false);
    }

    _getPLCStatusTimer->start();

}


// QString PLCTag::getPLCAddress() const
// {
//     return _plcAddress;
// }

bool PLCTag::getPLCIsConnected() const
{
    return _plcIsConnected;
}

void PLCTag::setPLCIsConnected(bool newValue)
{
    if (_plcIsConnected == newValue)
        return;

    _plcIsConnected = newValue;
    emit plcIsConnectedChanged(_plcIsConnected); // Emit signal to trigger QML updates
}


bool PLCTag::getEStop1Activated() const
{
    return _eStop1Activated;
}

void PLCTag::setEStop1Activated(bool newValue)
{
    if (_eStop1Activated == newValue)
        return;

    _eStop1Activated = newValue;
    emit eStop1ActivatedChanged(_eStop1Activated); // Emit signal to trigger QML updates
}

bool PLCTag::getRunState() const
{
    return _runState;
}

void PLCTag::setRunState(bool newValue)
{
    if (_runState == newValue)
        return;

    _runState = newValue;
    emit runStateChanged(_runState); // Emit signal to trigger QML updates
}

bool PLCTag::getRunStateSCAN() const
{
    return _runStateSCAN;
}

void PLCTag::setRunStateSCAN(bool newValue)
{
    if (_runStateSCAN == newValue)
        return;

    _runStateSCAN = newValue;
    emit runStateSCANChanged(_runStateSCAN); // Emit signal to trigger QML updates
}

void PLCTag::startButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::startButtonPressedChanged()";

    writePLCTag(_plcMainProgramName + "HMI_Start_PB",pressed);
}

void PLCTag::stopButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::stopButtonPressedChanged()";

    writePLCTag(_plcMainProgramName + "HMI_Stop_PB",pressed);
}

void PLCTag::resetButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::resetButtonPressedChanged()";

    writePLCTag(_plcSafetyProgramName + "HMI_Reset_PB",pressed);
}

void PLCTag::moveToHomeButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveToHomeButtonPressedChanged()";

    writePLCTag(_plcMainProgramName + "HMI_Home_PB",pressed);
}

void PLCTag::moveLeftButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveLeftButtonPressedChanged()";

    writePLCTag(_plcMainProgramName + "HMI_MoveLeft_PB",pressed);
}

void PLCTag::moveBackButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveBackButtonPressedChanged()";

    writePLCTag(_plcMainProgramName + "HMI_MoveBack_PB",pressed);
}

void PLCTag::moveForwardButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveForwardButtonPressedChanged()";

    writePLCTag(_plcMainProgramName + "HMI_MoveForward_PB",pressed);
}

void PLCTag::moveRightButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveRightButtonPressedChanged()";

    writePLCTag(_plcMainProgramName + "HMI_MoveRight_PB",pressed);
}


bool PLCTag::getRedPilotLight() const
{
    return _redPilotLight;
}

void PLCTag::setRedPilotLight(bool newValue)
{
    if (_redPilotLight == newValue)
        return;

    _redPilotLight = newValue;
    emit redPilotLightChanged(_redPilotLight);
}

bool PLCTag::getAmberPilotLight() const
{
    return _amberPilotLight;
}

void PLCTag::setAmberPilotLight(bool newValue)
{
    if (_amberPilotLight == newValue)
        return;

    _amberPilotLight = newValue;
    emit amberPilotLightChanged(_amberPilotLight);
}

bool PLCTag::getGreenPilotLight() const
{
    return _greenPilotLight;
}

void PLCTag::setGreenPilotLight(bool newValue)
{
    if (_greenPilotLight == newValue)
        return;

    _greenPilotLight = newValue;
    emit greenPilotLightChanged(_greenPilotLight);
}

bool PLCTag::getBluePilotLight() const
{
    return _bluePilotLight;
}

void PLCTag::setBluePilotLight(bool newValue)
{
    if (_bluePilotLight == newValue)
        return;

    _bluePilotLight = newValue;
    emit bluePilotLightChanged(_bluePilotLight);
}

bool PLCTag::getWhitePilotLight() const
{
    return _whitePilotLight;
}

void PLCTag::setWhitePilotLight(bool newValue)
{
    if (_whitePilotLight == newValue)
        return;

    _whitePilotLight = newValue;
    emit whitePilotLightChanged(_whitePilotLight);
}

/*
    int i = 0, rc = 0, elementCount = 10, elementSize = 4, dataTimeout = 5000;
    QString plcTagPath = "protocol=ab-eip&gateway=&plc=Micro800&elem_size=1&elem_count=1&name=DENNIS_TAG";
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








