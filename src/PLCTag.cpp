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

    for (auto [tagName, tag] : m_PLCTags.asKeyValueRange())
    {
        qDebug() << "DESTRYOING Tag: Name:" << tagName << "Value:" << tag;
        plc_tag_destroy(tag);
        plc_tag_unregister_callback(tag);
    }
    m_PLCTags.clear();
}

void PLCTag::connectToPLC(QString plcAddress, QString plcFamilyId, QString plcMainProgramName, QString plcSafetyProgramName)
{
    m_plcAddress = plcAddress;
    m_plcFamilyId = plcFamilyId;
    m_plcMainProgramName = plcMainProgramName;
    m_plcSafetyProgramName = plcSafetyProgramName;

    int frequency = 5; //number of times per second
    _getPLCStatusTimer->start((1000/frequency));
    //getPLCTag(_plcMainProgramName + "PLC_Heart_Beat");
}

// void PLCTag::eventCallback(int32_t tagId, int eventId, int status, void *userdata)
// {
//     qDebug() << "PLCTag::eventCallback()" << QDateTime::currentDateTime();
//     QString plcTtag = *((QString*)userdata);
//     qDebug() << "PLCTag::eventCallback()" << tagId << eventId << status  << plcTtag;

//     switch (eventId)
//     {
//         case PLCTAG_EVENT_READ_STARTED:
//             qDebug() << "PLCTag::eventCallback()" << "PLCTAG_EVENT_READ_STARTED";
//             break;
//         case PLCTAG_EVENT_READ_COMPLETED:
//             qDebug() << "PLCTag::eventCallback()" << "PLCTAG_EVENT_READ_COMPLETED" << plcTtag;
//             updateHMIFromPLCTag(plcTtag);
//             break;
//         case PLCTAG_EVENT_WRITE_STARTED:
//             qDebug() << "PLCTag::eventCallback()" << "PLCTAG_EVENT_WRITE_STARTED";
//             break;
//         case PLCTAG_EVENT_WRITE_COMPLETED:
//             qDebug() << "PLCTag::eventCallback()" << "PLCTAG_EVENT_WRITE_COMPLETED";
//             break;
//         case PLCTAG_EVENT_ABORTED:
//             qDebug() << "PLCTag::eventCallback()" << "PLCTAG_EVENT_ABORTED";
//             break;
//         case PLCTAG_EVENT_DESTROYED:
//             qDebug() << "PLCTag::eventCallback()" << "PLCTAG_EVENT_DESTROYED";
//             break;
//         case PLCTAG_EVENT_CREATED:
//             qDebug() << "PLCTag::eventCallback()" << "PLCTAG_EVENT_CREATED";
//             break;
//         default:
//             qDebug() << "PLCTag::eventCallback()" << "DEFAULT";
//             break;
//     }
// }

void PLCTag::disconnectFromPLC()
{
    setPLCIsConnected(false);
    _getPLCStatusTimer->stop();
}

int32_t PLCTag::getPLCTag(QString tagName, uint32_t elementSize)
{
    //qDebug() << "PLCTag::getPLCTag()" << QDateTime::currentDateTime();

    if (m_PLCTags.contains(tagName)) {
        //qDebug() << "Key" << tagName << " found.";
        return m_PLCTags.value(tagName);
    }

    qDebug() << "Key " << tagName << " NOT FOUND. Creating...";


    QString plcPath = (QString::compare(m_plcFamilyId, "controllogix", Qt::CaseInsensitive) == 0 ) ? QString("&path=1,0") : "";
    QString plcTagPath = QString("protocol=ab-eip&gateway=") + m_plcAddress + plcPath + QString("&plc=") + m_plcFamilyId + QString("&elem_size=") + QString::number(elementSize) + QString("&elem_count=1&name=") + tagName;

    int32_t tag = plc_tag_create(plcTagPath.toUtf8().constData(), 5000 /*wait for a maximumm of 5 seconds*/);

    qDebug() << plcTagPath;

    /* everything OK? */
    if(tag < 0)
    {
        qDebug() << "ERROR" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Could not create tag!";
        return 0;
    }
    qDebug() << "SUCCESS" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Tag created";

    int rc = 0;
    if((rc = plc_tag_status(tag)) != PLCTAG_STATUS_OK)
    {
        qDebug() << "Error setting up tag internal state. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
        plc_tag_destroy(tag);
        return 0;
    }
    qDebug() << "SUCCESS" << QString::fromUtf8(plc_tag_decode_error(tag)) << ": Tag status OK";

    m_PLCTags.insert(tagName, tag);
    return m_PLCTags.value(tagName);

    // QHash<QString, int>::iterator it = _PLCTags.find(tagName);
    // // Check if the key was found
    // if (it != _PLCTags.end()) {
    //     // Get a const reference to the key, then take its address (pointer)
    //     const QString* keyPointer = &(it.key());
    //     qDebug() << "TUNA-GET" << (*keyPointer);

    //     if((rc = plc_tag_register_callback_ex(tag, &PLCTag::eventCallback, (void*)keyPointer)) != PLCTAG_STATUS_OK)
    //     {
    //         qDebug() << "Error setting up callback. Error" << QString::fromUtf8(plc_tag_decode_error(rc));
    //         _PLCTags.remove(tagName);
    //         plc_tag_destroy(tag);
    //         return 0;
    //     }

    //     return _PLCTags.value(tagName);
    // }

    // plc_tag_destroy(tag);
    // return 0;

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
            m_PLCTags.remove(tagName);
            plc_tag_destroy(tag);
            return false;
        }
        //qDebug() << QString::number(tag);
        tagValue = static_cast<bool>(plc_tag_get_bit(tag, 0));

        return true;
    }

    return false;
}

bool PLCTag::readPLCTag(QString tagName, uint32_t elementSize, StepperMotor_AZD_AEP_t &tagValue)
{
    //qDebug() << "PLCTag::readPLCTag()" << QDateTime::currentDateTime();

    int32_t tag = getPLCTag(tagName, elementSize);


    if(tag > 0)
    {
        /* get the data */
        int rc = plc_tag_read(tag, _getPLCStatusTimer->interval());
        if(rc != PLCTAG_STATUS_OK)
        {
            qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
            m_PLCTags.remove(tagName);
            plc_tag_destroy(tag);
            return false;
        }
        //qDebug() << QString::number(tag);
        tagValue.connectionFaulted = static_cast<bool>(plc_tag_get_bit(tag, 0));
        tagValue.detectionPosition = plc_tag_get_int32(tag, 20);

        qDebug() << "tagValue.detectionPosition";
        qDebug() << QString::number(tagValue.detectionPosition);

        return true;
    }

    return false;
}

bool PLCTag::readPLCTag(QString tagName, uint64_t &tagValue)
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
            m_PLCTags.remove(tagName);
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
            qDebug() << "PLCTag::writePLCTag() - plc_tag_write  - FAIL";
            return false;
        }
        //plc_tag_destroy(tag);
        qDebug() << "PLCTag::writePLCTag() SUCCESS";

        return true;
    }

    qDebug() << "PLCTag::writePLCTag() FAIL";
    return false;
}

// bool PLCTag::updateHMIFromPLCTag(QString tagName)
// {
//     uint32_t tagId =_PLCTags[tagName];
//     int status = 0;

//     switch (tagName)
//     {
//         case "PLC_Heart_Beat":
//             uint64_t value = plc_tag_get_uint64(tagId, 0);
//             status = plc_tag_status(tagId);

//             if(status == PLCTAG_STATUS_OK)
//                 setPLCIsConnected(true);
//             else
//             {
//                 qDebug() << "Read error:" << plc_tag_decode_error(status);
//                 setPLCIsConnected(falsee);
//             }
//             break;
//         case "PLC_Heart_Beat1":
//             uint64_t value = plc_tag_get_uint64(tagId, 0);
//             status = plc_tag_status(tagId);

//             if(status == PLCTAG_STATUS_OK)
//                 setPLCIsConnected(true);
//             else
//             {
//                 qDebug() << "Read error:" << plc_tag_decode_error(status);
//                 setPLCIsConnected(falsee);
//             }
//             break;
//         default:
//             break;
//     }





//     if (tagName == "PLC_Heart_Beat")
//     {
//         uint64_t value = plc_tag_get_uint64(tagId, 0);
//         status = plc_tag_status(tagId);

//         if(status == PLCTAG_STATUS_OK)
//             setPLCIsConnected(true);
//         else
//         {
//             qDebug() << "Read error:" << plc_tag_decode_error(status);
//             setPLCIsConnected(falsee);
//         }
//     }
//     else if (tagName == "PLC_Heart_Beat1")
//     {
//         // Code block 2: executed if condition1 was false and condition2 is true
//     }
//     else if (tagName == "PLC_Heart_Beat2")
//     {
//         // Code block 3: executed if condition1 and condition2 were false and condition3 is true
//     }
//     else if (tagName == "PLC_Heart_Beat3")
//     {
//         // Code block 3: executed if condition1 and condition2 were false and condition3 is true
//     }
// }

void PLCTag::getPLCStatus()
{
    //qDebug() << "PLCTag::getPLCStatus()" << QDateTime::currentDateTime();
    //here we will get all the PLC tags, in threads (concurrently)
    //Program:SafetyProgram.PHY_ESTOP_ACTIVATED
    _getPLCStatusTimer->stop();

    uint64_t uint64TagValue = 0;

    if(readPLCTag(m_plcMainProgramName + "PLC_Heart_Beat", uint64TagValue))
    {
        setPLCIsConnected(true);
        qDebug() << "PLC_Heart_Beat" << uint64TagValue;

        QFutureSynchronizer<void> synchronizer;
        QFuture<void> future;

        future = QtConcurrent::run([this]() {
            bool tagValue = getAllSafetyInputsOK();
            readPLCTag(QString("") + "ALL_Safety_Inputs_OK", tagValue) ? setAllSafetyInputsOK(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getEStop1Activated();
            readPLCTag(m_plcSafetyProgramName + "PHY_ESTOP_1_ACTIVATED", tagValue) ? setEStop1Activated(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getEStop1Faulted();
            readPLCTag(m_plcSafetyProgramName + "PHY_ESTOP_1_FAULTED", tagValue) ? setEStop1Faulted(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getLightCurtain1Activated();
            readPLCTag(m_plcSafetyProgramName + "PHY_LIGHTCURTAIN_1_ACTIVATED", tagValue) ? setLightCurtain1Activated(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getLightCurtain1Faulted();
            readPLCTag(m_plcSafetyProgramName + "PHY_LIGHTCURTAIN_1_FAULTED", tagValue) ? setLightCurtain1Faulted(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getAreaScanner1Activated();
            readPLCTag(m_plcSafetyProgramName + "PHY_AREASCANNER_1_ACTIVATED", tagValue) ? setAreaScanner1Activated(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getAreaScanner1Faulted();
            readPLCTag(m_plcSafetyProgramName + "PHY_AREASCANNER_1_FAULTED", tagValue) ? setAreaScanner1Faulted(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getPowerState();
            readPLCTag(m_plcMainProgramName + "System_Powered_ON", tagValue) ? setPowerState(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue =getRunStateSCAN();
            readPLCTag(m_plcMainProgramName + "System_Run_SCAN", tagValue) ? setRunStateSCAN(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue =getRedPilotLight();
            readPLCTag(m_plcMainProgramName + "Red_Pilot_Light", tagValue) ? setRedPilotLight(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getAmberPilotLight();
            readPLCTag(m_plcMainProgramName + "Amber_Pilot_Light", tagValue) ? setAmberPilotLight(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getGreenPilotLight();
            readPLCTag(m_plcMainProgramName + "Green_Pilot_Light", tagValue) ? setGreenPilotLight(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getBluePilotLight();
            readPLCTag(m_plcMainProgramName + "Blue_Pilot_Light", tagValue) ? setBluePilotLight(tagValue) :  (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getWhitePilotLight();
            readPLCTag(m_plcMainProgramName + "White_Pilot_Light", tagValue) ? setWhitePilotLight(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue = getEndLimitSwitch();
            readPLCTag(m_plcMainProgramName + "PHY_End_LIMIT_SWITCH", tagValue) ? setEndLimitSwitch(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            bool tagValue =getHomeLimitSwitch();
            readPLCTag(m_plcMainProgramName + "PHY_Home_LIMIT_SWITCH", tagValue) ? setHomeLimitSwitch(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

        future = QtConcurrent::run([this]() {
            StepperMotor_AZD_AEP_t tagValue;// = getWhitePilotLight();
            readPLCTag(QString("") + "Stepper_MOT:I", 60, tagValue) ? setStepperMotor_AZD_AEP_Input(tagValue) : (void)0; // do nothiing if false
        });
        synchronizer.addFuture(future);

    }
    else
    {
        setPLCIsConnected(false);
    }

    _getPLCStatusTimer->start();

}


bool PLCTag::getPLCIsConnected() const
{
    return m_plcIsConnected;
}

void PLCTag::setPLCIsConnected(bool newValue)
{
    if (m_plcIsConnected == newValue)
        return;

    m_plcIsConnected = newValue;
    emit plcIsConnectedChanged(m_plcIsConnected); // Emit signal to trigger QML updates
}

bool PLCTag::getAllSafetyInputsOK() const
{
    return m_allSafetyInputsOK;
}

void PLCTag::setAllSafetyInputsOK(bool newValue)
{
    if (m_allSafetyInputsOK == newValue)
        return;

    m_allSafetyInputsOK = newValue;
    emit allSafetyInputsOKChanged(m_allSafetyInputsOK); // Emit signal to trigger QML updates
}

bool PLCTag::getEStop1Faulted() const
{
    return m_eStop1Faulted;
}

void PLCTag::setEStop1Faulted(bool newValue)
{
    if (m_eStop1Faulted == newValue)
        return;

    m_eStop1Faulted = newValue;
    emit eStop1FaultedChanged(m_eStop1Faulted); // Emit signal to trigger QML updates
}

bool PLCTag::getEStop1Activated() const
{
    return m_eStop1Activated;
}

void PLCTag::setEStop1Activated(bool newValue)
{
    if (m_eStop1Activated == newValue)
        return;

    m_eStop1Activated = newValue;
    emit eStop1ActivatedChanged(m_eStop1Activated); // Emit signal to trigger QML updates
}


bool PLCTag::getLightCurtain1Faulted() const
{
    return m_lightCurtain1Faulted;
}

void PLCTag::setLightCurtain1Faulted(bool newValue)
{
    if (m_lightCurtain1Faulted == newValue)
        return;

    m_lightCurtain1Faulted = newValue;
    emit lightCurtain1FaultedChanged(m_lightCurtain1Faulted); // Emit signal to trigger QML updates
}

bool PLCTag::getLightCurtain1Activated() const
{
    return m_lightCurtain1Activated;
}

void PLCTag::setLightCurtain1Activated(bool newValue)
{
    if (m_lightCurtain1Activated == newValue)
        return;

    m_lightCurtain1Activated = newValue;
    emit lightCurtain1ActivatedChanged(m_lightCurtain1Activated); // Emit signal to trigger QML updates
}


bool PLCTag::getAreaScanner1Faulted() const
{
    return m_areaScanner1Faulted;
}

void PLCTag::setAreaScanner1Faulted(bool newValue)
{
    if (m_areaScanner1Faulted == newValue)
        return;

    m_areaScanner1Faulted = newValue;
    emit areaScanner1FaultedChanged(m_areaScanner1Faulted); // Emit signal to trigger QML updates
}

bool PLCTag::getAreaScanner1Activated() const
{
    return m_areaScanner1Activated;
}

void PLCTag::setAreaScanner1Activated(bool newValue)
{
    if (m_areaScanner1Activated == newValue)
        return;

    m_areaScanner1Activated = newValue;
    emit areaScanner1ActivatedChanged(m_areaScanner1Activated); // Emit signal to trigger QML updates
}

bool PLCTag::getPowerState() const
{
    return m_powerState;
}

void PLCTag::setPowerState(bool newValue)
{
    if (m_powerState == newValue)
        return;

    m_powerState = newValue;
    emit powerStateChanged(m_powerState); // Emit signal to trigger QML updates
}

bool PLCTag::getRunStateSCAN() const
{
    return m_runStateSCAN;
}

void PLCTag::setRunStateSCAN(bool newValue)
{
    if (m_runStateSCAN == newValue)
        return;

    m_runStateSCAN = newValue;
    emit runStateSCANChanged(m_runStateSCAN); // Emit signal to trigger QML updates
}

void PLCTag::powerOnButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::powerOnButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_PowerON_PB", pressed);
}

void PLCTag::powerOffButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::powerOffButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_PowerOFF_PB", pressed);
}

void PLCTag::resetButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::resetButtonPressedChanged()";

    writePLCTag(m_plcSafetyProgramName + "HMI_Reset_PB", pressed);
}

void PLCTag::moveToHomeButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveToHomeButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_MoveToHome_PB", pressed);
}

void PLCTag::moveToEndButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveToEndButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_MoveToEnd_PB", pressed);
}

void PLCTag::startDataCaptureButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::startDataCaptureButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_StartDataCapture_PB", pressed);
}

void PLCTag::moveBackButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveBackButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_MoveBack_PB", pressed);
}

void PLCTag::moveForwardButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::moveForwardButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_MoveForward_PB", pressed);
}

void PLCTag::stopDataCaptureButtonPressedChanged(bool pressed)
{
    qDebug() << "PLCTag::stopDataCaptureButtonPressedChanged()";

    writePLCTag(m_plcMainProgramName + "HMI_StopDataCapture_PB", pressed);
}



bool PLCTag::getRedPilotLight() const
{
    return m_redPilotLight;
}

void PLCTag::setRedPilotLight(bool newValue)
{
    if (m_redPilotLight == newValue)
        return;

    m_redPilotLight = m_plcIsConnected && newValue;;
    emit redPilotLightChanged(m_redPilotLight);
}

bool PLCTag::getAmberPilotLight() const
{
    return m_amberPilotLight;
}

void PLCTag::setAmberPilotLight(bool newValue)
{
    if (m_amberPilotLight == newValue)
        return;

    m_amberPilotLight = m_plcIsConnected && newValue;;
    emit amberPilotLightChanged(m_amberPilotLight);
}

bool PLCTag::getGreenPilotLight() const
{
    return m_greenPilotLight;
}

void PLCTag::setGreenPilotLight(bool newValue)
{
    if (m_greenPilotLight == newValue)
        return;

    m_greenPilotLight = m_plcIsConnected && newValue;
    emit greenPilotLightChanged(m_greenPilotLight);
}

bool PLCTag::getBluePilotLight() const
{
    return m_bluePilotLight;
}

void PLCTag::setBluePilotLight(bool newValue)
{
    if (m_bluePilotLight == newValue)
        return;

    m_bluePilotLight = m_plcIsConnected && newValue;;
    emit bluePilotLightChanged(m_bluePilotLight);
}

bool PLCTag::getWhitePilotLight() const
{
    return m_whitePilotLight;
}

void PLCTag::setWhitePilotLight(bool newValue)
{
    if (m_whitePilotLight == newValue)
        return;

    m_whitePilotLight = m_plcIsConnected && newValue;;
    emit whitePilotLightChanged(m_whitePilotLight);
}

bool PLCTag::getEndLimitSwitch() const
{
    return m_EndLimitSwitch;
}

void PLCTag::setEndLimitSwitch(bool newValue)
{
    if (m_EndLimitSwitch == newValue)
        return;

    m_EndLimitSwitch = newValue;;
    emit endLimitSwitchChanged(m_EndLimitSwitch);
}

bool PLCTag::getHomeLimitSwitch() const
{
    return m_HomeLimitSwitch;
}

void PLCTag::setHomeLimitSwitch(bool newValue)
{
    if (m_HomeLimitSwitch == newValue)
        return;

    m_HomeLimitSwitch = newValue;;
    emit homeLimitSwitchChanged(m_HomeLimitSwitch);
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









StepperMotor_AZD_AEP_t PLCTag::getStepperMotor_AZD_AEP_Input() const
{
    return m_stepperMotor_AZD_AEP_Input;
}

void PLCTag::setStepperMotor_AZD_AEP_Input(const StepperMotor_AZD_AEP_t &newValue)
{
    setStepperMotorDetectionPosition(newValue.detectionPosition);
}

int PLCTag::getStepperMotorDetectionPosition() const
{
    return m_stepperMotor_AZD_AEP_Input.detectionPosition;
}

void PLCTag::setStepperMotorDetectionPosition(int newValue)
{
    if (m_stepperMotor_AZD_AEP_Input.detectionPosition == newValue)
        return;

    m_stepperMotor_AZD_AEP_Input.detectionPosition = newValue;
    emit stepperMotorDetectionPositionChanged(m_stepperMotor_AZD_AEP_Input.detectionPosition);
}
