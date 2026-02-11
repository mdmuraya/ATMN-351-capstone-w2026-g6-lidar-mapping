#include <QDebug>
#include <QTime>

#include "include/PLCTag.hpp"
#include "include/libplctag.h"



PLCTag::PLCTag(QObject *parent, QString plcAddress, QString plcType) :
    QObject (parent),
    _plcAddress(plcAddress),
    _plcType(plcType)
{
    qDebug() << QString("PLCTag::PLCTag(") + plcAddress + QString(", ") + plcType + QString(")");
}


PLCTag::~PLCTag()
{
    qDebug() << "PLCTag::~PLCTag()";

    for (auto [tagName, tag] : _PLCTags.asKeyValueRange()) { // asKeyValueRange() available from Qt 6.3
        qDebug() << "DESTRYOING Tag: Name:" << tagName << "Value:" << tag;
        plc_tag_destroy(tag);
    }
    _PLCTags.clear();
}

int32_t PLCTag::getPLCTag(QString tagName)
{
    qDebug() << "PLCTag::getPLCTag()" << QDateTime::currentDateTime();

    if (_PLCTags.contains(tagName)) {
        qDebug() << "Key" << tagName << " found.";
        return _PLCTags.value(tagName);
    }

    qDebug() << "Key " << tagName << " NOT FOUND. Creating...";

    QString plcTagPath = QString("protocol=ab-eip&gateway=") + _plcAddress + QString("&plc=") + _plcType + QString("&elem_size=1&elem_count=1&name=") + tagName;
    //QString plcTagPath = _plcTagPathBase + tagName;
    int32_t tag = plc_tag_create(plcTagPath.toUtf8().constData(), _dataTimeout);

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
    qDebug() << "PLCTag::readPLCTag()" << QDateTime::currentDateTime();

    int32_t tag = getPLCTag(tagName);

    if(tag > 0)
    {
        /* get the data */
        int rc = plc_tag_read(tag, _dataTimeout);
        if(rc != PLCTAG_STATUS_OK)
        {
            qDebug() << "ERROR: Unable to read the data! Got error code" << rc << ":" << QString::fromUtf8(plc_tag_decode_error(rc));
            _PLCTags.remove(tagName);
            plc_tag_destroy(tag);
            return false;
        }
        qDebug() << QString::number(tag);
        tagValue = static_cast<bool>(plc_tag_get_bit(tag, 0));

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

        if(plc_tag_write(tag, _dataTimeout) != PLCTAG_STATUS_OK) {
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








