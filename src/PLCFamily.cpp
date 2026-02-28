#include "include/PLCFamily.hpp"


PLCFamily::PLCFamily(QObject *parent, QString plcFamilyDescription, QString plcFamilyId):
    QObject (parent),
    _plcFamilyDescription(plcFamilyDescription),
    _plcFamilyId(plcFamilyId)
{
    //TODO
}

PLCFamily::~PLCFamily()
{
    //TODO
}

QString PLCFamily::getPLCFamilyDescription() const
{
    return _plcFamilyDescription;
}

QString PLCFamily::getPLCProgramName() const
{
    if (_plcFamilyId.toLower() == QString("micro800").toLower())
        return "";

    if (_plcFamilyId.toLower() == QString("controllogix").toLower())
        return "Program:MainProgram.";

    return "";

}

QString PLCFamily::getPLCFamilyId() const
{
    return _plcFamilyId;
}

