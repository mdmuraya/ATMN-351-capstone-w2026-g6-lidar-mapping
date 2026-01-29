#include <QDebug>

#include "include/libplctag.h"
#include "include/PLCTag.hpp"

PLCTag::PLCTag(QObject *parent, int plc_prot, std::string ip_address, bool debug) :
    QObject (parent)
{
    qDebug() << "PLCTag::PLCTag()";

}


PLCTag::~PLCTag()
{
    qDebug() << "PLCTag::~PLCTag()";
}
