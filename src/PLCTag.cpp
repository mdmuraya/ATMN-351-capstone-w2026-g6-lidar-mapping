#include "include/libplctag.h"
#include "include/PLCTag.hpp"

PLCTag::PLCTag(QObject *parent) :
    QObject (parent)
{
    qDebug() << "PLCTag::PLCTag()";

}


PLCTag::~PLCTag()
{
    qDebug() << "PLCTag::~PLCTag()";
}
