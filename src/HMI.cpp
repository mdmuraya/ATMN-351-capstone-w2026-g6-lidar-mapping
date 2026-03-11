#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "include/HMIBackendHelper.hpp"
#include "include/LIDARScanPointCloud2Geometry.hpp"
#include "include/DEMSurface.hpp"

int main(int argc, char *argv[])
{
    qInfo() << "***************************************";
    qInfo() << "*** LIDAR Mapping HMI starting up... ***";
    qInfo() << "***************************************";

    QGuiApplication qGuiApplication(argc, argv);

    qmlRegisterType<DEMSurface>("LIDARScanPointCloud2", 1, 0, "DEMSurface");

    HMIBackendHelper hmiBackendHelper;

    if( !hmiBackendHelper.initialize(&qGuiApplication))
    {
        qCritical() << "*****************************************************************************";
        qCritical() << "*** Could not instantiate HMIBackendHelper. LIDAR Mapping HMI aborting... ***";
        qCritical() << "*****************************************************************************";

        return -1;
    }

    auto returnValue = qGuiApplication.exec();

    qInfo() << "*******************************************************************************";
    qInfo() << "*** LIDAR Mapping HMI terminating with retun value:" << returnValue << "... ***";
    qInfo() << "*******************************************************************************";

    return returnValue;
}
