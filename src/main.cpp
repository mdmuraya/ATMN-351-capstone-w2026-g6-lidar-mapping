#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "include/MainBackendHelper.hpp"

int main(int argc, char *argv[])
{
    qInfo() << "***************************************";
    qInfo() << "*** LIDAR Mapping HMI App starting up... ***";
    qInfo() << "***************************************";

    QGuiApplication qGuiApplication(argc, argv);

    MainBackendHelper mainBackendHelper;

    if( !mainBackendHelper.initialize(&qGuiApplication))
    {
        qCritical() << "*****************************************************************************";
        qCritical() << "*** Could not instantiate MainBackendHelper. LIDAR Mapping HMI App aborting... ***";
        qCritical() << "*****************************************************************************";

        return -1;
    }

    auto returnValue = qGuiApplication.exec();

    qInfo() << "*******************************************************************************";
    qInfo() << "*** LIDAR Mapping HMI App terminating with retun value:" << returnValue << "... ***";
    qInfo() << "*******************************************************************************";

    return returnValue;

    /*

    //instantiate the MainBackendHelper, and if that suceeds, load the UI


    auto mainBackendHelper(std::make_unique<MainBackendHelper>());// new MainBackendHelper());

    if(mainBackendHelper == nullptr)
    {
        qCritical() << "*****************************************************************************";
        qCritical() << "*** Could not instantiate MainBackendHelper. LIDAR Mapping HMI App aborting... ***";
        qCritical() << "*****************************************************************************";

        return -1;
    }


    QQmlApplicationEngine engine;
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &qGuiApplication,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    engine.rootContext()->setContextProperty("MainBackendHelper", mainBackendHelper.get());
    engine.loadFromModule("LIDAR_Mapping_HMI", "Main");


    auto returnValue = qGuiApplication.exec();

    qInfo() << "*******************************************************************************";
    qInfo() << "*** LIDAR Mapping HMI App terminating with retun value:" << returnValue << "... ***";
    qInfo() << "*******************************************************************************";

    return returnValue;

*/
}
