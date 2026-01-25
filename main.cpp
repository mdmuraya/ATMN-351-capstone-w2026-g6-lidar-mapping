#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "MainBackendHelper.h"

int main(int argc, char *argv[])
{
    qInfo() << "***************************************";
    qInfo() << "*** LIDARMapping App starting up... ***";
    qInfo() << "***************************************";

    QGuiApplication qGuiApplication(argc, argv);

    //instantiate the MainBackendHelper, and if that suceeds, load the UI


    auto mainBackendHelper(std::make_unique<MainBackendHelper>());// new MainBackendHelper());

    if(mainBackendHelper == nullptr)
    {
        qCritical() << "*****************************************************************************";
        qCritical() << "*** Could not instantiate MainBackendHelper. LIDARMapping App aborting... ***";
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
    engine.loadFromModule("LIDARMapping", "Main");

    auto returnValue = qGuiApplication.exec();

    qInfo() << "*******************************************************************************";
    qInfo() << "*** LIDARMapping App terminating with retun value:" << returnValue << "... ***";
    qInfo() << "*******************************************************************************";

    return returnValue;
}
