#include <QGuiApplication>
#include <QQmlApplicationEngine>

#include "MainBackendHelper.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    //instantiate the MainBackendHelper, and if that suceeds, load the UI

    auto mainBackendHelper(std::make_unique<MainBackendHelper>());// new MainBackendHelper());

    if(mainBackendHelper == nullptr)
        return -1;


    QQmlApplicationEngine engine;
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("LIDARMapping", "Main");

    return app.exec();
}
