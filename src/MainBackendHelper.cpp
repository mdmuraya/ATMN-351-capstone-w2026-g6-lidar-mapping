#include <QCoreApplication>
#include <QDebug>
#include <QVariantList>

#include "include/MainBackendHelper.h"


MainBackendHelper::MainBackendHelper(QObject *parent) :
    QObject (parent),
    _getPLCStatusTimer (std::make_unique<QTimer>())
{
    qDebug() << "MainBackendHelper::MainBackendHelper()";

    // Get all arguments as a QStringList
    const QStringList commandLineArguments = QCoreApplication::arguments();

    qDebug() << "Total arguments:" << commandLineArguments.count();

    // Iterate over the arguments
    for (int i = 0; i < commandLineArguments.count(); ++i) {
        qDebug() << "Argument" << i << ":" << commandLineArguments.at(i);
    }

    // Access specific arguments (e.g., the second argument if it exists)
    if (commandLineArguments.count() > 1) {
        qDebug() << "Second argument:" << commandLineArguments.at(1);
    }

    connect(_getPLCStatusTimer.get(), &QTimer::timeout, [this](){
        emit requestPLCStatus();
    });

    connect(this, &MainBackendHelper::requestPLCStatus, this, &MainBackendHelper::onRequestPLCStatus);

    _getPLCStatusTimer->start(1000); //1000 miliseconds

}


MainBackendHelper::~MainBackendHelper()
{
    qDebug() << "MainBackendHelper::~MainBackendHelper()";
}

void MainBackendHelper::onConnectToPLC()
{
    qDebug() << "MainBackendHelper::onConnectToPLC()";
}

void MainBackendHelper::onStartScan()
{
    qDebug() << "MainBackendHelper::onStartScan()";
}

void MainBackendHelper::onResetSystem()
{
    qDebug() << "MainBackendHelper::onResetSystem()";
}

void MainBackendHelper::onStopScan()
{
    qDebug() << "MainBackendHelper::onStopScan()";
}

void MainBackendHelper::onSafetyStop()
{
    qDebug() << "MainBackendHelper::onSafetyStop()";
}

void MainBackendHelper::onMoveToHome()
{
    qDebug() << "MainBackendHelper::onMoveToHome()";
}

void MainBackendHelper::onMoveLeft()
{
    qDebug() << "MainBackendHelper::onMoveLeft()";
}

void MainBackendHelper::onMoveRight()
{
    qDebug() << "MainBackendHelper::onMoveRight()";
}

void MainBackendHelper::onMoveForward()
{
    qDebug() << "MainBackendHelper::onMoveForward()";
}

void MainBackendHelper::onMoveBack()
{
    qDebug() << "MainBackendHelper::onMoveBack()";
}

void MainBackendHelper::onRequestPLCStatus()
{
    qDebug() << "MainBackendHelper::onRequestPLCStatus()" << QDateTime::currentDateTime();
}




