#include <QDebug>
#include <QVariantList>

#include "MainBackendHelper.h"


MainBackendHelper::MainBackendHelper(QObject *parent) :
    QObject (parent),
    _getPLCStatusTimer (std::make_unique<QTimer>())
{
    qDebug() << "MainBackendHelper::MainBackendHelper()";

    connect(_getPLCStatusTimer.get(), &QTimer::timeout, [this](){
        //emit requestPLCStatus();
    });

    connect(this, &MainBackendHelper::requestPLCStatus, this, &MainBackendHelper::onRequestPLCStatus);

    _getPLCStatusTimer->start(100); //100 miliseconds

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
    qDebug() << "MainBackendHelper::onRequestPLCStatus()";
}




