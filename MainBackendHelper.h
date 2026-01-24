#ifndef MAINBACKENDHELPER_H
#define MAINBACKENDHELPER_H

#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QQmlEngine>


class MainBackendHelper : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON
public:
    explicit MainBackendHelper(QObject *parent = nullptr);
    ~MainBackendHelper();

signals:
    void requestPLCStatus();

public slots:
    void onConnectToPLC();
    void onStartScan();
    void onResetSystem();
    void onStopScan();
    void onSafetyStop();
    void onMoveToHome();
    void onMoveLeft();
    void onMoveRight();
    void onMoveForward();
    void onMoveBack();
    void onRequestPLCStatus();

private:
    std::unique_ptr<QTimer> _getPLCStatusTimer; // Default initialized to nullptr
    QDateTime _dateTimeOnApplicationStart = QDateTime::currentDateTime();    
};

#endif // MAINBACKENDHELPER_H
