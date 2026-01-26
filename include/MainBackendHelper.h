#ifndef MAINBACKENDHELPER_H
#define MAINBACKENDHELPER_H

#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QQmlEngine>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class MainBackendHelper : public QObject
{
    Q_OBJECT
    //QML_ELEMENT
    //QML_SINGLETON
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
        QDateTime _dateTimeOnApplicationStart = QDateTime::currentDateTime();
        std::unique_ptr<QTimer> _getPLCStatusTimer = nullptr;
        std::unique_ptr<QTimer> _publishTimer = nullptr;
        std::shared_ptr<rclcpp::Node> _rosNode = nullptr;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher = nullptr;
};

#endif // MAINBACKENDHELPER_H
