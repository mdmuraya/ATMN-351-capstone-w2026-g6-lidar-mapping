#ifndef PLCTAG_HPP
#define PLCTAG_HPP

#include <QObject>
#include <QHash>


class PLCTag : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString plcAddress READ getPLCAddress)
    Q_PROPERTY(bool plcIsConnected READ getPLCIsConnected WRITE setPLCIsConnected NOTIFY plcIsConnectedChanged)
    Q_PROPERTY(bool runState READ getRunState WRITE setRunState NOTIFY runStateChanged)
    Q_PROPERTY(bool runStateAUTO READ getRunStateAUTO WRITE setRunStateAUTO NOTIFY runStateAUTOChanged)
    Q_PROPERTY(bool redPilotLight READ getRedPilotLight WRITE setRedPilotLight NOTIFY redPilotLightChanged)
    Q_PROPERTY(bool amberPilotLight READ getAmberPilotLight WRITE setAmberPilotLight NOTIFY amberPilotLightChanged)
    Q_PROPERTY(bool greenPilotLight READ getGreenPilotLight WRITE setGreenPilotLight NOTIFY greenPilotLightChanged)
    Q_PROPERTY(bool bluePilotLight READ getBluePilotLight WRITE setBluePilotLight NOTIFY bluePilotLightChanged)
    Q_PROPERTY(bool whitePilotLight READ getWhitePilotLight WRITE setWhitePilotLight NOTIFY whitePilotLightChanged)

public:
    explicit PLCTag(QObject *parent = nullptr, QString plcAddress = "", QString plcType = "", QString plcProgramName = "");
    ~PLCTag();

    QString getPLCAddress() const;

    bool getPLCIsConnected() const;
    void setPLCIsConnected(bool newValue);

    bool getRunState() const;
    void setRunState(bool newValue);

    bool getRunStateAUTO() const;
    void setRunStateAUTO(bool newValue);

    bool getRedPilotLight() const;
    void setRedPilotLight(bool newValue);

    bool getAmberPilotLight() const;
    void setAmberPilotLight(bool newAmberPilotLight);

    bool getGreenPilotLight() const;
    void setGreenPilotLight(bool newGreenPilotLight);

    bool getBluePilotLight() const;
    void setBluePilotLight(bool newBluePilotLight);

    bool getWhitePilotLight() const;
    void setWhitePilotLight(bool newWhitePilotLight);


signals:
    void plcIsConnectedChanged(bool newValue);
    void runStateChanged(bool newValue);
    void runStateAUTOChanged(bool newValue);
    void redPilotLightChanged(bool newValue);
    void amberPilotLightChanged(bool newValue);
    void greenPilotLightChanged(bool newValue);
    void bluePilotLightChanged(bool newValue);
    void whitePilotLightChanged(bool newValue);

public slots:
    void onConnectToPLC();
    void startButtonPressedChanged(bool pressed);
    void stopButtonPressedChanged(bool pressed);
    void resetButtonPressedChanged(bool pressed);
    void moveToHomeButtonPressedChanged(bool pressed);
    void moveLeftButtonPressedChanged(bool pressed);
    void moveBackButtonPressedChanged(bool pressed);
    void moveForwardButtonPressedChanged(bool pressed);
    void moveRightButtonPressedChanged(bool pressed);

private:
    std::unique_ptr<QTimer> _getPLCStatusTimer = nullptr;
    QString _plcAddress = "";
    QString _plcType = "";
    QString _plcProgramName = "";
    int32_t _dataTimeout = 3000;
    QHash<QString, int32_t> _PLCTags;
    bool _plcIsConnected = false;
    bool _runState = false;
    bool _runStateAUTO = false;
    bool _redPilotLight = false;
    bool _amberPilotLight = false;
    bool _greenPilotLight = false;
    bool _bluePilotLight = false;
    bool _whitePilotLight = false;

    void getPLCStatus();
    int32_t getPLCTag(QString tagName);
    bool readPLCTag(QString tagName, bool &tagValue);
    uint64_t readPLCTag(QString tagName, uint64_t &tagValue);
    bool writePLCTag(QString tagName, bool tagValue);

};

#endif // PLCTAG_HPP
