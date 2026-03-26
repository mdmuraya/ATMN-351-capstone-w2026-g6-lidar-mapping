#ifndef PLCTAG_HPP
#define PLCTAG_HPP

#include <QObject>
#include <QHash>

struct StepperMotor_AZD_AEP_t
{
    bool connectionFaulted = false;
    int  detectionPosition;

    auto operator<=>(const StepperMotor_AZD_AEP_t&) const = default;

} ;

class PLCTag : public QObject
{
    Q_OBJECT
    //Q_PROPERTY(QList<PLCFamily> listOfPLCFamily READ getListOfPLCFamily)


    //Q_PROPERTY(QString plcAddress READ getPLCAddress NOTIFY plcAddressChanged)
    Q_PROPERTY(bool plcIsConnected READ getPLCIsConnected WRITE setPLCIsConnected NOTIFY plcIsConnectedChanged)
    Q_PROPERTY(bool allSafetyInputsOK READ getAllSafetyInputsOK WRITE setAllSafetyInputsOK NOTIFY allSafetyInputsOKChanged)
    Q_PROPERTY(bool eStop1Activated READ getEStop1Activated WRITE setEStop1Activated NOTIFY eStop1ActivatedChanged)
    Q_PROPERTY(bool eStop1Faulted READ getEStop1Faulted WRITE setEStop1Faulted NOTIFY eStop1FaultedChanged)
    Q_PROPERTY(bool lightCurtain1Activated READ getLightCurtain1Activated WRITE setLightCurtain1Activated NOTIFY lightCurtain1ActivatedChanged)
    Q_PROPERTY(bool lightCurtain1Faulted READ getLightCurtain1Faulted WRITE setLightCurtain1Faulted NOTIFY lightCurtain1FaultedChanged)
    Q_PROPERTY(bool areaScanner1Activated READ getAreaScanner1Activated WRITE setAreaScanner1Activated NOTIFY areaScanner1ActivatedChanged)
    Q_PROPERTY(bool areaScanner1Faulted READ getAreaScanner1Faulted WRITE setAreaScanner1Faulted NOTIFY areaScanner1FaultedChanged)
    Q_PROPERTY(bool runState READ getRunState WRITE setRunState NOTIFY runStateChanged)
    Q_PROPERTY(bool runStateSCAN READ getRunStateSCAN WRITE setRunStateSCAN NOTIFY runStateSCANChanged)
    Q_PROPERTY(bool redPilotLight READ getRedPilotLight WRITE setRedPilotLight NOTIFY redPilotLightChanged)
    Q_PROPERTY(bool amberPilotLight READ getAmberPilotLight WRITE setAmberPilotLight NOTIFY amberPilotLightChanged)
    Q_PROPERTY(bool greenPilotLight READ getGreenPilotLight WRITE setGreenPilotLight NOTIFY greenPilotLightChanged)
    Q_PROPERTY(bool bluePilotLight READ getBluePilotLight WRITE setBluePilotLight NOTIFY bluePilotLightChanged)
    Q_PROPERTY(bool whitePilotLight READ getWhitePilotLight WRITE setWhitePilotLight NOTIFY whitePilotLightChanged)
    Q_PROPERTY(int stepperMotorDetectionPosition READ getStepperMotorDetectionPosition WRITE setStepperMotorDetectionPosition NOTIFY stepperMotorDetectionPositionChanged)


    public:
        explicit PLCTag(QObject *paren);
        ~PLCTag();

        void connectToPLC(QString plcAddress, QString plcFamilyId, QString plcMainProgramName, QString plcSafetyProgramName);
        void disconnectFromPLC();

        bool getPLCIsConnected() const;
        void setPLCIsConnected(bool newValue);

        bool getAllSafetyInputsOK() const;
        void setAllSafetyInputsOK(bool newValue);

        bool getEStop1Activated() const;
        void setEStop1Activated(bool newValue);

        bool getEStop1Faulted() const;
        void setEStop1Faulted(bool newValue);

        bool getLightCurtain1Activated() const;
        void setLightCurtain1Activated(bool newValue);

        bool getLightCurtain1Faulted() const;
        void setLightCurtain1Faulted(bool newValue);

        bool getAreaScanner1Activated() const;
        void setAreaScanner1Activated(bool newValue);

        bool getAreaScanner1Faulted() const;
        void setAreaScanner1Faulted(bool newValue);

        bool getRunState() const;
        void setRunState(bool newValue);

        bool getRunStateSCAN() const;
        void setRunStateSCAN(bool newValue);

        bool getRedPilotLight() const;
        void setRedPilotLight(bool newValue);

        bool getAmberPilotLight() const;
        void setAmberPilotLight(bool newValue);

        bool getGreenPilotLight() const;
        void setGreenPilotLight(bool newValue);

        bool getBluePilotLight() const;
        void setBluePilotLight(bool newValue);

        bool getWhitePilotLight() const;
        void setWhitePilotLight(bool newValue);

        StepperMotor_AZD_AEP_t getStepperMotor_AZD_AEP_Input() const;
        void setStepperMotor_AZD_AEP_Input(const StepperMotor_AZD_AEP_t &newValue);

        int getStepperMotorDetectionPosition() const;
        void setStepperMotorDetectionPosition(int newValue);

    signals:
        //void plcAddressChanged(bool newValue);
        void plcIsConnectedChanged(bool newValue);
        void allSafetyInputsOKChanged(bool newValue);
        void eStop1ActivatedChanged(bool newValue);
        void eStop1FaultedChanged(bool newValue);//
        void lightCurtain1ActivatedChanged(bool newValue);
        void lightCurtain1FaultedChanged(bool newValue);
        void areaScanner1ActivatedChanged(bool newValue);
        void areaScanner1FaultedChanged(bool newValue);
        void runStateChanged(bool newValue);
        void runStateSCANChanged(bool newValue);
        void redPilotLightChanged(bool newValue);
        void amberPilotLightChanged(bool newValue);
        void greenPilotLightChanged(bool newValue);
        void bluePilotLightChanged(bool newValue);
        void whitePilotLightChanged(bool newValue);
        void stepperMotor_AZD_AEP_InputChanged(StepperMotor_AZD_AEP_t newValue);
        void stepperMotorDetectionPositionChanged(int newValue);

    public slots:
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

        QString m_plcAddress = "";
        QString _plcFamilyId = "";
        QString _plcMainProgramName = "";
        QString _plcSafetyProgramName = "";
        QHash<QString, int32_t> _PLCTags;
        bool _plcIsConnected = false;
        bool _runState = false;
        bool _allSafetyInputsOK = false;
        bool _eStop1Activated = false;
        bool _eStop1Faulted = false;
        bool _lightCurtain1Activated = false;
        bool _lightCurtain1Faulted = false;
        bool _areaScanner1Activated = false;
        bool _areaScanner1Faulted = false;
        bool _runStateSCAN = false;
        bool _redPilotLight = false;
        bool _amberPilotLight = false;
        bool _greenPilotLight = false;
        bool _bluePilotLight = false;
        bool _whitePilotLight = false;
        StepperMotor_AZD_AEP_t m_stepperMotor_AZD_AEP_Input;

        void getPLCStatus();
        int32_t getPLCTag(QString tagName, uint32_t elementSize = 1);
        // static void eventCallback(int32_t tagId, int eventId, int status, void *userdata);
        bool readPLCTag(QString tagName, bool &tagValue);
        bool readPLCTag(QString tagName, uint64_t &tagValue);
        bool readPLCTag(QString tagName, uint32_t elementSize, StepperMotor_AZD_AEP_t &tagValue);
        bool writePLCTag(QString tagName, bool tagValue);
        // bool updateHMIFromPLCTag(QString tagName);
};

#endif // PLCTAG_HPP
