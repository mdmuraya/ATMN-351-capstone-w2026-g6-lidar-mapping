import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {
    anchors.fill: parent

    SafetyIndicator {
        id: eStopSafetyIndicator
        Layout.alignment: Qt.AlignHCenter
        displayText: plcTag?.plcIsConnected ? plcTag?.eStop1Faulted ? "E-STOP (FLT)"  : "E-STOP" : "?? E-STOP"
        isActivated: (plcTag?.plcIsConnected && plcTag?.eStop1Activated) ?? false
    }
    SafetyIndicator {
        id: lightCurtainSafetyIndicator
        Layout.alignment: Qt.AlignHCenter
        displayText: plcTag?.plcIsConnected ? plcTag?.lightCurtain1Faulted ? "LIGHT CURTAIN (FLT)"  : "LIGHT CURTAIN" : "?? LIGHT CURTAIN"
        isActivated: (plcTag?.plcIsConnected && plcTag?.lightCurtain1Activated) ?? false
    }
    SafetyIndicator {
        id: areaScannerSafetyIndicator
        Layout.alignment: Qt.AlignHCenter
        displayText: plcTag?.plcIsConnected ? plcTag?.areaScanner1Faulted ? "AREA SCANNER (FLT)"  : "AREA SCANNER" : "?? AREA SCANNER"
        isActivated: (plcTag?.plcIsConnected && plcTag?.areaScanner1Activated) ?? false
    }
    //Item { Layout.fillWidth: true }

}
