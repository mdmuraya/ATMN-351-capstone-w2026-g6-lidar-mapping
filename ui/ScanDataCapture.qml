import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {
    anchors.fill: parent

    Button {
        id: startDataCaptureButton
        text: qsTr("Start Capture")
        enabled: (plcTag?.runStateSCAN && plcTag?.homeLimitSwitch )
        Material.background: startDataCaptureButton.down ? Material.Grey : Material.Green
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            pointSize: 10
        }
        onPressedChanged: {
            plcTag?.startDataCaptureButtonPressedChanged(pressed);
        }
        onClicked: {
            hmiBackendHelper?.startDataCaptureButtonClicked();
        }
    }

    Button {
        id: stopDataCaptureButton
        text: qsTr("Stop Capture")
        enabled: (plcTag?.runStateSCAN && !(plcTag?.endLimitSwitch || plcTag?.homeLimitSwitch) )
        Material.background: Material.Red
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            pointSize: 10
        }
        onPressedChanged: {
            plcTag?.stopDataCaptureButtonPressedChanged(pressed);
        }
        onClicked: {
            hmiBackendHelper?.stopDataCaptureButtonClicked();
        }
    }

    // Button {
    //     id: clearDataCaptureButton
    //     text: qsTr("Clear Data Capture")
    //     //enabled: (!plcTag?.runState)
    //     Material.background: Material.Blue
    //     Material.foreground: "white"
    //     Layout.alignment: Qt.AlignHCenter
    //     font {
    //         bold: true
    //         pointSize: 10
    //     }
    //     onPressedChanged: {
    //         hmiBackendHelper?.clearDataCaptureButtonClicked();
    //     }
    // }
}



