import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {
    anchors.fill: parent

    Button {
        id: startDataCaptureButton
        text: qsTr("Start Data Capture")
        //enabled: (!plcTag?.runState)
        Material.background: startDataCaptureButton.down ? Material.Grey : Material.Green
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            pointSize: 10
        }
        onPressedChanged: {
            hmiBackendHelper?.startDataCaptureButtonClicked();
        }
    }

    Button {
        id: stopDataCaptureButton
        text: qsTr("Stop Data Capture")
        //enabled: plcTag?.runState ?? false
        Material.background: Material.Red
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            pointSize: 10
        }
        onPressedChanged: {
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



