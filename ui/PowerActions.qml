import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {
    anchors.fill: parent
    Button {
        id: startButton
        text: qsTr("ON")
        enabled: (!plcTag?.powerState)
        Material.background: startButton.down ? Material.Grey : Material.Green
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            pointSize: 14
        }
        onPressedChanged: {
            plcTag?.powerOnButtonPressedChanged(pressed);
        }
    }

    Button {
        id: stopButton
        text: qsTr("OFF")
        enabled: plcTag?.powerState ?? false
        Material.background: Material.Red
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            pointSize: 14
        }
        onPressedChanged: {
            plcTag?.powerOffButtonPressedChanged(pressed);
        }
    }

    // Button {
    //     id: resetButton
    //     text: qsTr("RESET")
    //     enabled: (!plcTag?.runState)
    //     Material.background: Material.Blue
    //     Material.foreground: "white"
    //     Layout.alignment: Qt.AlignHCenter
    //     font {
    //         bold: true
    //         pointSize: 14
    //     }
    //     onPressedChanged: {
    //         plcTag?.resetButtonPressedChanged(pressed);
    //     }
    // }

}



