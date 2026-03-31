import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {
    anchors.fill: parent
    // RoundButton {
    //     text: qsTr("\u2B9D") //move left
    //     enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
    //     Material.background: Material.Blue
    //     Layout.alignment: Qt.AlignHCenter
    //     onPressedChanged: {
    //         plcTag?.moveLeftButtonPressedChanged(pressed);
    //     }
    // }

    RoundButton {
        text: qsTr("HOME")
        enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
        Material.background: Material.Blue
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            //pointSize: 12
        }
        onPressedChanged: {
            plcTag?.moveToHomeButtonPressedChanged(pressed);
        }
    }


    RowLayout {
        Item { Layout.fillWidth: true }
        RoundButton {
            text: qsTr("\u2B9C") //move back
            enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
            Material.background: Material.Blue
            onPressedChanged: {
                plcTag?.moveBackButtonPressedChanged(pressed);
            }
        }



        RoundButton {
            text: qsTr("STOP")
            enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
            Material.background: Material.Red
            Material.foreground: "white"
            font {
                bold: true
                //pointSize: 12
            }
            onPressedChanged: {
                plcTag?.stopMotionButtonPressedChanged(pressed);
            }
        }



        RoundButton {
            text: qsTr("\u2B9E") //move forward
            enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
            Material.background: Material.Blue
            onPressedChanged: {
                plcTag?.moveForwardButtonPressedChanged(pressed);
            }
        }
        Item { Layout.fillWidth: true }
    }

    RoundButton {
        text: qsTr("END")
        enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
        Material.background: Material.Blue
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            //pointSize: 12
        }
        onPressedChanged: {
            plcTag?.moveToEndButtonPressedChanged(pressed);
        }
    }

    // RoundButton {
    //     text: qsTr("\u2B9F") //move right
    //     enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
    //     Material.background: Material.Blue
    //     Layout.alignment: Qt.AlignHCenter
    //     onPressedChanged: {
    //         plcTag?.moveRightButtonPressedChanged(pressed);
    //     }
    // }

    // RoundButton {
    //     text: qsTr("HOME")
    //     enabled: ((plcTag?.powerState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
    //     Material.background: Material.Blue
    //     Material.foreground: "white"
    //     font {
    //         bold: true
    //         pointSize: 12
    //     }
    //     onPressedChanged: {
    //         plcTag?.moveToHomeButtonPressedChanged(pressed);
    //     }
    // }
}



