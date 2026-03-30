import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {

    PilotLight {
        id: greenPilotLight
        Layout.alignment: Qt.AlignHCenter
        color: "green"
        text: "GREEN"
        isOn: plcTag?.greenPilotLight ?? false
    }PilotLight {
        id: redPilotLight
        Layout.alignment: Qt.AlignHCenter
        color: "red"
        text: "RED"
        isOn: plcTag?.redPilotLight ?? false
    }
    PilotLight {
        id: amberPilotLight
        Layout.alignment: Qt.AlignHCenter
        color: "#FFBF00"
        text: "AMBER"
        isOn: plcTag?.amberPilotLight ?? false
    }

    PilotLight {
        id: bluePilotLight
        Layout.alignment: Qt.AlignHCenter
        color: "blue"
        text: "BLUE"
        isOn: plcTag?.bluePilotLight ?? false
    }
    PilotLight {
        id: whitePilotLight
        Layout.alignment: Qt.AlignHCenter
        color: "white"
        text: "WHITE"
        isOn: plcTag?.whitePilotLight ?? false
    }
}
