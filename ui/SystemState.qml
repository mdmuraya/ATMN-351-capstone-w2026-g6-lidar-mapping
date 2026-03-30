import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

ColumnLayout {
    anchors.fill: parent

    //Item { Layout.fillWidth: true }
    Rectangle {
        width: 150
        height: 35
        color: plcTag?.plcIsConnected ? "green" : "transparent"
        radius: 5 // Optional: adds rounded corners
        Layout.fillWidth: true
        border {
            width: 1
            color: "black"
        }
        Text {
            text: plcTag?.plcIsConnected ? "CONNECTED" : "NOT CONNECTED"
            color: plcTag?.plcIsConnected ? "white" : "black"
            font.bold: true
            font.pointSize: 12
            anchors.centerIn: parent // Centers the text within the rectangle
        }
    }

    Rectangle {
        //width: 150
        //Layout.fillWidth: true
        height: 35
        color: plcTag?.plcIsConnected ? (plcTag?.powerState ? "green" : "transparent") : "transparent"
        radius: 5 // Optional: adds rounded corners
        Layout.fillWidth: true
        border {
            width: 1
            color: "black"
        }
        Text {
            id: plcStatusText
            text: plcTag?.plcIsConnected ? (plcTag?.powerState ? "POWER ON" : "POWER OFF") : "?? POWER"
            color: plcTag?.plcIsConnected ? (plcTag?.powerState ? "white" : "black") : "black"
            font.bold: true
            font.pointSize: 12
            anchors.centerIn: parent // Centers the text within the rectangle
        }
    }

    Rectangle {
        //width: 150
        height: 35
        color: (plcTag?.plcIsConnected && plcTag?.powerState) ? (plcTag?.runStateSCAN ? "green" : "blue") : "transparent"
        radius: 5 // Optional: adds rounded corners
        Layout.fillWidth: true
        border {
            width: 1
            color: "black"
        }
        Text {
            text: (plcTag?.plcIsConnected && plcTag?.powerState) ? (plcTag?.runStateSCAN ? "SCAN" : "JOG") : "?? SCAN / JOG"
            color: (plcTag?.plcIsConnected && plcTag?.powerState) ? "white" : "black"
            font.bold: true
            font.pointSize: 12
            anchors.centerIn: parent // Centers the text within the rectangle
        }
    }

    //Item { Layout.fillWidth: true }

}


