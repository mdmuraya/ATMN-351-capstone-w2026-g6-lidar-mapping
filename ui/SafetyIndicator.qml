import QtQuick
import QtQuick.Layouts

Item {
    id: safetyIndicator
    width: containerRectangle.width
    height: containerRectangle.height
    property alias displayText: containerText.text
    property bool isActivated: false

    states: [
        State {
            name: "activatedState"
            when: safetyIndicator.isActivated
            PropertyChanges { target: containerRectangle; color: "red" }
            PropertyChanges { target: containerText; color: "white" }
        }
    ]

    Rectangle {
        id: containerRectangle
        width: 150
        height: 35
        color: "transparent"
        radius: 5 // Optional: adds rounded corners
        Layout.fillWidth: true
        border {
            width: 1
            color: "red"
        }
        Text {
            id: containerText
            color: "black"
            font.bold: true
            font.pointSize: 10
            anchors.centerIn: parent // Centers the text within the rectangle
        }
    }
}
