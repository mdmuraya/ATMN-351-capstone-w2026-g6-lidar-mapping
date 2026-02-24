import QtQuick
import QtQuick.Layouts

Item {
    id: pilotLight
    width: containerRectangle.width
    height: containerRectangle.height
    //property alias color: containerRectangle.color
    //property alias radius:  containerRectangle.radius
    //property alias borderColor: containerRectangle.border.color
    property alias text: containerText.text
    property bool isOn: false
    property color color: "transparent"


    states: [
        State {
            name: "onState"
            when: pilotLight.isOn
            PropertyChanges { target: containerRectangle; color: pilotLight.color }
            PropertyChanges { target: containerText; text: "" }
        }
    ]

    Rectangle {
        id: containerRectangle
        width: 150
        height: 35
        radius: 5
        color: "transparent"
        border {
            width: 1
            color: pilotLight.color
        }

        Text {
            id: containerText
            color: pilotLight.color
            text: ""
            font.bold: true
            font.pointSize: 10
            anchors.centerIn: parent // Centers the text within the rectangle
        }
    }
}
