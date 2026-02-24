import QtQuick
import QtQuick.Layouts

Item {
    id: pilotLight
    width: containerRectangle.width
    height: containerRectangle.height
    property alias color: containerRectangle.color
    property alias radius:  containerRectangle.radius
    property alias borderColor: containerRectangle.border.color
    property bool isOn: false

    states: [
        State {
            name: "onState"
            when: pilotLight.isOn
            PropertyChanges { target: containerRectangle; color: borderColor }
        }
    ]

    Rectangle {
        id: containerRectangle
        width: (radius * 2)
        height: width
        radius: 25
        color: "transparent"
        border {
            width: 1
            color: "black"
        }
    }
}
