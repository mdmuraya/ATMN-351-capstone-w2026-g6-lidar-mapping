import QtQuick
import QtQuick.Layouts

Item {
    id: pilotLightId
    width: containerRectangleId.width
    height: containerRectangleId.height
    property alias color: containerRectangleId.color
    property alias radius:  containerRectangleId.radius

    Rectangle {
        id: containerRectangleId
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
