import QtQuick
import QtQuick.Controls

Item {
    id: directionButtonId
    property alias text: roundButtonId.text
    signal buttonClicked

    RoundButton {
        id: roundButtonId
        text: qsTr("\u2B9C") //left arrow
        //Material.background: Material.Yellow
        //Material.foreground: Material.White
        onClicked: {
            directionButtonId.buttonClicked()
        }
    }

}
