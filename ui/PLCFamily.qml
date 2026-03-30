import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

RowLayout {
    spacing: 10
    Label {
        id: plcFamilyLabel
        text: qsTr("PLC Family:")
        MouseArea {
            anchors.fill: parent
            onClicked: plcFamilyComboBox.forceActiveFocus()
        }
    }

    ComboBox {
        id: plcFamilyComboBox
        implicitWidth: 250
        enabled: (!plcTag?.plcIsConnected)
        model: hmiBackendHelper?.listOfPLCFamily
        //currentValue: hmiBackendHelper?.plcAddress
        textRole: "plcFamilyDescription"
        valueRole: "plcFamilyId"
        onCurrentValueChanged: {
            console.debug("Selected item index:", currentIndex)
            console.debug("Selected item text:", currentText)
            console.debug("Selected item value:", currentValue)
            hmiBackendHelper.plcFamilyId = plcFamilyComboBox.currentValue
        }
    }

    Label {
        id: plcIPAddress
        text: qsTr("PLC IP Address:")
        MouseArea {
            anchors.fill: parent
            onClicked: plcIPAddressTextField.forceActiveFocus()
        }
    }

    TextField {
        id: plcIPAddressTextField
        enabled: (!plcTag?.plcIsConnected)
        font.bold: true // Sets the font for the entire TextField to bold
        font.pointSize: 14 // Optional: Adjust the font size
        implicitWidth: 200
        text: "192.168.1.102"
        // Restrict input to digits and dots
        //inputMask: "000.000.000.000;_"
        validator: RegularExpressionValidator {
            // Regex for an IPv4 address (0-255 in each octet)
            regularExpression: /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/
        }
        onEditingFinished: {
            if (plcIPAddressTextField.acceptableInput) {
                console.log("Valid IP Address:", plcIPAddressTextField.text);
                hmiBackendHelper.plcAddress = plcIPAddressTextField.text
            } else {
                console.log("Invalid IP Address format");
                // Optional: Provide visual feedback for invalid input
            }
        }

    }

    Button {
        id: connectToPLCButton
        text: plcTag?.plcIsConnected ? qsTr("Disconnect") : qsTr("Connect")
        enabled: (!(plcTag == null))
        Material.background: connectToPLCButton.down ? Material.Grey : Material.Green
        Material.foreground: "white"
        Layout.alignment: Qt.AlignHCenter
        font {
            bold: true
            pointSize: 14
        }
        onClicked: {

            if(plcTag?.plcIsConnected)
            {
                hmiBackendHelper.disconnectFromPLC()
            }
            else
            {
                // Check validation state before using the IP
                if (plcIPAddressTextField.acceptableInput) {
                    // Use the IP address, for example with a QHostAddress in C++
                    console.log("Connecting to:", plcIPAddressTextField.text);
                    hmiBackendHelper.connectToPLC()
                }
            }

        }
    }
}
