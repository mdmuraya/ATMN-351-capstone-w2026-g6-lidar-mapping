import QtQuick
import QtQuick3D
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import QtQuick.Controls.Material
import QtQuick3D.Helpers
import LIDARScanPointCloud2 1.0

//import LIDARMapping

ApplicationWindow {
    id: applicationWindow
    //width: 1800
    minimumWidth: 1800
    //height: 600//860
    minimumHeight: 860
    color: "#E0DFDB"
    //color: "#F6BFC2"
    visible: true
    visibility: Window.Maximized
    title: qsTr("Humber Polytechnic: Electromechanical Engineering Technology: Winter 2026 Capstone: Group 6")
    flags: Qt.Window | Qt.WindowTitleHint | Qt.WindowSystemMenuHint | Qt.WindowCloseButtonHint

    Item {
        states: [
            State {
                name: "safetyViolationState"
                when: (plcTag?.plcIsConnected && (! plcTag?.allSafetyInputsOK))
                PropertyChanges { target: applicationWindow; color: "#F6BFC2" }
            }
        ]
    }



    //flags: Qt.Window | Qt.FramelessWindowHint

    // A flag to indicate if the closing action is confirmned
    property bool quitConfirmed: false
    property string applicationName: qsTr("LIDAR Mapping HMI")


    header: ToolBar {
        contentHeight: 20
        // Use a Label for text that follows the app's style and font inheritance
        Label {
            text: applicationName
            font.bold: true
            font.pointSize: 12
            anchors.centerIn: parent
            // Center the text horizontally and vertically within the Label's bounds
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }


    contentData: Rectangle {
        id: contentDataContainer
        anchors.fill: parent
        anchors.margins: 5
        color: "transparent"

        ScrollView {
            anchors.fill: parent
            contentWidth: availableWidth
            contentHeight: availableHeight
            //clip: true // Optional: hide content outside bounds
            ColumnLayout {
                anchors.fill: parent
                anchors.rightMargin: 10
                anchors.leftMargin: 10

                RowLayout {
                    GroupBox {
                        Layout.fillWidth: true

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
                                text: "127.0.0.1"
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
                    }
                }

                RowLayout {
                     ColumnLayout {
                        id: columnLayoutControls
                        Layout.horizontalStretchFactor: 1

                        GroupBox {
                            title: "Actions"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            ColumnLayout {
                                anchors.fill: parent

                                Button {
                                    id: startButton
                                    text: qsTr("START")
                                    enabled: (!plcTag?.runState)
                                    Material.background: startButton.down ? Material.Grey : Material.Green
                                    Material.foreground: "white"
                                    Layout.alignment: Qt.AlignHCenter
                                    font {
                                        bold: true
                                        pointSize: 14
                                    }
                                    onPressedChanged: {
                                        plcTag?.startButtonPressedChanged(pressed);
                                    }
                                }

                                Button {
                                    id: stopButton
                                    text: qsTr("STOP")
                                    enabled: plcTag?.runState ?? false
                                    Material.background: Material.Red
                                    Material.foreground: "white"
                                    Layout.alignment: Qt.AlignHCenter
                                    font {
                                        bold: true
                                        pointSize: 14
                                    }
                                    onPressedChanged: {
                                        plcTag?.stopButtonPressedChanged(pressed);
                                    }
                                }

                                Button {
                                    id: resetButton
                                    text: qsTr("RESET")
                                    enabled: (!plcTag?.runState)
                                    Material.background: Material.Blue
                                    Material.foreground: "white"
                                    Layout.alignment: Qt.AlignHCenter
                                    font {
                                        bold: true
                                        pointSize: 14
                                    }
                                    onPressedChanged: {
                                        plcTag?.resetButtonPressedChanged(pressed);
                                    }
                                }
                            }
                        }



                        GroupBox {
                            title: "Jog"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            ColumnLayout {
                                anchors.fill: parent
                                RoundButton {
                                    text: qsTr("\u2B9D") //move left
                                    enabled: ((plcTag?.runState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
                                    Material.background: Material.Blue
                                    Layout.alignment: Qt.AlignHCenter
                                    onPressedChanged: {
                                        plcTag?.moveLeftButtonPressedChanged(pressed);
                                    }
                                }


                                RowLayout {
                                    Item { Layout.fillWidth: true }
                                    RoundButton {
                                        text: qsTr("\u2B9C") //move back
                                        enabled: ((plcTag?.runState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
                                        Material.background: Material.Blue
                                        onPressedChanged: {
                                            plcTag?.moveBackButtonPressedChanged(pressed);
                                        }
                                    }

                                    RoundButton {
                                        text: qsTr("HOME")
                                        enabled: ((plcTag?.runState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
                                        Material.background: Material.Blue
                                        Material.foreground: "white"
                                        font {
                                            bold: true
                                            pointSize: 12
                                        }
                                        onPressedChanged: {
                                            plcTag?.moveToHomeButtonPressedChanged(pressed);
                                        }
                                    }

                                    RoundButton {
                                        text: qsTr("\u2B9E") //move forward
                                        enabled: ((plcTag?.runState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
                                        Material.background: Material.Blue
                                        onPressedChanged: {
                                            plcTag?.moveForwardButtonPressedChanged(pressed);
                                        }
                                    }
                                    Item { Layout.fillWidth: true }
                                }

                                RoundButton {
                                    text: qsTr("\u2B9F") //move right
                                    enabled: ((plcTag?.runState ?? false) && (!(plcTag?.runStateSCAN ?? false)))
                                    Material.background: Material.Blue
                                    Layout.alignment: Qt.AlignHCenter
                                    onPressedChanged: {
                                        plcTag?.moveRightButtonPressedChanged(pressed);
                                    }
                                }
                            }
                        }
                        Item { Layout.fillHeight: true }
                        GroupBox {
                            Layout.fillWidth: true
                            ColumnLayout {
                                anchors.fill: parent
                                Button {
                                    text: qsTr("Quit HMI Application")
                                    Material.background: "black"
                                    Material.foreground: "white"
                                    font {
                                        bold: true
                                        pointSize: 12
                                    }
                                    onClicked: {
                                        confirmQuitDialog.open()
                                    }
                                }
                            }
                        }
                    }

                    GroupBox {
                        title: "Scan Area"
                        enabled: plcTag?.plcIsConnected ?? false
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.horizontalStretchFactor: 98
                        //Item { Layout.fillHeight: true }
                        ColumnLayout {
                            anchors.fill: parent

                            Rectangle {
                                id: rectScanArea
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                color: "transparent"

                                Rectangle {
                                    anchors.centerIn: parent
                                    width: rectScanArea.width * 0.93
                                    height: rectScanArea.height * 0.93
                                    //color: "#F7F7DA"
                                    color: applicationWindow.color
                                    border {
                                        width: 1
                                        color: "black"
                                    }

                                    View3D {
                                        anchors.fill: parent
                                        id: view3DNode

                                        PerspectiveCamera {
                                           id: camera
                                           z: 500
                                       }

                                       DirectionalLight {
                                           eulerRotation.x: -30
                                       }

                                       // // Terrain DEM
                                       // Model {
                                       //     geometry: HeightFieldGeometry {
                                       //         source: DEMSurface.heightMap
                                       //         extents: Qt.vector3d(100, 20, 100) // x,y,z size in world units
                                       //         smoothShading: true
                                       //     }
                                       //     materials: DefaultMaterial {
                                       //         diffuseColor: "#8c7a5b"
                                       //     }
                                       // }

                                       // Raw point cloud overlay

                                       Model {
                                           geometry: LIDARScanPointCloud2Geometry
                                           materials: PrincipledMaterial{
                                               pointSize: 1
                                               baseColor: "red"
                                           }
                                       }

                                        WasdController {
                                            controlledObject: camera
                                        }
                                    }
                                }


                                Text {
                                    anchors.top: parent.top
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    text: qsTr("\u2B9D = LEFT")
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                }

                                Text {
                                    anchors.bottom: parent.bottom
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    text: qsTr("\u2B9F = RIGHT")
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                }

                                Text {
                                    anchors.right: parent.right
                                    anchors.verticalCenter: parent.verticalCenter
                                    text: qsTr("\u2B9F = FRONT")
                                    rotation: 270
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                }

                                Text {
                                    anchors.left: parent.left
                                    anchors.verticalCenter: parent.verticalCenter
                                    text: qsTr("\u2B9D = BACK")
                                    rotation: 270
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                }

                                Text {
                                    anchors.left: parent.left
                                    anchors.bottom: parent.bottom
                                    text: qsTr("HOME")
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                }
                            }
                            RowLayout {
                                Item {
                                   Layout.fillWidth: true

                                   Rectangle {
                                        // Position the line
                                        // Set width and height to create a line
                                        width: parent.width // Stretches across the parent width
                                        height: 1            // Makes it a thin horizontal line
                                        color: "black"        // Set the line color
                                    }
                                }
                            }

                            ProgressBar {
                                id: progressBar
                                from: 0.0      // Minimum value
                                to: 210317    // Maximum value
                                value: plcTag?.stepperMotorDetectionPosition //50.36    // Current value
                                Layout.fillWidth: true

                                // Define the background (the progress bar track)
                                background: Rectangle {
                                    Layout.fillWidth: true
                                    implicitHeight: 20
                                    color: applicationWindow.color//"#e6e6e6" // Light gray track color
                                    radius: 5
                                    border.color: "#cccccc"
                                    border.width: 1
                                }

                                // Define the contentItem ( the progress indicator)
                                contentItem: Item {
                                    //implicitWidth: 300
                                    //implicitHeight: 20

                                    // Use a Rectangle and bind its width to the progress bar's visual position
                                    Rectangle {
                                        width: progressBar.visualPosition * parent.width
                                        height: parent.height
                                        radius: 5
                                        color: "steelblue"//"#4CAF50" // Green fill color

                                    }
                                }

                                // NumberAnimation on value {
                                //     from: progressBar.from
                                //     to: progressBar.to
                                //     duration:  5000
                                //     running: ((plcTag?.runState ?? false) && (plcTag?.runStateSCAN ?? false))
                                // }

                                // onValueChanged: {
                                //         if (value === to && to > 0) {
                                //             plcTag?.stopButtonPressedChanged(true)
                                //             plcTag?.stopButtonPressedChanged(false)
                                //         }
                                //     }
                            }

                            // Add a Text label to show the percentage value
                            Text {
                                text: ((progressBar.value / progressBar.to) * 100).toFixed(1) + "% complete"
                                font.bold: true
                                font.pointSize: 15
                                horizontalAlignment: Text.AlignHCenter
                                Layout.fillWidth: true
                            }
                        }
                    }

                    ColumnLayout {
                        id: columnLayoutIndicatorsId
                        Layout.horizontalStretchFactor: 1

                        GroupBox {
                            title: "System Mode/ State"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
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
                                    height: 35
                                    color: plcTag?.plcIsConnected ? (plcTag?.runStateSCAN ? "green" : "blue") : "transparent"
                                    radius: 5 // Optional: adds rounded corners
                                    Layout.fillWidth: true
                                    border {
                                        width: 1
                                        color: "black"
                                    }
                                    Text {
                                        text: plcTag?.plcIsConnected ? (plcTag?.runStateSCAN ? "SCAN" : "JOG") : "??"
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
                                    color: plcTag?.plcIsConnected ? (plcTag?.runState ? "green" : "transparent") : "transparent"
                                    radius: 5 // Optional: adds rounded corners
                                    Layout.fillWidth: true
                                    border {
                                        width: 1
                                        color: "black"
                                    }
                                    Text {
                                        id: plcStatusText
                                        text: plcTag?.plcIsConnected ? (plcTag?.runState ? "RUNNING" : "NOT RUNNING") : "??"
                                        color: plcTag?.plcIsConnected ? (plcTag?.runState ? "white" : "black") : "black"
                                        font.bold: true
                                        font.pointSize: 12
                                        anchors.centerIn: parent // Centers the text within the rectangle
                                    }
                                }
                                //Item { Layout.fillWidth: true }

                            }
                        }

                        GroupBox {
                            title: "Alarms"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            ColumnLayout {
                                anchors.fill: parent

                                SafetyIndicator {
                                    id: eStopSafetyIndicator
                                    Layout.alignment: Qt.AlignHCenter
                                    displayText: plcTag?.plcIsConnected ? plcTag?.eStop1Faulted ? "E-STOP (FLT)"  : "E-STOP" : "?? E-STOP"
                                    isActivated: (plcTag?.plcIsConnected && plcTag?.eStop1Activated) ?? false
                                }
                                SafetyIndicator {
                                    id: lightCurtainSafetyIndicator
                                    Layout.alignment: Qt.AlignHCenter
                                    displayText: plcTag?.plcIsConnected ? plcTag?.lightCurtain1Faulted ? "LIGHT CURTAIN (FLT)"  : "LIGHT CURTAIN" : "?? LIGHT CURTAIN"
                                    isActivated: (plcTag?.plcIsConnected && plcTag?.lightCurtain1Activated) ?? false
                                }
                                SafetyIndicator {
                                    id: areaScannerSafetyIndicator
                                    Layout.alignment: Qt.AlignHCenter
                                    displayText: plcTag?.plcIsConnected ? plcTag?.areaScanner1Faulted ? "AREA SCANNER (FLT)"  : "AREA SCANNER" : "?? AREA SCANNER"
                                    isActivated: (plcTag?.plcIsConnected && plcTag?.areaScanner1Activated) ?? false
                                }
                                //Item { Layout.fillWidth: true }

                            }
                        }

                        GroupBox {
                            title: "Indicator Lights"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            //Layout.fillHeight: true
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
                        }
                        GroupBox {
                            title: ""
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            ColumnLayout {
                                anchors.fill: parent

                            }

                        }

                    }
                }


                // Item { Layout.fillHeight: true }

                // RowLayout {
                //     Item {
                //        Layout.fillWidth: true

                //        Rectangle {
                //             // Position the line
                //             // Set width and height to create a line
                //             width: parent.width // Stretches across the parent width
                //             height: 1            // Makes it a thin horizontal line
                //             color: "black"        // Set the line color
                //         }
                //     }
                // }

                // RowLayout {
                //     Item { Layout.fillWidth: true }

                //     Button {
                //         text: qsTr("Quit HMI Application")
                //         Material.background: "black"
                //         Material.foreground: "white"
                //         font {
                //             bold: true
                //             pointSize: 12
                //         }
                //         onClicked: {
                //             confirmQuitDialog.open()
                //         }
                //     }

                // }

            }


        }

    }


    // footer: ToolBar {
    //     contentHeight: 20
    //     // Use a Label for text that follows the app's style and font inheritance
    //     Label {
    //         text: applicationName
    //         font.bold: true
    //         font.pointSize: 12
    //         anchors.centerIn: parent
    //         // Center the text horizontally and vertically within the Label's bounds
    //         horizontalAlignment: Text.AlignHCenter
    //         verticalAlignment: Text.AlignVCenter
    //     }
    // }


    MessageDialog {
        id: confirmQuitDialog
        title: qsTr("Confirm")
        text: "Do you want to quit?"
        //: StandardIcon.Question
        buttons: MessageDialog.Yes | MessageDialog.No

        // Handler for when the user clicks the "OK" button or dismisses the dialog
        onAccepted: {
            applicationWindow.quitConfirmed = true;
            applicationWindow.close();
            Qt.quit()
        }
    }

    // Handler for the window's closing signal
    onClosing: (close) => {
        // If not confirmed, ignore the close event and show the dialog
        if (!quitConfirmed) {
            close.accepted = false; // Prevent the window from closing immediately
            confirmQuitDialog.open(); // Open the confirmation dialog
        }
    }

    Connections {
        target: lidarScan2DData // The context property name
        function onScanDataChanged  () {
            scanCanvas.requestPaint();
        }
    }
}

