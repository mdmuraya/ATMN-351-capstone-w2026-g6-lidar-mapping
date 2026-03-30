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
                        PLCFamily {}
                    }
                }

                RowLayout {
                     ColumnLayout {
                        id: columnLayoutControls
                        Layout.horizontalStretchFactor: 1

                        GroupBox {
                            title: "Power"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true

                            PowerActions {}
                        }



                        GroupBox {
                            title: "Jog"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            JogActions{}

                        }

                        GroupBox {
                            title: "Scan Data Capture"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            ColumnLayout {
                                anchors.fill: parent

                                Button {
                                    id: startDataCaptureButton
                                    text: qsTr("Start Data Capture")
                                    //enabled: (!plcTag?.runState)
                                    Material.background: startButton.down ? Material.Grey : Material.Green
                                    Material.foreground: "white"
                                    Layout.alignment: Qt.AlignHCenter
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                    onPressedChanged: {
                                        hmiBackendHelper?.startDataCaptureButtonClicked();
                                    }
                                }

                                Button {
                                    id: stopDataCaptureButton
                                    text: qsTr("Stop Data Capture")
                                    //enabled: plcTag?.runState ?? false
                                    Material.background: Material.Red
                                    Material.foreground: "white"
                                    Layout.alignment: Qt.AlignHCenter
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                    onPressedChanged: {
                                        hmiBackendHelper?.stopDataCaptureButtonClicked();
                                    }
                                }

                                Button {
                                    id: clearDataCaptureButton
                                    text: qsTr("Clear Data Capture")
                                    //enabled: (!plcTag?.runState)
                                    Material.background: Material.Blue
                                    Material.foreground: "white"
                                    Layout.alignment: Qt.AlignHCenter
                                    font {
                                        bold: true
                                        pointSize: 10
                                    }
                                    onPressedChanged: {
                                        hmiBackendHelper?.clearDataCaptureButtonClicked();
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
                        ScanArea {}
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

