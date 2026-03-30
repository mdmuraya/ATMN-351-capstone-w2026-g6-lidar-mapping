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
                            ScanDataCapture {}
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
                            SystemState {}
                        }

                        GroupBox {
                            title: "Alarms"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            Alarms {}
                        }

                        GroupBox {
                            title: "Indicator Lights"
                            enabled: plcTag?.plcIsConnected ?? false
                            Layout.fillWidth: true
                            IndicatorLights {}
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

