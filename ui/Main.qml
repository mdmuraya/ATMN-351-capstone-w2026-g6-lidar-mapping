import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import QtQuick.Controls.Material
//import LIDARMapping

ApplicationWindow {
    id: applicationWindow
    width: 1600
    minimumWidth: 1600
    height: 850
    minimumHeight: 850
    color: "#E0DFDB"
    visible: true
    title: qsTr("Humber Polytechnic: Electromechanical Engineering Technology: Winter 2026 Capstone: Group 6: LIDAR Mapping HMI")
    //flags: Qt.Window | Qt.FramelessWindowHint

    // A flag to indicate if the closing action is confirmned
    property bool quitConfirmed: false

    /*
    header: Rectangle {
        height: 60
        color: "gray"
    }
    */
    contentData: Rectangle {
        id: contentDataContainer
        anchors.fill: parent
        anchors.margins: 5
        color: "transparent"

        ColumnLayout {
            anchors.fill: parent
            anchors.rightMargin: 10
            anchors.leftMargin: 10

            RowLayout {
                /*
                Frame {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.horizontalStretchFactor: 1

                    Rectangle {
                        anchors.fill: parent
                        Text {
                            id: controlPanelTextId
                            anchors.fill: parent
                            rotation: 270
                            text: qsTr("CONTROL PANEL")
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font {
                                bold: true
                                pointSize: 12
                            }
                        }
                    }
                }
                */

                ColumnLayout {
                    id: columnLayoutControls
                    Layout.horizontalStretchFactor: 1

                    GroupBox {
                        title: "System State"
                        Layout.fillWidth: true                        
                        ColumnLayout {
                            anchors.fill: parent

                            Item { Layout.fillWidth: true }

                            RadioButton {
                                id: runStateOFF
                                text: "OFF"
                                checked: MainBackendHelper.runStateOFF;
                                onCheckedChanged: MainBackendHelper.runStateOFF = checked
                            }

                            RadioButton {
                                id: runStateJOG
                                text: "RUN - JOG"
                                checked: MainBackendHelper.runStateJOG;
                                onCheckedChanged: MainBackendHelper.runStateJOG = checked
                            }

                            RadioButton {
                                id: runStateAUTO
                                text: "RUN - AUTO"
                                checked: MainBackendHelper.runStateAUTO;
                                onCheckedChanged: MainBackendHelper.runStateAUTO = checked
                                //Binding { target: MainBackendHelper; property: "runStateAUTO"; value: runStateAUTO.checked }
                            }
                            /*

                            Label {
                                text: qsTr("MANUAL")
                                font {
                                    bold: true
                                    pointSize: 14
                                }
                            }
                            Switch {
                               checked: false
                            }
                            Label {
                                text: qsTr("AUTO")
                                font {
                                    bold: true
                                    pointSize: 14
                                }
                            }
                            */

                            Item { Layout.fillWidth: true }

                        }
                    }

                    GroupBox {
                        title: "Actions"
                        Layout.fillWidth: true
                        ColumnLayout {
                            anchors.fill: parent
                            Button {
                                id: startButton
                                text: qsTr("START")
                                Material.background: startButton.down ? Material.Grey : Material.Green
                                Material.foreground: "white"
                                Layout.alignment: Qt.AlignHCenter
                                font {
                                    bold: true
                                    pointSize: 14
                                }
                                onClicked: {
                                    MainBackendHelper.onStartClicked();
                                }
                                onPressed: {
                                    MainBackendHelper.onStartPressed();
                                }
                                onReleased: {
                                    MainBackendHelper.onStartReleased();
                                }
                            }

                            Button {
                                text: qsTr("STOP")
                                Material.background: Material.Red
                                Material.foreground: "white"
                                Layout.alignment: Qt.AlignHCenter
                                font {
                                    bold: true
                                    pointSize: 14
                                }
                                onClicked: {
                                    MainBackendHelper.onStopClicked();
                                }
                                onPressed: {
                                    MainBackendHelper.onStopPressed();
                                }
                                onReleased: {
                                    MainBackendHelper.onStopReleased();
                                }
                            }

                            Button {
                                text: qsTr("RESET")
                                Material.background: Material.Blue
                                Material.foreground: "white"
                                Layout.alignment: Qt.AlignHCenter
                                font {
                                    bold: true
                                    pointSize: 14
                                }
                                onClicked: {
                                    MainBackendHelper.onResetSystem();
                                }
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }

                    GroupBox {
                        title: "Jog"
                        Layout.fillWidth: true
                        ColumnLayout {
                            anchors.fill: parent
                            RoundButton {
                                text: qsTr("\u2B9D") //move left
                                Material.background: Material.Yellow
                                Layout.alignment: Qt.AlignHCenter
                                onClicked: {
                                    MainBackendHelper.onMoveLeft();
                                }
                            }


                            RowLayout {
                                Item { Layout.fillWidth: true }
                                RoundButton {
                                    text: qsTr("\u2B9C") //move back
                                    Material.background: Material.Yellow
                                    onClicked: {
                                        MainBackendHelper.onMoveBack();
                                    }
                                }

                                RoundButton {
                                    text: qsTr("HOME")
                                    Material.background: "black"
                                    Material.foreground: "white"
                                    font {
                                        bold: true
                                        pointSize: 12
                                    }
                                    onClicked: {
                                        MainBackendHelper.onMoveToHome();
                                    }
                                }

                                RoundButton {
                                    text: qsTr("\u2B9E") //move forward
                                    Material.background: Material.Yellow
                                    onClicked: {
                                        MainBackendHelper.onMoveForward();
                                    }
                                }
                                Item { Layout.fillWidth: true }
                            }

                            RoundButton {
                                text: qsTr("\u2B9F") //move right
                                Material.background: Material.Yellow
                                Layout.alignment: Qt.AlignHCenter
                                onClicked: {
                                    MainBackendHelper.onMoveRight();
                                }
                            }
                        }
                    }
                }

                GroupBox {
                    title: "Scan Area"
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
                            color: "#F7F7DA"
                            Text {
                                anchors.centerIn: parent
                                text: qsTr("THIS IS THE SCAN AREA")
                                font {
                                    bold: true
                                    pointSize: 20
                                }
                            }

                            Text {
                                anchors.top: parent.top
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: qsTr("\u2B9D")
                                font {
                                    bold: true
                                    pointSize: 12
                                }
                            }

                            Text {
                                anchors.bottom: parent.bottom
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: qsTr("\u2B9F")
                                font {
                                    bold: true
                                    pointSize: 12
                                }
                            }

                            Text {
                                anchors.right: parent.right
                                anchors.verticalCenter: parent.verticalCenter
                                text: qsTr("\u2B9E")
                                font {
                                    bold: true
                                    pointSize: 12
                                }
                            }

                            Text {
                                anchors.left: parent.left
                                anchors.verticalCenter: parent.verticalCenter
                                text: qsTr("\u2B9C")
                                font {
                                    bold: true
                                    pointSize: 12
                                }
                            }

                            Text {
                                anchors.left: parent.left
                                anchors.bottom: parent.bottom
                                text: qsTr("HOME")
                                font {
                                    bold: true
                                    pointSize: 12
                                }
                            }
                        }

                        ProgressBar {
                                id: progressBar
                                from: 0.0      // Minimum value
                                to: 100.0     // Maximum value
                                value: 50.36    // Current value
                                Layout.fillWidth: true

                                // Define the background (the progress bar track)
                                background: Rectangle {
                                    Layout.fillWidth: true
                                    implicitHeight: 20
                                    color: "#e6e6e6" // Light gray track color
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
                                        color: "#4CAF50" // Green fill color
                                    }
                                }
                            }

                            // Add a Text label to show the percentage value
                            Text {
                                text: progressBar.value.toFixed(0) + "% complete"
                                font.pointSize: 12
                                horizontalAlignment: Text.AlignHCenter
                                Layout.fillWidth: true
                            }

                    }
                }

                ColumnLayout {
                    id: columnLayoutIndicatorsId
                    Layout.horizontalStretchFactor: 1

                    // Items inside a Layout should use Layout attached properties, not anchors
                    GroupBox {
                        title: "Indicator Lights"
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        ColumnLayout {
                            anchors.fill: parent
                            PilotLight {
                                id: redPilotLight
                                color: "red"
                                Layout.alignment: Qt.AlignHCenter
                            }
                            PilotLight {
                                id: amberPilotLight
                                color: "#FFBF00"
                                Layout.alignment: Qt.AlignHCenter
                            }
                            PilotLight {
                                id: greenPilotLight
                                color: "green"
                                Layout.alignment: Qt.AlignHCenter
                            }
                            PilotLight {
                                id: bluePilotLight
                                color: "blue"
                                Layout.alignment: Qt.AlignHCenter
                            }
                            PilotLight {
                                id: whitePilotLight
                                color: "white"
                                Layout.alignment: Qt.AlignHCenter
                            }
                        }
                    }

                    GroupBox {
                        title: "PLC"
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        ColumnLayout {
                            anchors.fill: parent



                            Rectangle {
                                    id: plcStatus
                                    width: 150
                                    height: 75
                                    color: MainBackendHelper.plcRunTag ? "green..." : "transparent"
                                    radius: 10 // Optional: adds rounded corners
                                    anchors.centerIn: parent // Centers the rectangle within the window

                                    Text {
                                        id: plcStatusText
                                        text: MainBackendHelper.plcRunTag ? "RUNNING..." : "OFF"
                                        color: "white"
                                        font.bold: true
                                        font.pointSize: 16
                                        anchors.centerIn: parent // Centers the text within the rectangle
                                    }
                                }


                        }
                    }


                }
            }

            Item { Layout.fillHeight: true }

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

            RowLayout {
                Item { Layout.fillWidth: true }

                Button {
                    text: qsTr("Quit HMI Application")
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
    /*
    footer: Rectangle {
        height: 40
        color: "gray"
    }
    */

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
}

