import QtQuick
import QtQuick3D
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import QtQuick.Controls.Material
import QtQuick3D.Helpers
import Example 1.0

//import LIDARMapping

ApplicationWindow {
    id: applicationWindow
    width: 1400
    minimumWidth: 1400
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
                 ColumnLayout {
                    id: columnLayoutControls
                    Layout.horizontalStretchFactor: 1

                    GroupBox {
                        title: "PLC Connection"
                        enabled: plcTag?.plcIsConnected
                        Layout.fillWidth: true                        
                        ColumnLayout {
                            anchors.fill: parent

                            //Item { Layout.fillWidth: true }

                            Rectangle {
                                width: 150
                                height: 50
                                color: "black"
                                radius: 5 // Optional: adds rounded corners
                                Layout.fillWidth: true
                                border {
                                    width: 1
                                    color: "black"
                                }
                                Text {
                                    text: "Address: " + plcTag?.plcAddress
                                    color: "white"
                                    font.bold: true
                                    font.pointSize: 12
                                    anchors.centerIn: parent // Centers the text within the rectangle
                                }
                            }
                            Rectangle {
                                width: 150
                                height: 50
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
                            //Item { Layout.fillWidth: true }

                        }
                    }

                    GroupBox {
                        title: "System State"
                        enabled: plcTag?.plcIsConnected
                        Layout.fillWidth: true
                        ColumnLayout {
                            anchors.fill: parent

                            //Item { Layout.fillWidth: true }

                            Rectangle {
                                width: 150
                                height: 50
                                color: plcTag?.plcIsConnected ? (plcTag?.runState ? "green" : "transparent") : "transparent"
                                radius: 5 // Optional: adds rounded corners
                                Layout.fillWidth: true
                                border {
                                    width: 1
                                    color: "black"
                                }
                                Text {
                                    id: plcStatusText
                                    text: plcTag?.plcIsConnected ? (plcTag?.runState ? "RUNNING" : "OFF") : "??"
                                    color: plcTag?.plcIsConnected ? (plcTag?.runState ? "white" : "black") : "black"
                                    font.bold: true
                                    font.pointSize: 12
                                    anchors.centerIn: parent // Centers the text within the rectangle
                                }
                            }
                            Rectangle {
                                width: 150
                                height: 50
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
                            //Item { Layout.fillWidth: true }

                        }
                    }

                    GroupBox {
                        title: "Actions"
                        enabled: plcTag?.plcIsConnected
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

                    Item { Layout.fillHeight: true }

                    GroupBox {
                        title: "Jog"
                        enabled: plcTag?.plcIsConnected
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
                }

                GroupBox {
                    title: "Scan Area"
                    enabled: plcTag?.plcIsConnected
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
                                color: "#F7F7DA"
                                border {
                                    width: 1
                                    color: "black"
                                }

                                // View3D {
                                //     anchors.fill: parent

                                //     PerspectiveCamera {
                                //         id: camera
                                //         z: 200
                                //     }

                                //     DirectionalLight {

                                //     }

                                //     Model {
                                //         source: "#Cylinder"
                                //         materials: DefaultMaterial {}
                                //         eulerRotation.y: 20
                                //         Node {
                                //             // Empty spatial Node to give 2D item
                                //             // a position in 3D space
                                //             y: 75
                                //             Text {
                                //                 // 2D content in 3D
                                //                 anchors.centerIn: parent
                                //                 text: "THIS IS THE SCAN AREA"
                                //                 color: "black"
                                //             }
                                //         }
                                //     }
                                // }
                                // WasdController{
                                //     controlledObject: camera
                                // }



                                View3D {
                                    anchors.fill: parent
                                    environment: SceneEnvironment {
                                        lightProbe: Texture{
                                            ///source: "imageTesto.hdr"
                                        }
                                        backgroundMode: SceneEnvironment.SkyBox
                                    }

                                    PerspectiveCamera {
                                        id: camera
                                        z: 300
                                    }

                                    DirectionalLight {
                                        eulerRotation.x: -30
                                    }

                                    Model {
                                        geometry: CustomGeometry{
                                            //property string name: ""
                                            //name: "mypointcloud"
                                            count: 10000000
                                        }
                                        materials: PrincipledMaterial{
                                        pointSize: 10
                                        }
                                    }
                                }





    //                             Canvas {
    //                                 id: scanCanvas
    //                                 anchors.centerIn: parent
    //                                 width: rectScanArea.width * 0.93
    //                                 height: rectScanArea.height * 0.93
    //                                 // Define the center and radius of the circle
    //                                 property int centerX: scanCanvas.width / 2
    //                                 property int centerY: scanCanvas.height / 2
    //                                 property int radius: scanCanvas.height * 1
    //                                 // Define the size and color for the points
    //                                 property int pointSize: 4
    //                                 property color pointColor: "green"

    //                                 onPaint: {
    //                                     var ctx = getContext("2d");
    //                                     ctx.clearRect(0, 0, scanCanvas.width, scanCanvas.height); // Clear the canvas
    //                                     console.log("lidarScan2DData.numberOfPoints " + lidarScan2DData.numberOfPoints)

    //                                     // Calculate and draw points
    //                                     ////var numberOfPoints = 100;
    //                                     for (var i = 0; i < lidarScan2DData.numberOfPoints; i++) {
    //                                         // Calculate the angle in radians for each point
    //                                         ////var angle = (i * 2 * Math.PI) / numberOfPoints;

    //                                         // Use polar coordinates formula to find the x, y position
    //                                         var x = centerX + (radius * (lidarScan2DData.ranges[i]/6) * Math.cos(lidarScan2DData.angleInRadians));
    //                                         var y = centerY + (radius * lidarScan2DData.ranges[i] * Math.sin(lidarScan2DData.angleInRadians));

    //                                         // Draw a small filled circle (point) at the calculated coordinates
    //                                         ctx.beginPath();
    //                                         // The arc method is used to draw a full circle for each point
    //                                         ctx.arc(x, y, pointSize / 2, 0, 2 * Math.PI, false);
    //                                         ctx.fillStyle = pointColor;
    //                                         ctx.fill();
    //                                     }
    //                                 }
    //                             }
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
                            to: 100.0     // Maximum value
                            //value: 50.36    // Current value
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
                                    color: "steelblue"//"#4CAF50" // Green fill color

                                }
                            }

                            NumberAnimation on value {
                                from: progressBar.from
                                to: progressBar.to
                                duration:  5000
                                running: ((plcTag?.runState ?? false) && (plcTag?.runStateSCAN ?? false))
                            }

                            onValueChanged: {
                                    if (value === to && to > 0) {
                                        plcTag?.stopButtonPressedChanged(true)
                                        plcTag?.stopButtonPressedChanged(false)
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
                        title: "Alarms"
                        enabled: plcTag?.plcIsConnected
                        Layout.fillWidth: true
                        ColumnLayout {
                            anchors.fill: parent

                            //Item { Layout.fillWidth: true }

                            Rectangle {
                                width: 150
                                height: 50
                                color: plcTag?.plcIsConnected ? (plcTag?.runState ? "red" : "transparent") : "transparent"
                                radius: 5 // Optional: adds rounded corners
                                Layout.fillWidth: true
                                border {
                                    width: 1
                                    color: "red"
                                }
                                Text {
                                    text: plcTag?.plcIsConnected ? "E-STOP" : "?? E-STOP"
                                    color: plcTag?.plcIsConnected ? (plcTag?.runState ? "white" : "black") : "black"
                                    font.bold: true
                                    font.pointSize: 12
                                    anchors.centerIn: parent // Centers the text within the rectangle
                                }
                            }
                            Rectangle {
                                width: 150
                                height: 50
                                color: plcTag?.plcIsConnected ? (plcTag?.runState ? "red" : "transparent") : "transparent"
                                radius: 5 // Optional: adds rounded corners
                                Layout.fillWidth: true
                                border {
                                    width: 1
                                    color: "red"
                                }
                                Text {
                                    text: plcTag?.plcIsConnected ? "LIGHT CURTAIN" : "?? LIGHT CURTAIN"
                                    color: plcTag?.plcIsConnected ? (plcTag?.runState ? "white" : "black") : "black"
                                    font.bold: true
                                    font.pointSize: 12
                                    anchors.centerIn: parent // Centers the text within the rectangle
                                }
                            }
                            Rectangle {
                                width: 150
                                height: 50
                                color: plcTag?.plcIsConnected ? (plcTag?.runState ? "red" : "transparent") : "transparent"
                                radius: 5 // Optional: adds rounded corners
                                Layout.fillWidth: true
                                border {
                                    width: 1
                                    color: "red"
                                }
                                Text {
                                    text: plcTag?.plcIsConnected ? "AREA SCANNER" : "?? AREA SCANNER"
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
                        title: "Indicator Lights"
                        enabled: plcTag?.plcIsConnected
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        ColumnLayout {
                            anchors.fill: parent
                            PilotLight {
                                id: redPilotLight
                                Layout.alignment: Qt.AlignHCenter
                                borderColor: "red"
                                Binding on color {
                                    value: redPilotLight.borderColor
                                    when: plcTag?.redPilotLight ?? false
                                }
                            }
                            PilotLight {
                                id: amberPilotLight
                                Layout.alignment: Qt.AlignHCenter
                                borderColor: "#FFBF00"
                                Binding on color {
                                    value: amberPilotLight.borderColor
                                    when: plcTag?.amberPilotLight ?? false
                                }
                            }
                            PilotLight {
                                id: greenPilotLight
                                Layout.alignment: Qt.AlignHCenter
                                borderColor: "green"
                                Binding on color {
                                    value: greenPilotLight.borderColor
                                    when: plcTag?.greenPilotLight ?? false
                                }
                            }
                            PilotLight {
                                id: bluePilotLight
                                Layout.alignment: Qt.AlignHCenter
                                borderColor: "blue"
                                Binding on color {
                                    value: bluePilotLight.borderColor
                                    when: plcTag?.bluePilotLight ?? false
                                }
                            }
                            PilotLight {
                                id: whitePilotLight
                                Layout.alignment: Qt.AlignHCenter
                                borderColor: "white"
                                Binding on color {
                                    value: greenPilotLight.borderColor
                                    when: plcTag?.whitePilotLight ?? false
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

    Connections {
        target: lidarScan2DData // The context property name
        // Signal handler format: on<SignalName>
        onScanDataChanged: {
            //console.log("QML received signal:", message, value)
            //statusText.text = "Received: " + message + " with value " + value
            scanCanvas.requestPaint();
        }
    }
}

