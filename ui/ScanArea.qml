import QtQuick
import QtQuick3D
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import QtQuick.Controls.Material
import QtQuick3D.Helpers
import LIDARScanPointCloud2 1.0

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
                   position: Qt.vector3d(5, 0, 30)
                   //z: 30
                   eulerRotation.z: 90
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

        Label  {
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            text: qsTr("\u2B9F = FRONT")
            rotation: 270
            font {
                bold: true
                pointSize: 10
            }
        }

        Label   {
            anchors.left: parent.left
            anchors.verticalCenter: parent.verticalCenter
            text: qsTr("\u2B9D = BACK")
            rotation: 270
            font {
                bold: true
                pointSize: 10
            }
        }

        Label  {
            anchors.left: parent.left
            anchors.bottom: parent.bottom
            text: qsTr("HOME")
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font {
                bold: true
                pointSize: 10
            }
            background: Rectangle {
                color: (plcTag?.plcIsConnected && plcTag?.powerState) ? (plcTag?.backLimitSwitch ? "#34eb7d" : "transparent") : "transparent"
                radius: 5 // Optional: rounded corners
                width: parent.width + 5 // Add some padding
                height: parent.height + 5 // Add some padding
            }
        }

        Label  {
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            text: qsTr("END")
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font {
                bold: true
                pointSize: 10
            }
            background: Rectangle {
                color: (plcTag?.plcIsConnected && plcTag?.powerState) ? (plcTag?.frontLimitSwitch ? "#34eb7d" : "transparent") : "transparent"
                radius: 5 // Optional: rounded corners
                width: parent.width + 5 // Add some padding
                height: parent.height + 5 // Add some padding
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
