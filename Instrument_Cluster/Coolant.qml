import QtQuick 2.15
//import QtQuick.Effects
import QtGraphicalEffects 1.15

Item {
    id: root
    property real value: 0.5
    width: 200
    height: 200

    //Qt.hsla( Hue, Saturation, Lightness, Alpha )
    property color dynamicColor: Qt.hsla(root.value * 0.33, 1.0, 0.60, 1.0)

    Image {
        id: temp_warning
        source: "qrc:/icons/icons-left/Coolant/temp_coolant_icon.png"
        width: 50
        height: 50

        anchors.top: parent.top
        anchors.topMargin: -45
        anchors.left: parent.left
        anchors.leftMargin: 0
        smooth: true
        visible: false
    }

    ColorOverlay {
        anchors.fill: temp_warning
        source: temp_warning

        //colorization: 1.0
        //colorizationColor: root.value < 0.2 ? "#FF0000" : "#FFFFFF"
        color: root.value < 0.2 ? "#FF0000" : "#FFFFFF"

        opacity: (root.value < 0.15 && blinkTimer.running) ? 0.2 : 1.0
    }

    Timer {
        id: blinkTimer
        interval: 500
        running: root.value < 0.15
        repeat: true
    }

    Image {
        id: background
        source: "qrc:/icons/icons-left/Coolant/temp_coolant.png"

        smooth: true
        visible: false

        anchors.fill: parent
        fillMode: Image.PreserveAspectFit
    }

    ColorOverlay {
        anchors.fill: background
        source: background

        // phủ màu
        // colorization: 1.0 // phủ màu 100%
        // colorizationColor: "#FFFFFF"
        //color: root.value < 0.2 ? "#FF0000" : "#FFFFFF"
        color: "#FFFFFF"
        opacity: 0.3
    }

    Item {
        id: clipper
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: parent.height * root.value

        clip: true
        Behavior on height {
            NumberAnimation {
                duration: 500
            }
        }

        Image {
            id: fgFullSource
            source: "qrc:/icons/icons-left/Coolant/temp_coolant.png"

            height: root.height
            width: root.width
            anchors.bottom: parent.bottom

            fillMode: Image.PreserveAspectFit
            visible: false
        }

        ColorOverlay {
            anchors.fill: fgFullSource
            source: fgFullSource

            //colorization: 1.0
            //colorizationColor: root.dynamicColor
            color: root.dynamicColor

            // Hiệu ứng đổi màu mượt
            Behavior on color {
                ColorAnimation {
                    duration: 300
                }
            }
        }
    }
}
