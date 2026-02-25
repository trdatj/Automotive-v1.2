import QtQuick 2.15

Item {
    id: icon
    property bool active: true
    property string iconImageOn: ""
    property string iconImageOff: ""
    property bool checked: false
    property bool blinking: false
    property int blinkInterval: 300

    Image {
        id: iconID
        source: icon.checked ? icon.iconImageOn : icon.iconImageOff
        anchors.fill: parent
        fillMode: Image.Stretch
        visible: icon.active

        transform: Rotation {
            origin.x: parent.width / 2
            origin.y: parent.height / 2
            angle: 0
        }
    }

    // Timer {
    //     id: internalBlinkTimer
    //     interval: icon.blinkInterval
    //     running: icon.blinking
    //     repeat: true
    //     onTriggered: {
    //         icon.checked = !icon.checked
    //     }
    // }
    onCheckedChanged: {
        // console.log("[FunctionIcon.qml] Checked changed to", checked)
        // console.log("[FunctionIcon.qml] Source changed to",
        //             checked ? iconImageOn : iconImageOff)
        iconID.source = checked ? iconImageOn : iconImageOff
    }
}
