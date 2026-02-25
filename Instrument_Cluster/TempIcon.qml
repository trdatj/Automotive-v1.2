import QtQuick 2.15

Item {
    id: tempIcon
    width: 100
    height: 45

    property string status: "CANDEVICE"
    onStatusChanged: {
        console.log("Status changed to:", status)
        state = status.toLowerCase()
    }

    property string normalSource: ""
    property string warningSource: ""
    property string dangerousSource: ""
    property string candeviceSource: ""

    Image {
        id: iconImage
        anchors.fill: parent
        opacity: 0.9
        source: tempIcon.candeviceSource

        Behavior on source {
            PropertyAnimation {
                duration: 200
            }
        }
    }

    states: [
        State {
            name: "normal"
            PropertyChanges {
                target: iconImage
                source: tempIcon.normalSource
            }
        },
        State {
            name: "warning"
            PropertyChanges {
                target: iconImage
                source: tempIcon.warningSource
            }
        },
        State {
            name: "dangerous"
            PropertyChanges {
                target: iconImage
                source: tempIcon.dangerousSource
            }
        },
        State {
            name: "CAN_device"
            PropertyChanges {
                target: iconImage
                source: tempIcon.candeviceSource
            }
        }
    ]
}
