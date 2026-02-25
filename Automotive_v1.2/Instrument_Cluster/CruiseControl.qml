import QtQuick 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    width: 100
    height: 100

    property int cruiseStatus: 0
    property int targetSpeed: 0

    property string normalSource: "qrc:/icons/cruise_off.png"
    property string readySource: "qrc:/icons/cruise_ready.png"
    property string activeSource: "qrc:/icons/cruise_active.png"

    ColumnLayout {
        anchors.centerIn: parent
        spacing: 5

        Image {
            id: cruiseIcon
            Layout.alignment: Qt.AlignHCenter
            Layout.preferredWidth: 60
            Layout.preferredHeight: 60
            fillMode: Image.PreserveAspectFit
            source: {
                switch (root.cruiseStatus) {
                case 1:
                    return root.readySource // Ready
                case 2:
                    return root.activeSource // Active
                default:
                    return root.normalSource // Off
                }
            }

            opacity: root.cruiseStatus === 0 ? 0.5 : 1

            Behavior on opacity {
                NumberAnimation {
                    duration: 300
                }
            }
        }

        Text {
            id: speedText
            text: "SET " + root.targetSpeed + " km/h"
            Layout.alignment: Qt.AlignHCenter

            color: "#4CAF50"
            font.pixelSize: 16
            font.bold: true
            font.family: "Arial"

            visible: root.cruiseStatus === 2
        }
    }
}
