import QtQuick 2.15

Item {
    id: mainScreen
    width: 1920
    height: 980
    signal startClicked
    property bool active: true

    // Background khởi động
    Image {
        id: background
        anchors.fill: parent
        source: "qrc:/img/Panel.png"
        opacity: mainScreen.active ? 1 : 0

        Behavior on opacity {
            NumberAnimation {
                duration: 500
                easing.type: Easing.InOutQuad
            }
        }

        // Nút Start
        Image {
            id: startButton
            source: "qrc:/img/start_btn.png"
            width: 350
            height: 350
            anchors.centerIn: parent
            opacity: mainScreen.active ? 1 : 0

            Behavior on opacity {
                NumberAnimation {
                    duration: 300
                }
            }

            // Hiệu ứng khi hover
            states: [
                State {
                    name: "hovered"
                    when: buttonMouse.containsMouse
                    PropertyChanges {
                        target: startButton
                        scale: 1.1
                    }
                }
            ]

            transitions: Transition {
                NumberAnimation {
                    properties: "scale"
                    duration: 200
                }
            }

            MouseArea {
                id: buttonMouse
                anchors.fill: parent
                onClicked: {
                    mainScreen.startClicked()
                }
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
            }
        }
    }
}
