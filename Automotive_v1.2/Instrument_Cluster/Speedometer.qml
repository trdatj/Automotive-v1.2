import QtQuick 2.15

Item {
    id: speedometer
    width: 500
    height: 500

    property string meterImage: "" //url đến ảnh mặt đồng hồ
    property string indicatorImage: "" //url đến ảnh kim
    property int angle: 148 // góc xoay kim

    //Mặt đồng hồ
    Image {
        id: meterImageID
        source: speedometer.meterImage
        width: parent.width
        height: parent.height
        anchors.centerIn: parent
    }

    //Kim đồng hồ
    Image {
        id: indicatorID
        source: speedometer.indicatorImage
        width: 182
        height: 20
        x: parent.width / 2 - rotationTransform.origin.x
        y: parent.height / 2 - rotationTransform.origin.y

        transform: Rotation {
            id: rotationTransform
            origin.x: 15
            origin.y: indicatorID.height / 2
            angle: speedometer.angle

            Behavior on angle {
                NumberAnimation {
                    duration: 300
                    easing.type: Easing.InOutQuad
                }
            }
        }
    }
}
