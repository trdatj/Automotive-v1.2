import QtQuick 2.15
import QtQuick.Window 2.15

Window {
    id: root
    width: 1920
    height: 980
    visible: true
    visibility: "FullScreen"
    title: qsTr("Instrument Cluster")
    property bool showMainContent: false
    color: "#141414"

    //property string currentSignSource: ""
    property int simulatedStatus: 0
    property int simulatedSpeed: 40

    property string speedLimitValue: "---"
    property string currentGear: "P"
    property double currentRealSpeed: 0.0

    property bool isManualMode: false

    Shortcut {
        sequence: "Esc"
        onActivated: Qt.quit()
    }

    Item {
        id: content
        width: 1920
        height: 980

        anchors.centerIn: parent

        scale: Math.min(root.width / 1920,
                        root.height / 980) // scale toàn bộ nội dung
        Behavior on opacity {
            NumberAnimation {
                duration: 500
                easing.type: Easing.InOutQuad
            }
        }

        //Background
        Image {
            id: background
            source: "qrc:/img/Panel.png"
            anchors.fill: parent
            fillMode: Image.PreserveAspectFit
            //fillMode: Image.Stretch
            property int currentAngle: 150

            //Mặt đồng hồ trái
            Speedometer {
                id: speedometerLeft
                width: 500
                height: 500
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: parent.left
                anchors.leftMargin: 130

                meterImage: "qrc:/img/speedImg.png"
                indicatorImage: "qrc:/img/Indicator.png"
                angle: 148
            }

            //Coolant
            Coolant {
                id: coolant
                width: 300
                height: 300

                anchors.left: speedometerLeft.left
                anchors.leftMargin: -60
                anchors.bottom: speedometerLeft.bottom
                anchors.bottomMargin: -20
                value: 0.75
            }

            //Mặt đồng hồ phải
            Speedometer {
                id: speedometerRight
                width: 500
                height: 500
                anchors.verticalCenter: parent.verticalCenter
                anchors.right: parent.right
                anchors.rightMargin: 130

                meterImage: "qrc:/img/tachoImg.png"
                indicatorImage: "qrc:/img/Indicator.png"
                angle: 150
            }

            //Fuel
            Fuel_gauge {
                id: fuel_gauge
                width: 300
                height: 300

                anchors.right: speedometerRight.right
                anchors.rightMargin: -60
                anchors.bottom: speedometerRight.bottom
                anchors.bottomMargin: -20
                value: 0.5
            }

            Image {
                id: lane
                source: "qrc:/icons/Road/road.png"
                width: 725
                height: 390
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.centerIn: parent

                Image {
                    id: car
                    source: "qrc:/icons/Road/car.png"
                    //anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.right: parent.right
                    anchors.rightMargin: 250
                }

                Canvas {
                    id: gapDisplay
                    anchors.horizontalCenter: car.horizontalCenter
                    anchors.top: car.top
                    anchors.topMargin: -100
                    width: 180
                    height: 100
                    property int currentGap: 3

                    onCurrentGapChanged: requestPaint()

                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.reset()

                        // Cấu hình nét vẽ
                        ctx.lineWidth = 6
                        ctx.lineCap = "round"

                        var centerX = width / 2
                        var centerY = height

                        var startAngle = Math.PI * 1.2
                        var endAngle = Math.PI * 1.8

                        // --- vạch 1 ---
                        ctx.beginPath()
                        ctx.strokeStyle = currentGap >= 1 ? "#4CAF50" : "#444444"
                        // arc(x, y, bán kính, góc bắt đầu, góc kết thúc)
                        ctx.arc(centerX, centerY, 30, startAngle, endAngle)
                        ctx.stroke()

                        // --- vạch 2 ---
                        ctx.beginPath()
                        ctx.strokeStyle = currentGap >= 2 ? "#4CAF50" : "#444444"
                        ctx.arc(centerX, centerY, 55, startAngle, endAngle)
                        ctx.stroke()

                        // --- vạch 3 ---
                        ctx.beginPath()
                        ctx.strokeStyle = currentGap >= 3 ? "#4CAF50" : "#444444"
                        ctx.arc(centerX, centerY, 80, startAngle, endAngle)
                        ctx.stroke()
                    }
                }
            }

            //----xi nhan trái----
            FuncIcon {
                id: turnLeftID
                width: 120
                height: 80
                checked: false
                anchors.leftMargin: -80
                anchors.left: topbarID.left
                anchors.verticalCenter: topbarID.verticalCenter
                iconImageOff: "qrc:/icons/icons-left/xi_nhan_trai.svg"
                iconImageOn: "qrc:/icons/icons-left-checked/xi_nhan_trai_checked.svg"

                Timer {
                    id: blinkTimerLeft
                    interval: turnLeftID.blinkInterval
                    running: turnLeftID.blinking
                    repeat: true
                    onTriggered: turnLeftID.checked = !turnLeftID.checked
                }
            }

            //----xi nhan phải----
            FuncIcon {
                id: turnRightID
                width: 120
                height: 80
                anchors.rightMargin: -80
                anchors.right: topbarID.right
                anchors.verticalCenter: topbarID.verticalCenter
                checked: false

                iconImageOff: "qrc:/icons/icons-right/xi_nhan_phai.svg"
                iconImageOn: "qrc:/icons/icons-right-checked/xi_nhan_phai_checked.svg"

                Timer {
                    id: blinkTimerRight
                    interval: turnRightID.blinkInterval
                    running: turnRightID.blinking
                    repeat: true
                    onTriggered: turnRightID.checked = !turnRightID.checked
                }
            }

            //----top bar----
            Image {
                id: topbarID
                width: 960
                height: 110
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.topMargin: 80
                source: "qrc:/img/top.png"

                //logo
                Image {
                    id: logoID
                    source: "qrc:/icons/nissan-6.svg"
                    width: 100
                    height: 85
                    anchors.centerIn: topbarID
                }

                // Cruise control
                CruiseControl {
                    id: cruise
                    anchors.left: logoID.left
                    anchors.leftMargin: -120
                    anchors.verticalCenter: topbarID.verticalCenter
                    cruiseStatus: simulatedStatus
                    targetSpeed: simulatedSpeed

                    normalSource: "qrc:/icons/cruise_off.png"
                    readySource: "qrc:/icons/cruise_ready.png"
                    activeSource: "qrc:/icons/cruise_active.png"
                }

                //Phanh tay
                FuncIcon {
                    id: brakekID
                    width: 60
                    height: 60
                    opacity: 0.8
                    anchors.left: topbarID.left
                    anchors.leftMargin: 210
                    anchors.verticalCenter: topbarID.verticalCenter
                    checked: false

                    iconImageOn: ""
                    iconImageOff: "qrc:/icons/parking-brake.png"
                }

                //----cos----
                FuncIcon {
                    id: cosID
                    width: 60
                    height: 60
                    opacity: 0.9
                    anchors.right: topbarID.right
                    anchors.rightMargin: 210
                    anchors.verticalCenter: topbarID.verticalCenter
                    checked: false

                    // iconImageOn: "qrc:/icons/icons-left-checked/light_cos_checked.svg"
                    // iconImageOff: "qrc:/icons/icons-left/light_cos.svg"
                    iconImageOn: "qrc:/icons/icons-left-checked/low_beam_active.png"
                    iconImageOff: "qrc:/icons/icons-left/low_beam.png"
                }

                //----pha----
                FuncIcon {
                    id: phaID
                    width: 60
                    height: 60
                    opacity: 0.9
                    anchors.right: topbarID.right
                    anchors.rightMargin: 100
                    anchors.verticalCenter: topbarID.verticalCenter
                    checked: false

                    // iconImageOn: "qrc:/icons/icons-left-checked/light-high-checked.svg"
                    // iconImageOff: "qrc:/icons/icons-left/light_high.svg"
                    iconImageOn: "qrc:/icons/icons-left-checked/high_beam_active.png"
                    iconImageOff: "qrc:/icons/icons-left/high_beam.png"
                }

                //----Hazard----
                FuncIcon {
                    id: hazardID
                    width: 60
                    height: 60
                    opacity: 0.9
                    checked: false
                    anchors.right: logoID.right
                    anchors.rightMargin: -100
                    anchors.verticalCenter: topbarID.verticalCenter

                    iconImageOn: "qrc:/icons/icons-left-checked/hazard_active.png"
                    iconImageOff: "qrc:/icons/icons-left/hazard_low.png"

                    property bool wasLeftBlinkingBeforeHazard: false
                    property bool wasRightBlinkingBeforeHazard: false

                    Timer {
                        id: hazardBlinkTimer
                        interval: 500
                        running: false
                        repeat: true
                        onTriggered: {
                            turnLeftID.checked = !turnLeftID.checked
                            turnRightID.checked = !turnRightID.checked
                        }
                    }
                }
            }

            //----bottom bar----
            Image {
                id: bottomBarID
                source: "qrc:/img/bottom.png"
                width: 960
                height: 110
                anchors.bottom: parent.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottomMargin: 80

                //----Thời gian----
                Text {
                    id: realTimeID
                    anchors.verticalCenter: bottomBarID.verticalCenter
                    anchors.horizontalCenter: bottomBarID.horizontalCenter
                    color: "white"
                    font.pixelSize: 24
                    text: Qt.formatDateTime(new Date(), "hh:mm:ss AP")
                }

                Timer {
                    id: realTimeUpdateID
                    interval: 1000
                    running: true
                    repeat: true
                    onTriggered: {
                        realTimeID.text = Qt.formatDateTime(new Date(),
                                                            "hh:mm:ss AP")
                    }
                }

                FuncIcon {
                    id: mapID
                    width: 50
                    height: 50
                    opacity: 0.9
                    checked: false
                    anchors.verticalCenter: bottomBarID.verticalCenter
                    anchors.left: bottomBarID.left
                    anchors.leftMargin: 100

                    iconImageOn: ""
                    iconImageOff: "qrc:/icons/maps-and-flags.png"
                }

                FuncIcon {
                    id: seatbeltID
                    width: 50
                    height: 50
                    opacity: 0.9
                    checked: false
                    anchors.verticalCenter: bottomBarID.verticalCenter
                    anchors.right: bottomBarID.right
                    anchors.rightMargin: 100

                    iconImageOn: ""
                    iconImageOff: "qrc:/icons/seatbelt.png"
                }

                Rectangle {
                    width: 80
                    height: 80
                    radius: 10
                    color: "transparent"
                    border.color: "#333"
                    border.width: 2
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.right: realTimeID.left
                    anchors.rightMargin: 50

                    Text {
                        anchors.centerIn: parent
                        text: root.currentGear
                        font.pixelSize: 60
                        font.bold: true
                        font.family: "Arial"

                        color: root.currentGear === "D" ? "#00FF00" : (root.currentGear === "R" ? "#FF0000" : "#FFCC00")
                    }
                }

                FuncIcon {
                    id: collisionAlertIcon
                    width: 60
                    height: 60
                    opacity: 0.9
                    checked: false
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.left: realTimeID.right
                    anchors.leftMargin: 50

                    iconImageOn: "qrc:/icons/forward_waning.png"
                    iconImageOff: "qrc:/icons/forward.png"

                    SequentialAnimation {
                        running: collisionAlertIcon.checked
                        loops: Animation.Infinite

                        PropertyAnimation {
                            target: collisionAlertIcon
                            property: "opacity"
                            to: 0.2
                            duration: 150
                        }
                        PropertyAnimation {
                            target: collisionAlertIcon
                            property: "opacity"
                            to: 1.0
                            duration: 150
                        }
                    }

                    onCheckedChanged: {
                        if (!checked)
                            collisionAlertIcon.opacity = 0.9
                    }
                }
            }

            //---detect sign----
            Image {
                id: speedID
                source: "qrc:/icons/Road/ss.svg"
                width: 120
                height: 120
                opacity: 0.9
                anchors.top: topbarID.top
                anchors.topMargin: 135
                anchors.right: background.right
                anchors.rightMargin: 590
                anchors.leftMargin: 295

                Text {
                    id: speedLimitTextDisplay
                    text: root.speedLimitValue
                    font.pixelSize: 40
                    font.bold: true
                    //color: "black"
                    //anchors.centerIn: parent
                    horizontalAlignment: Text.AlignHCenter
                    anchors.top: parent.top
                    anchors.topMargin: 55
                    anchors.horizontalCenter: speedID.horizontalCenter

                    property bool isOverspeed: {
                        // Chuyển biển báo (string) sang số
                        var limit = parseInt(root.speedLimitValue)

                        // Nếu biển báo là "---" hoặc không phải số thì bỏ qua
                        if (isNaN(limit))
                            return false

                        // Nếu biển STOP ("STOP") thì coi như limit = 0 -> xe chạy là cảnh báo
                        if (root.speedLimitValue === "STOP"
                                && root.currentRealSpeed > 0)
                            return true

                        // So sánh: Tốc độ thực > Biển báo => Cảnh báo
                        return root.currentRealSpeed > limit
                    }

                    color: isOverspeed ? "red" : "black"

                    // 3. Hiệu ứng nhấp nháy (Opacity)
                    SequentialAnimation on opacity {
                        running: speedLimitTextDisplay.isOverspeed // Chỉ chạy khi quá tốc độ
                        loops: Animation.Infinite // Lặp vô tận

                        PropertyAnimation {
                            to: 0.2
                            duration: 200
                        } // Mờ đi
                        PropertyAnimation {
                            to: 1.0
                            duration: 200
                        } // Sáng lại
                    }

                    // Khi hết quá tốc độ, reset độ sáng về 1
                    onIsOverspeedChanged: {
                        if (!isOverspeed)
                            opacity = 1.0
                    }
                }

                // Image {
                //     id: speedLimitImage
                //     source: root.currentSignSource
                //     width: 60
                //     height: 60
                //     anchors.top: parent.top
                //     anchors.topMargin: 50
                //     anchors.horizontalCenter: parent.horizontalCenter
                // }
            }

            // --- construction ---
            FuncIcon {
                id: constructID
                width: 60
                height: 60
                opacity: 0.9
                checked: false
                anchors.verticalCenter: topbarID.verticalCenter
                anchors.left: topbarID.left
                anchors.leftMargin: 90

                iconImageOn: "qrc:/icons/construction_warning.png"
                iconImageOff: "qrc:/icons/constuction.png"

                property bool wasLeftBlinkingBeforeHazard: false
                property bool wasRightBlinkingBeforeHazard: false

                Behavior on opacity {
                    NumberAnimation {
                        duration: 300
                    }
                }
            }

            Timer {
                id: hidePopupTimer
                interval: 3000 // 3 giây
                running: false
                repeat: false
                onTriggered: {
                    warningPopup.visible = false
                    constructID.checked = true
                }
            }
            Rectangle {
                id: warningPopup
                width: 500
                height: 200
                anchors.centerIn: parent
                color: "#CC0000"
                radius: 20
                border.color: "yellow"
                border.width: 5
                visible: false
                z: 100

                Column {
                    anchors.centerIn: parent
                    spacing: 5

                    Text {
                        text: "CÔNG TRƯỜNG"
                        color: "yellow"
                        font.bold: true
                        font.pixelSize: 40
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    Text {
                        text: "CHUYỂN LÁI THỦ CÔNG!"
                        color: "white"
                        font.bold: true
                        font.pixelSize: 28
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                SequentialAnimation on opacity {
                    running: warningPopup.visible
                    loops: Animation.Infinite
                    PropertyAnimation {
                        to: 0.5
                        duration: 300
                    }
                    PropertyAnimation {
                        to: 1.0
                        duration: 300
                    }
                }
            }
        }

        StartWindows {
            id: startWindow
            anchors.fill: parent
            active: !root.showMainContent

            onStartClicked: {
                root.showMainContent = true
                console.log("Starting...")
            }

            MouseArea {
                anchors.fill: parent // Phủ kín màn hình Start
                onClicked: {
                    root.showMainContent = true
                    console.log("Starting via Screen Click...")
                }
            }

            // Ẩn content ban đầu
            Component.onCompleted: {
                showMainContent = false
            }
        }

        Connections {
            target: myBackend

            function onDataReceived(message) {
                console.log("NHẬN SIGNAL RAW:", JSON.stringify(message))

                const cleanMsg = message.trim()
                console.log("SAU TRIM:", JSON.stringify(cleanMsg))

                const parts = cleanMsg.split(":")
                if (parts.length !== 2) {
                    console.log("INVALID MESSAGE:", cleanMsg)
                    return
                }

                const device = parts[0]
                const status = parts[1]

                if (device === "AI") {
                    if (status === "0") {
                        if (root.isManualMode === false) {
                            constructID.checked = true
                            warningPopup.visible = true

                            hidePopupTimer.restart()
                            root.isManualMode = true
                            cruise.cruiseStatus = 0
                            cruise.targetSpeed = 0
                            root.speedLimitValue = "---"
                        }
                    } else {
                        if (root.isManualMode === true) {
                            constructID.checked = false
                            warningPopup.visible = false
                            root.isManualMode = false
                        }

                        if (status === "1") {
                            root.speedLimitValue = "40"
                        } else if (status === "2") {
                            root.speedLimitValue = "50"
                        } else if (status === "3") {
                            root.speedLimitValue = "60"
                        } else if (status === "4") {
                            root.speedLimitValue = "80"
                        } else if (status === "5") {
                            root.speedLimitValue = "STOP"
                            cruise.cruiseStatus = 0
                        } else {
                            root.speedLimitValue = ""
                        }
                    }
                } else {
                    switch (device) {
                    case "Car":
                        if (status === "Started!") {
                            root.showMainContent = true
                            console.log("Xe đã khởi động, hiển thị giao diện chính.")
                        } else if (status === "Stopped!") {
                            root.showMainContent = false
                            console.log("Xe đã dừng, hiển thị lại giao diện khởi động.")
                        }
                        break
                    case "TURN_LEFT":
                        if (status === "ON") {
                            if (!turnLeftID.blinking) {
                                console.log("Left turn signal on")
                                turnLeftID.blinking = true
                                turnLeftID.checked = true
                            }
                        } else {
                            if (turnLeftID.blinking) {
                                console.log("Left turn signal off")
                                turnLeftID.blinking = false
                                turnLeftID.checked = false
                            }
                        }
                        break
                    case "TURN_RIGHT":
                        if (status === "ON") {
                            if (!turnRightID.blinking) {
                                console.log("Right turn signal on")
                                turnRightID.blinking = true
                                turnRightID.checked = true
                            }
                        } else {
                            if (turnRightID.blinking) {
                                console.log("Right turn signal off")
                                turnRightID.blinking = false
                                turnRightID.checked = false
                            }
                        }
                        break
                    case "HAZARD":
                        if (status === "ON") {
                            if (!hazardID.checked) {
                                console.log("Đèn hazard bật")
                                hazardID.checked = true
                                turnRightID.blinking = true
                                turnRightID.checked = true
                                turnLeftID.blinking = true
                                turnLeftID.checked = true
                            }
                        } else {
                            if (hazardID.checked) {
                                console.log("Đèn hazard tắt")
                                hazardID.checked = false
                                turnRightID.blinking = false
                                turnRightID.checked = false
                                turnLeftID.blinking = false
                                turnLeftID.checked = false
                            }
                        }
                        break
                    case "DEN_COS":
                        if (status === "ON") {
                            console.log("Đèn COS bật")
                            cosID.checked = true
                        } else {
                            console.log("Đèn COS tắt")
                            cosID.checked = false
                        }
                        break
                    case "DEN_PHA":
                        if (status === "ON") {
                            console.log("Đèn pha bật")
                            phaID.checked = true
                        } else {
                            console.log("Đèn pha tắt")
                            phaID.checked = false
                        }
                        break
                    case "BTN_MANUAL":
                        if (status === "ON") {
                            console.log("Đã nhấn nút cứng chuyển lái thủ công!")
                            root.isManualMode = true
                            warningPopup.visible = false
                            constructID.checked = false
                        }
                        break
                    case "CRUISE":
                        var cStatus = parseInt(status)
                        if (!isNaN(cStatus)) {
                            if (root.isManualMode === true) {
                                cruise.cruiseStatus = 0
                            } else {
                                cruise.cruiseStatus = cStatus
                                console.log("Cruise State Updated:", cStatus)
                            }
                        }
                        break
                    case "SET_SPEED":
                        var setSpd = parseInt(status)
                        if (!isNaN(setSpd)) {
                            cruise.targetSpeed = setSpd
                            console.log("Updated Target Speed:", setSpd)
                        }
                        break
                    case "GAP_LEVEL":
                        var gap = parseInt(status)
                        if (!isNaN(gap)) {
                            // Gọi vào cái ID hiển thị Gap (sẽ tạo ở Bước 3)
                            gapDisplay.currentGap = gap
                        }
                        break
                    case "GEAR":
                        if (status === "3") {
                            root.currentGear = "D"
                        } else if (status === "1") {
                            root.currentGear = "R"
                        } else {
                            root.currentGear = "P"
                        }
                        console.log("Gear changed to:", root.currentGear)
                        break
                    case "REAL_SPEED":
                        root.currentRealSpeed = parseFloat(status)
                        break
                    case "DISTANCE":
                        var dist = parseInt(status)
                        if (!isNaN(dist)) {
                            if (dist > 0 && dist < 30) {
                                collisionAlertIcon.checked = true
                            } else {
                                collisionAlertIcon.checked = false
                            }
                        }
                        break
                    case "POT_VAL":
                        // Chuyển đổi giá trị từ chuỗi thành số nguyên
                        var potValue = parseInt(status)
                        if (isNaN(potValue)) {
                            // Kiểm tra nếu giá trị không phải số
                            console.log("POT_VAL không phải số: " + value)
                            break
                        }

                        var minAngle = 148
                        var maxAngle = 392

                        var mappedAngle = minAngle + (potValue / 4095.0) * (maxAngle - minAngle)
                        speedometerLeft.angle = mappedAngle

                        // speedometerRight biểu diễn RPM
                        var minRpmAngle = 148
                        var maxRpmAngle = 392

                        var normalizedPotValue = potValue / 4095.0
                        var nonLinearProgress = Math.pow(normalizedPotValue,
                                                         1.5)

                        var mappedRpmAngle = minRpmAngle + nonLinearProgress
                                * (maxRpmAngle - minRpmAngle)
                        speedometerRight.angle = mappedRpmAngle
                        break
                    case "DATE/TIME":
                        realTimeID.text = status
                        break
                    default:
                        console.log("Invalid Device:", device)
                        break
                    }
                }
            }
        }
    }
}
