import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root

    color: "#11161c"

    implicitWidth: 390
    implicitHeight: 610

    property string safetyState: "UNKNOWN"
    property bool gateOpen: false
    property string operatorWarning: ""

    function send(command) {
        _SafetyPanel.SendCommand(command)
    }

    function stateText() {
        if (safetyState === "RUNNING")
            return "RUNNING"

        if (safetyState === "MANUAL_PAUSE")
            return "MANUAL PAUSE"

        if (safetyState === "PROTECTIVE_STOP")
            return "PROTECTIVE STOP"

        if (safetyState === "E_STOP")
            return "EMERGENCY STOP"

        return "WAITING FOR SAFETY SUPERVISOR"
    }

    function stateColor() {
        if (safetyState === "RUNNING")
            return "#27ae60"

        if (safetyState === "MANUAL_PAUSE")
            return "#f2b134"

        if (safetyState === "PROTECTIVE_STOP")
            return "#f39c12"

        if (safetyState === "E_STOP")
            return "#e53935"

        return "#66717e"
    }

    function showWarning(message) {
        operatorWarning = message
        warningTimer.restart()
    }

    function requestResume() {
        if (gateOpen) {
            showWarning(
                "YOU NEED TO CLOSE THE DOOR FIRST."
            )
            return
        }

        if (
            safetyState === "PROTECTIVE_STOP"
            || safetyState === "E_STOP"
        ) {
            showWarning(
                "YOU NEED TO RESET FIRST."
            )
            return
        }

        operatorWarning = ""
        send("resume")
    }

    function requestReset() {
        if (gateOpen) {
            showWarning(
                "YOU NEED TO CLOSE THE DOOR FIRST."
            )
            return
        }

        operatorWarning = ""
        send("reset")
    }

    Timer {
        id: stateRefresh

        interval: 100
        running: true
        repeat: true

        onTriggered: {
            root.safetyState =
                _SafetyPanel.SafetyState()

            root.gateOpen =
                _SafetyPanel.GateOpen()
        }
    }

    Timer {
        id: warningTimer

        interval: 5000
        repeat: false

        onTriggered: {
            root.operatorWarning = ""
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 10

        // ----------------------------------------------------
        // HEADER
        // ----------------------------------------------------

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 58

            radius: 8
            color: "#1b232d"
            border.color: "#303b47"
            border.width: 1

            Column {
                anchors.centerIn: parent
                spacing: 2

                Label {
                    anchors.horizontalCenter:
                        parent.horizontalCenter

                    text: "SORTING CELL"
                    color: "#f3f6f8"

                    font.bold: true
                    font.pixelSize: 20
                    font.letterSpacing: 1.3
                }

                Label {
                    anchors.horizontalCenter:
                        parent.horizontalCenter

                    text: "SAFETY CONTROL PANEL"
                    color: "#8fa0b3"

                    font.pixelSize: 11
                    font.letterSpacing: 1.0
                }
            }
        }


        // ----------------------------------------------------
        // LIVE MACHINE STATUS
        // ----------------------------------------------------

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 84

            radius: 8
            color: "#181f27"
            border.color: root.stateColor()
            border.width: 2

            RowLayout {
                anchors.fill: parent
                anchors.margins: 12
                spacing: 12

                Rectangle {
                    Layout.preferredWidth: 18
                    Layout.preferredHeight: 18

                    radius: 9
                    color: root.stateColor()

                    border.color: "#e5e7eb"
                    border.width: 1
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 2

                    Label {
                        text: "SYSTEM STATE"
                        color: "#8fa0b3"

                        font.pixelSize: 10
                        font.bold: true
                    }

                    Label {
                        text: root.stateText()
                        color: root.stateColor()

                        font.pixelSize: 18
                        font.bold: true
                    }

                    Label {
                        text: root.gateOpen
                            ? "Safety door: OPEN"
                            : "Safety door: CLOSED"

                        color: root.gateOpen
                            ? "#ff6b6b"
                            : "#9bd6ae"

                        font.pixelSize: 12
                        font.bold: true
                    }
                }
            }
        }


        // ----------------------------------------------------
        // MOTION CONTROL
        // ----------------------------------------------------

        Label {
            text: "MOTION CONTROL"
            color: "#8493a5"

            font.pixelSize: 11
            font.bold: true
            font.letterSpacing: 1.0

            Layout.leftMargin: 2
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Button {
                id: resumeButton

                text: "START / RESUME"

                Layout.fillWidth: true
                Layout.preferredHeight: 48

                background: Rectangle {
                    radius: 6

                    color: resumeButton.down
                        ? "#145a32"
                        : resumeButton.hovered
                            ? "#27864b"
                            : "#1f7440"

                    border.color: "#56c77a"
                    border.width: 1
                }

                contentItem: Label {
                    text: resumeButton.text

                    color: "white"
                    font.bold: true
                    font.pixelSize: 13

                    horizontalAlignment:
                        Text.AlignHCenter

                    verticalAlignment:
                        Text.AlignVCenter
                }

                onClicked: root.requestResume()
            }

            Button {
                id: pauseButton

                text: "PAUSE"

                enabled:
                    root.safetyState === "RUNNING"

                Layout.fillWidth: true
                Layout.preferredHeight: 48

                background: Rectangle {
                    radius: 6

                    color: !pauseButton.enabled
                        ? "#353a40"
                        : pauseButton.down
                            ? "#8a6209"
                            : pauseButton.hovered
                                ? "#c38b12"
                                : "#a9780d"

                    border.color: pauseButton.enabled
                        ? "#e5b642"
                        : "#555b62"

                    border.width: 1
                }

                contentItem: Label {
                    text: pauseButton.text

                    color: pauseButton.enabled
                        ? "white"
                        : "#777d84"

                    font.bold: true
                    font.pixelSize: 13

                    horizontalAlignment:
                        Text.AlignHCenter

                    verticalAlignment:
                        Text.AlignVCenter
                }

                onClicked: {
                    root.operatorWarning = ""
                    root.send("pause")
                }
            }
        }

        Button {
            id: resetButton

            text: "SAFETY RESET"

            Layout.fillWidth: true
            Layout.preferredHeight: 44

            background: Rectangle {
                radius: 6

                color: resetButton.down
                    ? "#334155"
                    : resetButton.hovered
                        ? "#52657b"
                        : "#405166"

                border.color: "#8295aa"
                border.width: 1
            }

            contentItem: Label {
                text: resetButton.text

                color: "#f4f6f8"
                font.bold: true
                font.pixelSize: 13

                horizontalAlignment:
                    Text.AlignHCenter

                verticalAlignment:
                    Text.AlignVCenter
            }

            onClicked: root.requestReset()
        }


        // ----------------------------------------------------
        // OPERATOR WARNING
        // ----------------------------------------------------

        Rectangle {
            Layout.fillWidth: true

            Layout.preferredHeight:
                root.operatorWarning === ""
                    ? 0
                    : 48

            visible:
                root.operatorWarning !== ""

            radius: 6
            color: "#31191b"

            border.color: "#ef5350"
            border.width: 2

            Label {
                anchors.fill: parent
                anchors.margins: 8

                text: root.operatorWarning
                color: "#ff5252"

                font.bold: true
                font.pixelSize: 13

                horizontalAlignment:
                    Text.AlignHCenter

                verticalAlignment:
                    Text.AlignVCenter

                wrapMode:
                    Text.WordWrap
            }
        }


        // ----------------------------------------------------
        // EMERGENCY STOP
        // ----------------------------------------------------

        Button {
            id: emergencyButton

            text: "EMERGENCY STOP"

            Layout.fillWidth: true
            Layout.preferredHeight: 86

            background: Rectangle {
                radius: 10

                color: emergencyButton.down
                    ? "#8f1515"
                    : emergencyButton.hovered
                        ? "#d42b2b"
                        : "#b91f1f"

                border.color: "#ffd43b"
                border.width: 4
            }

            contentItem: Column {
                anchors.centerIn: parent
                spacing: 2

                Label {
                    anchors.horizontalCenter:
                        parent.horizontalCenter

                    text: "EMERGENCY STOP"
                    color: "white"

                    font.bold: true
                    font.pixelSize: 19
                }

                Label {
                    anchors.horizontalCenter:
                        parent.horizontalCenter

                    text: "PRESS TO STOP MOTION"
                    color: "#ffdcdc"

                    font.bold: true
                    font.pixelSize: 10
                }
            }

            onClicked: {
                root.operatorWarning = ""
                root.send("emergency_stop")
            }
        }


        // ----------------------------------------------------
        // SAFETY DOOR
        // ----------------------------------------------------

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 110

            radius: 8
            color: "#181f27"

            border.color: root.gateOpen
                ? "#e55c5c"
                : "#384553"

            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 7

                RowLayout {
                    Layout.fillWidth: true

                    Label {
                        text: "SAFETY DOOR"

                        color: "#dce3ea"
                        font.bold: true
                        font.pixelSize: 12
                    }

                    Item {
                        Layout.fillWidth: true
                    }

                    Label {
                        text: root.gateOpen
                            ? "OPEN"
                            : "CLOSED"

                        color: root.gateOpen
                            ? "#ff6262"
                            : "#69cf8b"

                        font.bold: true
                        font.pixelSize: 12
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8

                    Button {
                        id: openGateButton

                        text: "OPEN DOOR"
                        enabled: !root.gateOpen

                        Layout.fillWidth: true
                        Layout.preferredHeight: 42

                        background: Rectangle {
                            radius: 5

                            color:
                                openGateButton.enabled
                                ? "#4b3524"
                                : "#30363c"

                            border.color:
                                openGateButton.enabled
                                ? "#c58a53"
                                : "#474d53"
                        }

                        contentItem: Label {
                            text: openGateButton.text

                            color:
                                openGateButton.enabled
                                ? "#f2d0ad"
                                : "#6e747a"

                            font.bold: true

                            horizontalAlignment:
                                Text.AlignHCenter

                            verticalAlignment:
                                Text.AlignVCenter
                        }

                        onClicked: {
                            root.operatorWarning = ""
                            root.send("gate_open")
                        }
                    }

                    Button {
                        id: closeGateButton

                        text: "CLOSE DOOR"
                        enabled: root.gateOpen

                        Layout.fillWidth: true
                        Layout.preferredHeight: 42

                        background: Rectangle {
                            radius: 5

                            color:
                                closeGateButton.enabled
                                ? "#234d35"
                                : "#30363c"

                            border.color:
                                closeGateButton.enabled
                                ? "#63b781"
                                : "#474d53"
                        }

                        contentItem: Label {
                            text: closeGateButton.text

                            color:
                                closeGateButton.enabled
                                ? "#c6efd3"
                                : "#6e747a"

                            font.bold: true

                            horizontalAlignment:
                                Text.AlignHCenter

                            verticalAlignment:
                                Text.AlignVCenter
                        }

                        onClicked: {
                            root.operatorWarning = ""
                            root.send("gate_close")
                        }
                    }
                }
            }
        }


        Item {
            Layout.fillHeight: true
        }


        // ----------------------------------------------------
        // STATUS LEGEND
        // ----------------------------------------------------

        Label {
            Layout.fillWidth: true

            text:
                "GREEN  Running    •    YELLOW  Pause / Protective Stop    •    RED  E-Stop"

            color: "#788696"
            font.pixelSize: 9

            horizontalAlignment:
                Text.AlignHCenter

            wrapMode:
                Text.WordWrap
        }
    }
}
