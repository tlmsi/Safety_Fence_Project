import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    color: "#202124"

    implicitWidth: 360
    implicitHeight: 430

    function send(command) {
        _SafetyPanel.SendCommand(command)
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12

        Label {
            text: "SORTING CELL SAFETY"
            color: "white"
            font.bold: true
            font.pixelSize: 19
            Layout.alignment: Qt.AlignHCenter
        }

        RowLayout {
            Layout.fillWidth: true

            Button {
                text: "START / RESUME"
                Layout.fillWidth: true
                highlighted: true
                onClicked: send("resume")
            }

            Button {
                text: "PAUSE"
                Layout.fillWidth: true
                onClicked: send("pause")
            }
        }

        Button {
            text: "SAFETY RESET"
            Layout.fillWidth: true
            onClicked: send("reset")
        }

        Item {
            Layout.preferredHeight: 8
        }

        Button {
            text: "EMERGENCY STOP"
            Layout.fillWidth: true
            Layout.preferredHeight: 90

            background: Rectangle {
                radius: 12
                color: "#c51f1f"
                border.color: "#ffdb00"
                border.width: 6
            }

            contentItem: Label {
                text: parent.text
                color: "white"
                font.bold: true
                font.pixelSize: 20
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }

            onClicked: send("emergency_stop")
        }

        Item {
            Layout.preferredHeight: 8
        }

        Label {
            text: "SAFETY GATE"
            color: "white"
            font.bold: true
            Layout.alignment: Qt.AlignHCenter
        }

        RowLayout {
            Layout.fillWidth: true

            Button {
                text: "OPEN GATE"
                Layout.fillWidth: true
                onClicked: send("gate_open")
            }

            Button {
                text: "CLOSE GATE"
                Layout.fillWidth: true
                onClicked: send("gate_close")
            }
        }

        Item {
            Layout.fillHeight: true
        }

        Label {
            text: "Green: Running   Yellow: Pause / Gate   Red: E-Stop"
            color: "#dddddd"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
        }
    }
}
