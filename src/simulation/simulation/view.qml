import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.14
import QtQuick.Window 2.15
import QtPositioning 5.15
import QtLocation 5.15

Window {
    id: mainWindow
    visible: true
    width: 800
    height: 600
    title: qsTr("SimGUI")

    property bool map_follow_vehicle: false
    property var location_history_proxy: backend.location_history

    function centerMapOn(point) {
        cute_map.center = point;
        console.log("Centering map on: " + point)
    }

    Component.onCompleted: {
        console.log("QML LOADED")
        backend.exitRequested.connect(mainWindow.close)
    }

    Plugin {
        id: mapPlugin
        name: "osm"  // ✅ Changed from "mapboxgl" to "osm"
    }

    Map {
        id: cute_map
        anchors.fill: parent
        plugin: mapPlugin
        center: QtPositioning.coordinate(32.072734, 34.787465)
        zoomLevel: 17.0
        maximumZoomLevel: 17.9
        activeMapType: supportedMapTypes[0]

        function toggleMap() {
            if (activeMapType == supportedMapTypes[0] && supportedMapTypes.length > 1) {
                activeMapType = supportedMapTypes[1]
            } else {
                activeMapType = supportedMapTypes[0]
            }
        }

        MapQuickItem {
            coordinate: backend.target_location
            anchorPoint: Qt.point(sourceItem.width / 2, sourceItem.height / 2)
            sourceItem: Rectangle {
                width: 30
                height: 30
                radius: 15
                color: 'green'
                opacity: 0.5
                border.color: "black"
                border.width: 2
            }
        }

        MapItemView {
            model: location_history_proxy
            delegate: MapQuickItem {
                coordinate: modelData
                anchorPoint: Qt.point(sourceItem.width / 2, sourceItem.height / 2)
                sourceItem: Rectangle {
                    width: 10
                    height: 10
                    radius: 5
                    color: 'red'
                    opacity: 1.0
                }
            }
        }

        MapQuickItem {
            coordinate: backend.robot_location
            anchorPoint: Qt.point(sourceItem.width / 2, sourceItem.height / 2)
            sourceItem: Rectangle {
                width: 30
                height: 30
                radius: 15
                color: 'red'
                opacity: 1.0
                border.color: "black"
                border.width: 2
            }
        }

        MouseArea {
            id: mapMouseArea
            anchors.fill: parent
            hoverEnabled: true
            acceptedButtons: Qt.LeftButton | Qt.RightButton

            onClicked: {
                var coord = cute_map.toCoordinate(Qt.point(mouse.x, mouse.y));
                if (mouse.button === Qt.LeftButton) {
                    backend.set_target_point(coord.latitude, coord.longitude)
                } else if (mouse.button === Qt.RightButton) {
                    contextMenu.popup()
                }
            }

            Menu {
                id: contextMenu

                Action {
                    text: "Center on Robot"
                    onTriggered: centerMapOn(backend.robot_location)
                }

                Action {
                    text: "Send Robot Here"
                    onTriggered: {
                        var coord = cute_map.toCoordinate(Qt.point(mouse.x, mouse.y));
                        backend.set_target_point(coord.latitude, coord.longitude)
                    }
                }

                Action {
                    text: "Clear History"
                    onTriggered: backend.clearHistory()
                }

                Action {
                    text: "Toggle Map Type"
                    onTriggered: cute_map.toggleMap()
                }
            }
        }
    }
}
