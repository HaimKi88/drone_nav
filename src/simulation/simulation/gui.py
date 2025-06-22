#!/usr/bin/env python3
import sys
import pathlib
import threading
import signal

import rclpy
from rclpy.node import Node

from geographic_msgs.msg import GeoPoint

from PySide6 import QtCore, QtGui, QtQml
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtCore import Property, QObject, Signal, Slot


class ROS2Backend(Node, QObject):
    coordinateClicked = Signal(float, float)  # lat, lon

    def __init__(self, parent=None):
        Node.__init__(self, "map_gui_node")
        QObject.__init__(self, parent)

        self._target_location = QGeoCoordinate(32.072734, 34.787465)

        self._pub_target = self.create_publisher(GeoPoint, "/target_location", 10)

    @Slot(float, float)
    def onMapClicked(self, lat, lon):
        print(f"Map clicked at lat={lat}, lon={lon}")
        self._target_location = QGeoCoordinate(lat, lon)

        msg = GeoPoint()
        msg.latitude = lat
        msg.longitude = lon
        self._pub_target.publish(msg)
        self.coordinateClicked.emit(lat, lon)

    @Property(QGeoCoordinate)
    def target_location(self):
        return self._target_location


def main():
    rclpy.init()
    app = QtGui.QGuiApplication(sys.argv)

    backend = ROS2Backend(parent=app)

    engine = QtQml.QQmlApplicationEngine()
    engine.rootContext().setContextProperty("backend", backend)

    qml_file = pathlib.Path(__file__).parent / "view.qml"
    engine.load(str(qml_file))

    if not engine.rootObjects():
        sys.exit(-1)

    # Setup signal handler
    def sigint_handler(sig, frame):
        print("SIGINT received, shutting down.")
        app.quit()

    signal.signal(signal.SIGINT, sigint_handler)

    ret = app.exec()
    backend.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
