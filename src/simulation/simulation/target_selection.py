#!/usr/bin/env python3

import sys
import signal
import pathlib
import threading
import logging

import rclpy
from rclpy.node import Node

from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix

from PySide2 import QtCore, QtGui, QtQml
from PySide2.QtPositioning import QGeoCoordinate
from PySide2.QtCore import Property, QObject, Signal, Slot


def get_coord(lat, lon):
    pos = QGeoCoordinate()
    pos.setLatitude(lat)
    pos.setLongitude(lon)
    return pos


class Backend(QObject):
    exitRequested = Signal()
    valueChanged = Signal()
    locationChanged = Signal()
    targetLocationChanged = Signal()
    historyChanged = Signal()

    def __init__(self, ros_node: Node, parent=None):
        super().__init__(parent)
        self.node = ros_node
        self._should_run = True
        self._robot_location = get_coord(32.072734, 34.787465)
        self._target_location = get_coord(32.072734, 34.787465)
        self._robot_route = []
        self.max_tractor_hist = 1000

        self._sub_location = self.node.create_subscription(
            NavSatFix,
            "/robot_location",
            self.cb_location,
            10
        )

        self._pub_target = self.node.create_publisher(
            GeoPoint,
            "/target_location",
            10
        )

        self._thread = threading.Thread(target=self.main_loop)
        self._thread.start()

    @Slot(float, float)
    def set_target_point(self, lat, lon):
        self._target_location = get_coord(lat, lon)
        msg = GeoPoint()
        msg.latitude = lat
        msg.longitude = lon
        self._pub_target.publish(msg)
        self.targetLocationChanged.emit()

    @Property(QGeoCoordinate, constant=False, notify=locationChanged)
    def robot_location(self):
        return self._robot_location

    @Property(QGeoCoordinate, constant=False, notify=targetLocationChanged)
    def target_location(self):
        return self._target_location

    def cb_location(self, msg: NavSatFix):
        self._robot_location = get_coord(msg.latitude, msg.longitude)
        self.locationChanged.emit()

    def append_location_to_hist(self):
        self._robot_route.append(self._robot_location)
        if len(self._robot_route) > self.max_tractor_hist:
            del self._robot_route[0]
        self.historyChanged.emit()

    @Slot()
    def clearHistory(self):
        self._robot_route.clear()

    @Slot()
    def exit(self):
        self._should_run = False
        self._thread.join()

    def main_loop(self):
        rate = self.node.create_rate(1)
        while rclpy.ok() and self._should_run:
            self.append_location_to_hist()
            rate.sleep()

    @QtCore.Property(list, constant=False, notify=historyChanged)
    def location_history(self):
        logging.warning("Location history requested")
        return self._robot_route


def main():
    rclpy.init()
    node = rclpy.create_node("sim_gui")
    
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QtGui.QGuiApplication(sys.argv)
    app.setApplicationVersion("0.1")
    
    # OPTIONAL: set general names for settings (or skip entirely if not using settings)
    app.setOrganizationName("SimNavigation")
    app.setOrganizationDomain("example.org")  # You can remove this if not needed

    
    
    import os
    os.environ["QT_SSL_USE_TEMPORARY_KEYCHAIN"] = "1"

    backend = Backend(ros_node=node, parent=app)

    engine = QtQml.QQmlApplicationEngine()
    engine.rootContext().setContextProperty("backend", backend)

    current_dir = pathlib.Path(__file__).resolve().parent
    engine.load(QtCore.QUrl(str(current_dir / "view.qml")))

    if not engine.rootObjects():
        sys.exit(-1)

    # Keeps GUI responsive
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    def sigint_handler(sig, frame):
        print(f"\tSIGINT: exiting (signal={sig}, frame={frame})", flush=True)
        backend.exitRequested.emit()

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)

    result = app.exec_()
    backend.exit()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
