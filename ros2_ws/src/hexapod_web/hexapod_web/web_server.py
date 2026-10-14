"""Punto de entrada del panel web ROS 2."""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException

from .web_app import create_app
from .web_node import WebPanelNode


def spin_ros(node):
    """Mantiene activo ROS 2 hasta que se cierre su contexto."""
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WebPanelNode()
    app = create_app(node)
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    try:
        app.run(host='0.0.0.0', port=node.web_port)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
