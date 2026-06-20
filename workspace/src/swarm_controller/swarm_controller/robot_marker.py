#!/usr/bin/env python3
"""Простая «фигурка» робота (kobuki + лидар MID360) маркерами для foxglove/rviz.

Публикует MarkerArray, привязанный к base-фрейму робота (`frame_id`, по умолч.
base_footprint). frame_locked=True -> едет вместе с TF фрейма. Мешей/URDF не нужно.

Параметры:
  frame_id  — фрейм привязки (напр. "alpha/base_footprint")
  topic     — имя топика маркеров (относительное; под namespace -> /<id>/...)
  rate      — частота переиздания, Гц (для поздних подписчиков)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class RobotMarker(Node):

    def __init__(self) -> None:
        super().__init__('robot_marker')
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value
        self.topic = self.declare_parameter('topic', 'robot_marker').value
        rate = float(self.declare_parameter('rate', 2.0).value)

        self.pub = self.create_publisher(MarkerArray, self.topic, 1)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'[robot_marker] frame={self.frame_id} -> topic {self.topic}')

    def _m(self, mid: int, mtype: int) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = 'robot'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.frame_locked = True
        m.pose.orientation.w = 1.0
        m.color.a = 1.0
        return m

    def _tick(self) -> None:
        arr = MarkerArray()

        # корпус kobuki — диск Ø0.351, высота 0.09
        body = self._m(0, Marker.CYLINDER)
        body.scale.x = body.scale.y = 0.351
        body.scale.z = 0.09
        body.pose.position.z = 0.045
        body.color.r, body.color.g, body.color.b = 0.25, 0.25, 0.28
        arr.markers.append(body)

        # «бампер»/перед — тонкий брусок спереди для наглядности направления
        bump = self._m(1, Marker.CUBE)
        bump.scale.x, bump.scale.y, bump.scale.z = 0.04, 0.30, 0.07
        bump.pose.position.x = 0.16
        bump.pose.position.z = 0.05
        bump.color.r, bump.color.g, bump.color.b = 0.85, 0.65, 0.10
        arr.markers.append(bump)

        # лидар MID360 — цилиндр сверху (z=0.20, как mount base_link->lidar_70)
        lidar = self._m(2, Marker.CYLINDER)
        lidar.scale.x = lidar.scale.y = 0.065
        lidar.scale.z = 0.06
        lidar.pose.position.z = 0.20
        lidar.color.r, lidar.color.g, lidar.color.b = 0.05, 0.05, 0.05
        arr.markers.append(lidar)

        # стрелка направления (+x)
        arrow = self._m(3, Marker.ARROW)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.28, 0.05, 0.05
        arrow.pose.position.z = 0.11
        arrow.color.r, arrow.color.g, arrow.color.b = 0.10, 0.50, 1.0
        arr.markers.append(arrow)

        self.pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
