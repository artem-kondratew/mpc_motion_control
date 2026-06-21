#!/usr/bin/env python3
"""odom_tf_broadcaster — публикует Odometry-топик как TF (frame_id -> child_frame).

Зачем: чтобы ВИДЕТЬ лидера на карте ведомого. Зеркало позы лидера
(`/<leader>/lio_sam/mapping/odometry`, frame_id=<leader>/lio_sam_odom)
приходит от v2v_receiver как ТОПИК, без TF. Этот узел broadcast'ит его как TF —
и лидер появляется (и движется) в дереве ведомого, подвешенный через статик-анкер
`<self>/lio_sam_odom -> <leader>/lio_sam_odom` (нода peer_map_anchor_tf в transforms).

child-фрейм: если задан параметр `child_frame` — публикуем ИМЕННО его (напр. голый
`alpha` — vehicle-фрейм лидера); иначе берём child_frame_id из самого сообщения.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):

    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/alpha/lio_sam/mapping/odometry')
        # если задан — публикуем ИМЕННО этот child-фрейм (приоритет над сообщением)
        self.declare_parameter('child_frame', '')

        topic = self.get_parameter('odom_topic').value
        self.child_override = self.get_parameter('child_frame').value

        self.br = TransformBroadcaster(self)
        # зеркало от v2v_receiver — RELIABLE; берём дефолтный reliable
        self.create_subscription(Odometry, topic, self._cb, 10)
        self.get_logger().info(
            f'[odom_tf_broadcaster] {topic} -> TF (frame_id -> child_frame_id)')

    def _cb(self, msg: Odometry) -> None:
        child = self.child_override or msg.child_frame_id   # override приоритетнее
        if not msg.header.frame_id or not child:
            self.get_logger().warn(
                'odom без frame_id/child_frame_id — задай child_frame параметром',
                throttle_duration_sec=5.0)
            return
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = child
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.br.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
