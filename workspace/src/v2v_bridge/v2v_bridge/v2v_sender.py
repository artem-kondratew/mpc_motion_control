#!/usr/bin/env python3
"""v2v_sender — сторона ЛИДЕРА UDP-моста.

Подписывается на топики лидера (`/<leader_id>/<suffix>`), сериализует каждое
сообщение в CDR и шлёт по UDP на `peer_ip:port` ведомому. Каналы (топик/тип/порт/
ограничение частоты) описаны в `channels_file`.

Подписки идут с BEST_EFFORT QoS — совместимо и с reliable-, и с best_effort-
издателями (например, lio_sam публикует odometry как BEST_EFFORT).
"""

import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from rosidl_runtime_py.utilities import get_message

from .udp_common import load_channels, pack_fragments


class V2VSender(Node):

    def __init__(self) -> None:
        super().__init__('v2v_sender')

        self.declare_parameter('channels_file', '')
        self.declare_parameter('leader_id', 'alpha')
        self.declare_parameter('peer_ip', '127.0.0.1')

        channels_file = self.get_parameter('channels_file').value
        self.leader_id = self.get_parameter('leader_id').value
        self.peer_ip = self.get_parameter('peer_ip').value

        channels = load_channels(channels_file)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = {}

        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.get_logger().info(
            f'[v2v_sender] leader_id={self.leader_id} -> peer {self.peer_ip}')
        for cid, ch in enumerate(channels):
            topic = f"/{self.leader_id}/{ch['suffix']}"
            msg_cls = get_message(ch['type'])
            port = int(ch['port'])
            max_rate = float(ch.get('max_rate', 0.0))
            self._seq[cid] = 0
            self._make_sub(cid, topic, msg_cls, be_qos, port, max_rate, ch['name'])
            self.get_logger().info(
                f"  {ch['name']}: {topic} ({ch['type']}) -> udp {self.peer_ip}:{port}"
                f"{'' if max_rate <= 0 else f' @≤{max_rate}Hz'}")

    def _make_sub(self, cid, topic, msg_cls, qos, port, max_rate, name) -> None:
        min_dt = (1.0 / max_rate) if max_rate > 0 else 0.0
        state = {'t': None}

        def cb(msg) -> None:
            now = self.get_clock().now()
            if min_dt > 0 and state['t'] is not None:
                if (now - state['t']).nanoseconds * 1e-9 < min_dt:
                    return
            state['t'] = now
            data = serialize_message(msg)
            seq = self._seq[cid]
            self._seq[cid] = (seq + 1) & 0xffff
            for frag in pack_fragments(cid, seq, data):
                try:
                    self.sock.sendto(frag, (self.peer_ip, port))
                except OSError as e:
                    self.get_logger().warn(
                        f'[v2v_sender] {name} sendto failed: {e}',
                        throttle_duration_sec=2.0)

        self.create_subscription(msg_cls, topic, cb, qos)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V2VSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
