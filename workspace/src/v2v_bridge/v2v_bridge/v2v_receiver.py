#!/usr/bin/env python3
"""v2v_receiver — сторона ВЕДОМОГО UDP-моста.

На каждый канал биндит UDP-порт, пересобирает фрагменты, десериализует CDR
обратно в ROS-сообщение и переиздаёт его зеркалом в свой граф под именем
`/<leader_id>/<suffix>` — теми же топиками/фреймами, что были у лидера.

Сокеты неблокирующие; таймер дренирует их все на каждом тике. Издатели —
RELIABLE (совместимо с любыми подписчиками вниз по потоку).
"""

import socket

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

from rosidl_runtime_py.utilities import get_message

from .udp_common import Reassembler, load_channels


class V2VReceiver(Node):

    def __init__(self) -> None:
        super().__init__('v2v_receiver')

        self.declare_parameter('channels_file', '')
        self.declare_parameter('leader_id', 'alpha')
        self.declare_parameter('bind_addr', '0.0.0.0')
        self.declare_parameter('poll_period', 0.005)

        channels_file = self.get_parameter('channels_file').value
        self.leader_id = self.get_parameter('leader_id').value
        bind_addr = self.get_parameter('bind_addr').value

        channels = load_channels(channels_file)
        self._socks = []

        self.get_logger().info(f'[v2v_receiver] leader_id={self.leader_id}')
        for ch in channels:
            topic = f"/{self.leader_id}/{ch['suffix']}"
            msg_cls = get_message(ch['type'])
            port = int(ch['port'])
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((bind_addr, port))
            sock.setblocking(False)
            pub = self.create_publisher(msg_cls, topic, 10)
            self._socks.append({
                'sock': sock, 'pub': pub, 'cls': msg_cls,
                'asm': Reassembler(), 'name': ch['name'],
            })
            self.get_logger().info(
                f"  {ch['name']}: udp :{port} -> {topic} ({ch['type']})")

        self.create_timer(
            float(self.get_parameter('poll_period').value), self._drain)

    def _drain(self) -> None:
        for s in self._socks:
            sock = s['sock']
            while True:
                try:
                    data, _ = sock.recvfrom(65535)
                except (BlockingIOError, OSError):
                    break
                full = s['asm'].feed(data)
                if full is None:
                    continue
                try:
                    msg = deserialize_message(full, s['cls'])
                    s['pub'].publish(msg)
                except Exception as e:        # noqa: BLE001 — мусор в сети не должен ронять ноду
                    self.get_logger().warn(
                        f"[v2v_receiver] {s['name']} deserialize failed: {e}",
                        throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V2VReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
