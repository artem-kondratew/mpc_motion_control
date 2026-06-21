"""Общие хелперы UDP-моста: фрагментация датаграмм и загрузка конфига каналов.

Транспорт: ROS-сообщение сериализуется в CDR (`serialize_message`), бьётся на
датаграммы с маленьким бинарным заголовком и шлётся по UDP. На приёме —
пересборка по seq и `deserialize_message`. Заголовок (network byte order):

    magic(2s) | channel_id(B) | seq(H) | frag_cnt(B) | frag_idx(B)

Одна датаграмма = заголовок + кусок payload. Большинство сообщений (Odometry,
Twist) влезают в один фрагмент; Path при большом числе точек бьётся на несколько.
Path статична после анкеринга, поэтому потеря датаграммы некритична — придёт
следующая публикация.
"""

import os
import struct

import yaml

MAGIC = b'V2'
HEADER_FMT = '!2sBHBB'                       # magic, channel_id, seq, frag_cnt, frag_idx
HEADER_SIZE = struct.calcsize(HEADER_FMT)    # 7 байт
MAX_PAYLOAD = 60000                          # payload на фрагмент (датаграмма < 64 КБ)


def pack_fragments(channel_id: int, seq: int, data: bytes) -> list:
    """Разбить сериализованное сообщение на UDP-датаграммы с заголовком."""
    n = max(1, (len(data) + MAX_PAYLOAD - 1) // MAX_PAYLOAD)
    if n > 255:
        raise ValueError(f'message too large to fragment ({len(data)} bytes, >255 frags)')
    out = []
    for i in range(n):
        chunk = data[i * MAX_PAYLOAD:(i + 1) * MAX_PAYLOAD]
        header = struct.pack(HEADER_FMT, MAGIC, channel_id & 0xff,
                             seq & 0xffff, n, i)
        out.append(header + chunk)
    return out


class Reassembler:
    """Пересборка фрагментов одного канала (одного UDP-порта).

    Держит только последний seq: новый seq сбрасывает недособранный пакет
    (старый теряем — не блокируемся на дырках).
    """

    def __init__(self) -> None:
        self._seq = None
        self._cnt = 0
        self._frags = {}

    def feed(self, datagram: bytes):
        """Скормить датаграмму. Возвращает собранные байты или None."""
        if len(datagram) < HEADER_SIZE:
            return None
        magic, _ch, seq, cnt, idx = struct.unpack(HEADER_FMT, datagram[:HEADER_SIZE])
        if magic != MAGIC:
            return None
        payload = datagram[HEADER_SIZE:]
        if cnt == 1:
            return payload
        if self._seq != seq:
            self._seq, self._cnt, self._frags = seq, cnt, {}
        self._frags[idx] = payload
        if len(self._frags) == cnt:
            data = b''.join(self._frags[i] for i in range(cnt))
            self._frags = {}
            return data
        return None


def load_channels(path: str) -> list:
    """Загрузить список каналов из yaml (относительный путь -> share/v2v_bridge/config)."""
    if not path:
        path = 'channels.yaml'
    if not os.path.isabs(path):
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory('v2v_bridge'), 'config', path)
    with open(path) as f:
        data = yaml.safe_load(f)
    return list(data['channels'])
