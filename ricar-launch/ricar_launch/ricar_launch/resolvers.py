#!/usr/bin/env python

"""Резолверы значений placeholder перед подстановкой в shell-команду.

Резолвер -- функция (str) -> str, которая преобразует введенное пользователем
значение в нужное для команды. Например, имя сервиса -> id контейнера.
"""

import subprocess


def get_container_id_by_profile(service: str) -> str:
    """По имени сервиса найти id контейнера через docker compose

    Берем и запущенные, и остановленные контейнеры (-a), т.к. docker умеет
    коммитить остановленные, а ricar down оставляет контейнер как Exited.

    :service имя сервиса

    :return id контейнера (при отсутствии -- завершаем с ошибкой)
    """
    proc = subprocess.run(
        ["docker", "compose", "ps", "-aq", service],
        stdout=subprocess.PIPE,
    )
    container_id = proc.stdout.decode().split("\n")[0].strip()

    if not container_id:
        print(f"No container for service: {service}")
        exit(1)

    return container_id
