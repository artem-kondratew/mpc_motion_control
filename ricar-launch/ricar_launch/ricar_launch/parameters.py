#!/usr/bin/env python

"""Модуль для получения параметров, использующихся в командах в .yaml"""

import subprocess

from typing import List


__author__ = "Сахаров Дмитрий <saharov.dd@starline.ru>"
__copyright__ = "StarLine LLC, 2024"


def get_images() -> List[str]:
    """Получить образы

    :return список образов
    """
    cmd = "docker"
    args = ["images", "--format", "{{.Repository}}:{{.Tag}}"]

    images = []
    proc = subprocess.Popen([cmd, *args], stdout=subprocess.PIPE)

    # TODO: добавить таймаут
    while True:
        # TODO: Доработать парсер: ubuntu:<none> парсится как ubuntu:\<none\>
        image = proc.stdout.readline().decode()
        if not image:
            break
        images.append(image.strip())

    return images


def get_containers() -> List[str]:
    """Получить контейнеры

    :return список контейнеров
    """
    cmd = "docker"
    args = ["ps", "-a", "--format", "{{.Names}}"]

    containers = []
    proc = subprocess.Popen([cmd, *args], stdout=subprocess.PIPE)

    # TODO: добавить таймаут
    while True:
        container = proc.stdout.readline().decode()
        if not container:
            break
        containers.append(container.strip())

    return containers


def get_services(config: dict) -> List[str]:
    """Получить профили из yaml конфига

    :config конфигурация

    :return список сервисов
    """
    if "services" not in config:
        return []

    services = list(sorted(config["services"].keys()))
    return services


def get_profiles(config: dict) -> List[str]:
    """Получить профили из yaml конфига

    :config конфигурация

    :return список профилей
    """
    if "services" not in config:
        return []

    profiles = []
    for service, service_dict in config["services"].items():
        if "profiles" in service_dict.keys():
            profiles.extend(service_dict["profiles"])

    profiles = list(sorted(set(profiles)))
    return profiles


def get_tasks(config: dict) -> List[str]:
    """Получить задания (команды для исполнения внутри контейнера)
    из yaml конфига

    :config конфигурация

    :return список заданий
    """
    if "services" not in config:
        return []

    tasks = []
    for service, service_dict in config["services"].items():
        if "tasks" in service_dict.keys():
            tasks.extend(service_dict["tasks"])

    tasks = list(sorted(set(tasks)))
    return tasks


def get_parameters(config: dict, parameter: str) -> List[str]:
    """Получить параметры

    :config docker compose конфигурация (yaml)
    :parameter имя параметра

    :return список возможных значений для параметра
    """
    if parameter == "__profile__":
        return get_profiles(config)

    if parameter == "__service__":
        return get_services(config)

    if parameter == "__image__":
        return get_images()

    if parameter == "__container__":
        return get_containers()
    
    if parameter == "__task__":
        return get_tasks(config)

    print("Unknown parameter:", parameter)
    return []
