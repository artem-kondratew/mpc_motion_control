#!/usr/bin/env python

"""Вспомогательные функции"""

import os
import yaml

__author__ = "Сахаров Дмитрий <saharov.dd@starline.ru>"
__copyright__ = "StarLine LLC, 2024"


def read_yaml(path: str) -> dict:
    """Прочитать yaml файл

    :path путь к файлу
    """
    with open(path) as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as ex:
            print(ex)

    return {}


def execute(command: str, work_dir: str = "") -> int:
    """Выполнить команду

    :command команда
    :work_dir рабочая директория

    :return код ошибки
    """
    cwd = os.getcwd()
    if work_dir:
        os.chdir(work_dir)

    ret = os.system(command)
    os.chdir(cwd)
    return ret
