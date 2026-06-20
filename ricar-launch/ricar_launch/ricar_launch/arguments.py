#!/usr/bin/env python

"""Модуль парсинга аргументов командной строки"""

import argparse
import argcomplete
import re

import command

from typing import List, Union

__author__ = "Сахаров Дмитрий <saharov.dd@starline.ru>"
__copyright__ = "StarLine LLC, 2024"


def register_arguments(commands: List[command.Command]) -> argparse.ArgumentParser:
    """Зарегистрировать параметры

    :commands команды

    :return парсер аргументов
    """
    parser = argparse.ArgumentParser(argument_default=argparse.SUPPRESS)
    subparser = parser.add_subparsers(dest="cmd")

    # регистрируем команды вроде up, start и т.п.
    for cmd in commands:
        p = subparser.add_parser(
            cmd.name, aliases=cmd.aliases, help=cmd.description, add_help=False
        )
        p.set_defaults(cmd=cmd.name)  # разрешаем alias относительно cmd
        for i in range(len(cmd.parameters)):
            if cmd.name == "commit" and cmd.parameters_templates[i] == "__image__":
                arg = p.add_argument(f"placeholder{i}")
                arg.completer = argcomplete.completers.ChoicesCompleter(cmd.parameters[i])
            else:
                p.add_argument(f"placeholder{i}", choices=cmd.parameters[i])
        if cmd.long_command:
            p.add_argument(f"placeholder{len(cmd.parameters)}", nargs=argparse.REMAINDER)

    return parser


def parse_placeholders(args: argparse.Namespace) -> List[Union[int, float, str]]:
    """Распарсить значения placeholders из полученных аргументов

    :args аргументы командной строки

    :return значения для подстановки в placeholders
    """

    # тут ищем значения для параметров вида placeholder0, placeholder1 и т.д.
    placeholders = []
    for i, (placeholder_name, placeholder_value) in enumerate(args.__dict__.items()):
        if placeholder_name == 'cmd' and placeholder_value is None:
            continue
        if len(placeholder_value) == 0:
            empty_placeholder_name = f"__placeholder_{i}__"
            cmd = args.__dict__['cmd']
            print(f'ricar {cmd}: error: empty placeholder: {empty_placeholder_name}')
            exit(1)
        # мы ищем именно placeholders, т.к. в args.__dict__ хранятся еще другие параметры
        if len(re.findall(r"placeholder\d+", placeholder_name)) == 1:
            placeholders.append((placeholder_name, placeholder_value))

    # чтобы быть уверенными, что placeholder0 на индексе 0 и т.д.
    placeholders = sorted(placeholders, key=lambda x: x[0])

    # а после сортировки имена уже не нужны, оставляем только значения
    placeholders = [
        " ".join(v) if isinstance(v, list) else v
        for k, v in placeholders
    ]

    return placeholders
