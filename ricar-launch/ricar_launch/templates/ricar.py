#!/usr/bin/env python

"""Основной модуль запуска стека"""

import argcomplete
import os
import sys

# Это для импортов, чтобы либы импортились во всех случаях:
# - локальный запуск скрипта
# - глобальный (установленный через pip) запуск скрипта
if os.path.exists(os.path.abspath("../ricar_launch/__init__.py")):
    sys.path.append("../ricar_launch")
else:
    try:
        import ricar_launch

        sys.path.append(os.path.dirname(ricar_launch.__file__))
    except ImportError:
        print("Module ricar_launch not found!")
        exit(1)

import arguments
import command
import misc


__author__ = "Сахаров Дмитрий <saharov.dd@starline.ru>"
__copyright__ = "StarLine LLC, 2024"

# PYTHON_ARGCOMPLETE_OK

# Jinja2 параметры, заполняются при установке через setup.py
DOCKER_COMPOSE_DIR = "{{ docker_compose_dir }}"
COMPOSE_FILE = "{{ compose_file }}"
LAUNCH_FILE = "{{ launch_file }}"
PREDEFINED_TASKS_FILE = "{{ predefined_tasks_file }}"


def main():
    os.chdir(DOCKER_COMPOSE_DIR)  # для выполнения compose команд
    compose_config = misc.read_yaml(COMPOSE_FILE)
    run_config = misc.read_yaml(LAUNCH_FILE)
    predef_config = misc.read_yaml(PREDEFINED_TASKS_FILE)

    commands : list= command.parse_commands(compose_config, run_config)
    predef_tasks = command.parse_predefined_tasks(compose_config, predef_config)
    commands.extend(predef_tasks)

    parser = arguments.register_arguments(commands)
    argcomplete.autocomplete(parser)

    args = parser.parse_args()
    placeholders = arguments.parse_placeholders(args)

    if not args.cmd:
        parser.print_help()
        return 1

    for cmd in commands:
        if args.cmd == cmd.name:
            cmd.execute(*placeholders)

    return 0


if __name__ == "__main__":
    exit(main())
