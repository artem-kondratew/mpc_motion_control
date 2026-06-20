#!/usr/bin/env python

import os

from jinja2 import Environment, FileSystemLoader, select_autoescape
from setuptools import setup

# Выходное имя скрипта
script_name = "ricar"

# Локальные пути к файлам, нужным для установки
#  *они не устанавливаются вместе с пакетом, используются только их пути
docker_compose_file = "docker-compose.yaml"
launch_file = "launch.yaml"
requirements_file = "requirements.txt"
predefined_tasks_file = "predefined_tasks.yaml"

current_dir = os.path.dirname(os.path.realpath(__file__))

#
# Получаем зависимости для установки
#
requirements_dir = os.path.join(current_dir, requirements_file)
with open(requirements_dir, "r") as f:
    requirements = f.readlines()
requirements = [r.strip() for r in requirements]

#
# Тут заполняем Jinja2 шаблоны и сохраняем в scripts
#
templates_dir = os.path.join(current_dir, "templates")
env = Environment(
    loader=FileSystemLoader(templates_dir),
    autoescape=select_autoescape(),
)

docker_compose_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
template = env.get_template("ricar.py")

# заполняются шаблоны именно здесь
rendered = template.render(
    docker_compose_dir=docker_compose_dir,
    compose_file=docker_compose_file,
    launch_file=launch_file,
    predefined_tasks_file=predefined_tasks_file,
)

# TODO: устанавливать в build
scripts_dir = os.path.join(current_dir, "scripts")
script_path = os.path.join(scripts_dir, script_name)

if not os.path.exists(scripts_dir):
    os.makedirs(scripts_dir)

with open(script_path, "w") as f:
    f.write(rendered)

#
# Установка
#
setup(
    name="ricar_launch",
    version="1.0",
    license="BSD",
    description="ricar launch module",
    author="Sakharov Dmitry",
    author_email="saharov.dd@starline.ru",
    packages=["ricar_launch"],
    scripts=[script_path],
    install_requires=requirements,
)
