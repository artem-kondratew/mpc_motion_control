# tvp_docker_image

Репозиторий для сборки docker-образа автопилота

## Структура репозитория

- `Dockerfile` – описание слоев образа
- `docker-compose.yml` – обвязка вокруг образа (net, volumes, env, gpu и т.п.)
- `build.sh` – сборка образа (с режимами пересборки)
- `run.sh` – запуск контейнера на основе образа
- `exec.sh` – вход внутрь запущенного контейнера
- `stop.sh` – остановка и удаление контейнера
- `restart.sh` – перезапуск контейнера

## Структура Dockerfile
- Базовый образ: `ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble`
- Различные переменные окружения
- RU-зеркала для `apt`
- Установка пакетов через `apt`
- Создание директорий
- Граница кеша `ARG REBUILD_PYTHON_PACKAGES=0`
- Установка внешних `python`-зависимостей
- Граница кеша `ARG REBUILD_GITLAB_PACKAGES=0`
- Установка пакетов восприятия
- Установка зависимостей для пакетов навигации
- Установка пакетов навигации, которые редко меняются
- Граница кеша `ARG REBUILD_FREQUENTLY_CHANGING_NAVIGATION_PACKAGES=0`
- Установка пакетов навигации, которые часто меняются
- `colcon build`
- Конфигурация `.bashrc`

## Смысл границ кеша
- `ARG REBUILD_PYTHON_PACKAGES=0`: позволяет взять из кеша установку пакетов через `apt` и забилдить остальное

- `ARG REBUILD_GITLAB_PACKAGES=0`: позволяет помимо `apt` взять из кеша `python`-зависимости. Все `gitlab`-пакеты проекта склонируются заново

- `ARG REBUILD_FREQUENTLY_CHANGING_NAVIGATION_PACKAGES=0`: из кеша возьмется все, кроме пакетов навигации, которые часто меняются

## Режимы пересборки образа
- `./build.sh` – дефолтный режим, использует кеш; если в репозиториях после сборки произошли изменения,
этот режим их не добавит, т.к. просто возьмет слои с репозиториями из кеша

- `./build.sh debug` – тот же режим, что и дефолтный, но с расширенным выводом информации на экран (`--progress=plain`)

- `./build.sh no-cache` – полная пересборка образа без использования кеша

- `./build.sh rebuild-python` – пересборка слоев начиная с установки `python`-зависимостей; слой с установкой пакетов через `apt` будет взят из кеша

- `./build.sh rebuild-git` – пересборка только слоев с репозиториями; `apt` и `python` будут взяты из кеша

- `./build.sh rebuild-freq` – пересборка только слоя с часто изменяющимися пакетами навигации и слоя с `colcon build`; остальное берется из кеша

## Требования к окружению, в котором производится сборка
- Наличие переменной окружения `VEHICLE_ID`. Проверка:

    `echo $VEHICLE_ID`

    Ожидаемый вывод:

    `alpha` (или любое другое существующее имя ровера)

## Регистрация python-пакетов для добавления в Dockerfile

Необходимо:

- Добавить пакет в [список](https://192.168.31.177:6748/robolab/navigation/tvp_configs/-/blob/develop/alpha/python3/pip/local_packages.txt?ref_type=heads),
чтобы `uv` смог отделить внешние зависимости от внутренних

- Добавить пакет в структуры [файла](https://192.168.31.177:6748/robolab/navigation/tvp_configs/-/blob/develop/alpha/python3/pip/pyproject.toml?ref_type=heads):
    ```
    [project]
    name = "tvp-python-stack"
    version = "0.1.0"
    dependencies = [
        "setuptools==58.2.0",
        "astral-ab3dmot-utils",
        ...
        "<new_package>"
    ]

    [tool.uv.sources]
    astral-ab3dmot-utils = { git = "ssh://git@192.168.31.177/perception/astral_ab3dmot_utils.git", branch = "develop" }
    ...
    <new_package>        = { git = "<repo_link>", branch = "<branch_name>" }
    ```

- Добавить пакет в один из слоев `Dockerfile`:
    ```
    RUN --mount=type=ssh \
        ssh-keyscan -H 192.168.31.177 >> /root/.ssh/known_hosts 2>/dev/null || true && \
        git clone -b develop git@192.168.31.177:robolab/navigation/tvp_launch.git ${WORKDIR}/src/tvp_launch && \
        ...
        git clone -b develop git@192.168.31.177:robolab/navigation/ros2parquet_msgs.git ${WORKDIR}/src/ros2parquet_msgs && \
        git clone -b <branch_name> "<repo_link>" ${WORKDIR}/src/<repo_name>
    ```
