# mpc_motion_control

MPC-алгоритмы управления продольным и поперечным движением мобильных роботов:
круиз-контроль (CC), адаптивный круиз-контроль (ACC) и удержание в полосе.
Реализация на ROS 2 Humble для diff-drive платформы с лидарной локализацией.

Алгоритмы сформулированы в магистерской диссертации «MPC-алгоритмы управления
продольным и поперечным движением мобильных роботов» (НТУ «Сириус», 2026) и
проверены сначала в симуляторе CARLA, а затем на реальной платформе — код в
этом репозитории соответствует второму этапу.

## Возможности

- **Круиз-контроль** — удержание целевой скорости, MPC по линеаризованной модели
  продольной динамики (`swarm_cc_mpc_node`).
- **Адаптивный круиз-контроль** — удержание дистанции до впереди идущего робота
  по политике постоянного временного интервала (CTH), с интегральным контуром по
  зазору и ограничением скорости по кривизне пути (`swarm_acc_mpc_node`).
- **Удержание в полосе** — поперечное управление на LTV-велосипедной модели, плюс
  геометрические контроллеры Pure Pursuit и Stanley для сравнения
  (`swarm_lat_mpc_node`).
- **Управление в скользящем режиме** — альтернативный продольный контур
  (`swarm_sliding_mode_node`).

Задача квадратичного программирования решается через OSQP, продольная и поперечная
динамика разведены на два независимых контура.

## Состав

| Каталог | Назначение |
|---|---|
| `workspace/src/swarm_controller/` | контроллеры CC / ACC / lateral / sliding-mode, MPC-ядра в `submodules/` |
| `workspace/src/swarm_msgs/` | сообщения телеметрии между роботами |
| `workspace/src/v2v_bridge/` | UDP-мост телеметрии между роботами мимо DDS |
| `workspace/src/lio_sam/` | лидарно-инерциальная одометрия и построение карты |
| `workspace/src/livox_drivers/`, `livox_ros_driver2/`, `livox_sdk/` | драйвер лидара Livox MID360 |
| `workspace/src/kobuki_*/` | драйвер базы TurtleBot2 (Kobuki) |
| `workspace/src/tvp_launch/` | точки запуска стека, `launch/components/tvp_core_*.launch.xml` |
| `workspace/src/autoware_integration_tools/` | оценка скорости по лидарной одометрии, мост в Foxglove |
| `tvp_docker_image/` | Dockerfile и сборка образа |
| `ricar-launch/` | запуск стека: docker compose, профили, CLI `ricar` |
| `analysis/` | обработка логов экспериментов |

## Платформа

Стек разворачивается в Docker поверх образа Autoware Universe (ROS 2 Humble),
транспорт — CycloneDDS.

| Компонент | Значение |
|---|---|
| База | TurtleBot2 (Kobuki), USB-serial |
| Лидар | Livox MID360, Ethernet |
| Локализация | LIO-SAM |
| Решатель QP | OSQP |

## Сборка

Требуются Docker с BuildKit и docker compose. Идентификатор робота задаётся
переменной `VEHICLE_ID` — она разводит топики, TF-фреймы и имена нод, чтобы
несколько роботов работали в одной сети без коллизий.

```bash
export VEHICLE_ID=alpha
cd tvp_docker_image
./build.sh
```

Режимы: `./build.sh debug` — подробный вывод, `./build.sh no-cache` — полная
пересборка.

## Запуск

Стек разбит на профили, каждый — отдельный контейнер. Управление через CLI `ricar`
(устанавливается из `ricar-launch/ricar_launch/`):

```bash
ricar start vehicle        # драйвер базы
ricar start sensing        # лидар
ricar start localization   # LIO-SAM
ricar start transforms     # TF-дерево и URDF
ricar start control        # контроллеры
ricar start all            # весь стек
```

Диагностика: `ricar ps`, `ricar log <профиль>`, `ricar enter debug`.
Остановка: `ricar stop <профиль>`, пересоздание контейнера: `ricar clean <профиль>`.

Режим работы задаётся в `ricar-launch/.env`:

| Переменная | Значение |
|---|---|
| `VEHICLE_ID` | идентификатор робота, префикс топиков и TF-фреймов |
| `LONGITUDINAL` | `cc` — свой целевой профиль скорости, `acc` — следование за лидером |
| `LATERAL` | `true` — удержание в полосе, `false` — только продольное управление |
| `TRAJECTORY` | форма опорной траектории: `line`, `circle`, `lanelet` |
| `LEADER_ID` | идентификатор ведущего робота в колонне |

## Работа с кодом

`workspace/src` монтируется в контейнер как volume, colcon собран с
`--symlink-install`. Поэтому правки конфигов, launch-файлов и Python-нод
существующих пакетов применяются без пересборки образа — достаточно
`ricar clean <профиль> && ricar start <профиль>`.

Пересборка образа нужна при изменении C++ кода, добавлении нового пакета,
новых сообщений или правке Dockerfile.

## Namespacing

Топики, имена нод и TF-фреймы разводятся единым префиксом `VEHICLE_ID`. Это два
независимых механизма: `push-ros-namespace` для топиков и нод, и `frame_prefix`
для TF — потому что `/tf` и `/tf_static` в ROS 2 глобальные и namespace их не
затрагивает.

Базовые имена фреймов лежат в конфигах без префикса, он подставляется в рантайме.
Одно и то же базовое имя лидарного фрейма используется в трёх местах —
`livox_drivers/config/rewrite_frames.yaml`, `lio_sam/config/params.yaml`
(`lidarFrame`) и статическом преобразовании в `tvp_core_transforms.launch.xml`;
они должны совпадать.

## Тюнинг

| Что | Где |
|---|---|
| Круиз-контроль | `swarm_controller/config/cc_mpc.param.yaml` |
| Адаптивный круиз-контроль | `swarm_controller/config/acc_mpc.param.yaml` |
| Удержание в полосе | `swarm_controller/config/lat_mpc.param.yaml`, `lane.param.yaml` |
| LIO-SAM | `lio_sam/config/params.yaml` |
| Лидар | `livox_drivers/config/` |
| Установка лидара на базе | `tvp_launch/launch/components/tvp_core_transforms.launch.xml` |

## Связанные репозитории

- [`swarm_cruise_control`](https://github.com/artem-kondratew/swarm_cruise_control) — ранняя версия круиз-контроля для другой платформы.
- [`av_trajectory_planning`](https://github.com/artem-kondratew/av_trajectory_planning) — планирование траектории для платформы с рулевым управлением по Аккерману.

## Лицензия

MIT, см. [LICENSE](LICENSE).
