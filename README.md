# mpc_motion_control — ветка hackathon2026

Контроллеры MPC для симулятора [hackathon2026](https://github.com/just-robotics/hackathon2026):
круиз-контроль (CC), адаптивный круиз-контроль (ACC) и удержание в полосе.

Эта ветка — срез ветки `main` под симулятор Gazebo. Оставлены только пакеты,
нужные чтобы поехать в симе; всё, что относится к реальному железу
(LIO-SAM, драйверы Livox и Kobuki, v2v-мост, обвязка `ricar`), удалено.
Основная разработка идёт в `main`.

## Состав

| Каталог | Назначение |
|---|---|
| `workspace/src/swarm_controller/` | контроллеры CC / ACC / lateral, MPC-ядра в `submodules/` |
| `workspace/src/swarm_msgs/` | `Telemetry.msg` для ACC |

## Подключение к hackathon2026

Пакеты подключаются подмодулем — код не копируется:

```bash
git submodule add -b hackathon2026 \
    git@github.com:artem-kondratew/mpc_motion_control.git src/mpc_motion_control
```

В корне репозитория лежит `COLCON_IGNORE`, поэтому автообход colcon подмодуль
пропускает. Собирать нужно явными путями:

```bash
colcon build --base-paths \
    src/mpc_motion_control/workspace/src/swarm_msgs \
    src/mpc_motion_control/workspace/src/swarm_controller
```

В образ хакатона нужно доставить решатель QP — его там нет:

```dockerfile
RUN python3 -m pip install --no-cache-dir osqp scipy
```

## Запуск

```bash
ros2 launch swarm_controller sim_control.launch.py
```

| Аргумент | По умолчанию | Значения |
|---|---|---|
| `lateral` | `true` | `true` — удержание в полосе поверх продольного контура |
| `longitudinal` | `cc` | `cc` — свой целевой профиль скорости, `acc` — зазор за лидером |
| `trajectory` | `circle` | `line`, `circle`, `lanelet` |
| `trajectory_file` | `my_trajectory5.yaml` | waypoints для `lanelet` |
| `odom_topic` | `/odom` | одометрия симулятора |
| `cmd_vel_topic` | `/cmd_vel` | команда скорости в симулятор |
| `frame_id` | `odom` | фрейм опорной траектории |
| `peer_id` | `leader` | префикс топиков лидера для `acc` |

## Отличия от main

Симулятор публикует топики без префикса робота, а одометрия Gazebo точная,
поэтому лидарная локализация не нужна:

| | main (реальный робот) | эта ветка (симулятор) |
|---|---|---|
| Одометрия | `/<vehicle_id>/odom` | `/odom` |
| Команда | `/<vehicle_id>/cmd_vel` | `/cmd_vel` |
| Поза | LIO-SAM, `/<id>/lio_sam/mapping/odometry` | `/odom` |
| Фрейм траектории | `<id>/lio_sam_odom` | `odom` |
| Профили запуска | `vehicle`, `sensing`, `localization`, `transforms`, `control` | только `control` |

Параметры MPC (`config/*.param.yaml`) снимались на реальной Kobuki — в
симуляторе динамика другая, их нужно перетюнить.

## Тюнинг

| Что | Файл |
|---|---|
| Круиз-контроль | `swarm_controller/config/cc_mpc.param.yaml` |
| Адаптивный круиз-контроль | `swarm_controller/config/acc_mpc.param.yaml` |
| Удержание в полосе | `swarm_controller/config/lat_mpc.param.yaml`, `lane.param.yaml` |

## Лицензия

MIT, см. [LICENSE](LICENSE).
