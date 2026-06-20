# TurtleBot2 + Livox + LIO-SAM на стеке TVP/RICAR

Handoff-документ. Этот ноут (`omegabot`) стоит **на роботе**. Агент на другом ПК
правит код/конфиги здесь по SSH, чинит ошибки и тюнит параметры. Ниже — всё,
что нужно: структура, сборка, запуск, поток данных, TF, тюнинг, типовые ошибки.

> Это адаптация автоварной платформы **TVP** (репо `just-robotics/robot`, ветка
> `turtlebot2`) под **TurtleBot2 (Kobuki) + Livox MID360 + LIO-SAM**, транспорт
> CycloneDDS. Автомобильные части autoware (planning/perception/lanelet2/CUDA/
> внутренний gitlab `192.168.31.177`) вырезаны.

---

## 1. Железо и сеть

| Что | Значение |
|---|---|
| База | Kobuki (TurtleBot2), USB-serial на `/dev/ttyUSB0` |
| Лидар | Livox **MID360**, Ethernet |
| Livox IP | **`10.10.10.70`** |
| Host (этот ноут) IP в лидарной сети | **`10.10.10.103`** |
| `VEHICLE_ID` (env на хосте, в `~/.bashrc`) | **`alpha`** |

Проверка сети лидара: `ip a | grep 10.10.10` (на NIC должен быть `10.10.10.103/24`),
`ping 10.10.10.70`.

---

## ★ Namespacing под 2 робота (КЛЮЧЕВАЯ модель — прочитать первым)

Два робота работают в **одной сети** (общий CycloneDDS, `network_mode: host`). Чтобы
не было коллизий топиков/TF/имён нод, всё разведено **единым префиксом = `VEHICLE_ID`**.
У этого ноута `VEHICLE_ID=alpha` → всё под `/alpha/...`. На втором ноуте задаётся
свой (напр. `export VEHICLE_ID=robot2`) → `/robot2/...`. DDS-домены НЕ делим — роботы
должны видеть друг друга (ACC, этап 2).

**Два независимых механизма (оба от одного `VEHICLE_ID`):**
1. **Топики + имена нод** — `push-ros-namespace $(var vehicle_id)` в каждом `tvp_core_*`.
   Относительные имена становятся `/<id>/...`. Примеры: `/alpha/cmd_vel`, `/alpha/odom`,
   `/alpha/sensing/livox/points`, `/alpha/lio_sam/mapping/odometry`, `/alpha/control/v_ref`.
2. **TF-фреймы** — префикс `<id>/` на КАЖДЫЙ фрейм: `alpha/base_link`, `alpha/lidar_70`,
   `alpha/lio_sam_odom` и т.д. Это ОТДЕЛЬНЫЙ механизм, потому что **`/tf` и `/tf_static`
   в ROS2 глобальные** (tf2_ros публикует абсолютно, namespace их НЕ трогает). Изоляцию
   TF между роботами даёт именно префикс фреймов, а не namespace.

**Как префикс «течёт» по стеку:**
```
ricar (env $VEHICLE_ID) ─► tvp_core_*.launch.xml (arg vehicle_id)
   ├─ push-ros-namespace $(var vehicle_id)                    # топики+ноды
   └─ frame_prefix="$(var vehicle_id)/" ─► дочерние launch:   # TF-фреймы
        • lio_sam/run.launch.py     → ros-param framePrefix     (C++ utility.hpp префиксует 5 фреймов)
        • kobuki.launch.py          → override odom_frame/base_frame
        • livox/all_lidars.launch.py→ ros-param frame_prefix    (C++ rewrite_frames префиксует lidar_70/imu_70)
        • robot_state_publisher     → param frame_prefix          (URDF-фреймы)
        • transforms-статики        → args с $(var vehicle_id)/
        • twist_estimator           → frame_prefix + topic_prefix
   control: кросс-компонентные топики заданы абсолютно — /$(var vehicle_id)/{odom,cmd_vel,lio_sam/...}
```

**Что это значит для агента-отладчика:**
- Все команды `ros2 topic echo / param set / topic hz` — **с префиксом**:
  `ros2 param set /alpha/control/swarm_cc_mpc_node start true`,
  `ros2 topic echo /alpha/lio_sam/mapping/odometry`. (См. §10.)
- Базовые имена фреймов лежат в конфигах **без префикса** (`lidar_70`, `base_footprint`,
  `lio_sam_odom`...). Префикс добавляется в рантайме. Переименовываешь фрейм → меняешь
  базовое имя в конфиге, `<id>/` подставится сам. ВАЖНО: одно и то же базовое имя
  (`lidar_70`) фигурирует в трёх местах — livox `rewrite_frames.yaml`, lio_sam
  `lidarFrame`, статик-mount в `transforms`; они должны совпадать (берут общий префикс).
- **Изменён C++** (нужен ребилд, см. §5): `lio_sam` (utility.hpp: `framePrefix`,
  `lidarVizFrame`; mapOptmization.cpp), `livox_drivers` (rewrite_frames.cpp: `frame_prefix`).
- Логи (`logging`-профиль) и ноутбук (`analysis/turtlebot_analysis.ipynb`, перем. `NS`)
  уже под `/<id>/`. `/tf`, `/tf_static`, `/human_triggers` — глобальные (без префикса).

---

## 2. Структура репозитория (`~/robot`)

```
~/robot/
├── tvp_docker_image/          # СБОРКА образа
│   ├── Dockerfile             # база autoware:universe-devel-humble (без CUDA)
│   ├── docker-compose.yml     # обвязка образа; context: .. (корень репо) для COPY
│   ├── build.sh               # сборка (режимы: normal/debug/no-cache/rebuild-*)
│   ├── run.sh / stop.sh / exec.sh / restart.sh
│   └── check_repos.sh
├── ricar-launch/              # ЗАПУСК (CLI `ricar`)
│   ├── docker-compose.yaml    # профили (vehicle/sensing/localization/control/...)
│   ├── common.yaml            # базовый сервис `app` (env, volumes, net)
│   ├── init-compose.yaml      # init-сервисы (ros-daemon, cleanup, wait-init)
│   ├── .env                   # IMAGE, CMD, LOGGING_*
│   ├── launch.yaml            # описание команд ricar + алиасы
│   └── ricar_launch/          # python-пакет CLI (pip install)
├── workspace/src/             # ROS2-пакеты (СМОНТИРОВАН как volume в контейнер -> /autoware/src)
│   ├── kobuki_core/           # драйвер базы (C++) — НЕ трогать без нужды
│   ├── kobuki_ros/            # kobuki_node (cmd_vel_wrapper, kobuki_description, ...)
│   ├── kobuki_ros_interfaces/
│   ├── livox_sdk/ livox_ros_driver2/   # драйвер лидара (системный SDK ставится в Dockerfile)
│   ├── livox_drivers/         # all_lidars.launch.py + config_mid360_all.json + rewrite_frames
│   ├── lio_sam/               # SLAM/локализация
│   ├── tvp_launch/            # ТОЧКИ ЗАПУСКА: launch/components/tvp_core_*.launch.xml
│   ├── autoware_integration_tools/  # twist_estimator (скорость по лидару), foxglove_visualizer
│   ├── swarm_controller/      # КОНТРОЛЛЕРЫ: CC/ACC/lat MPC + sliding-mode
│   ├── swarm_msgs/            # Telemetry.msg (для ACC/lat, этапы 2-3)
│   └── robot/ robot_msgs/ teleop/ caterwil_sensor_kit_launch/  # легаси/вспомогательное
├── README.md                  # этот файл
└── .dockerignore              # контекст = корень репо
```

**Поток компонентов:** `ricar up <profile>` → контейнер на образе `tvp_image:latest`
→ `ros2 launch tvp_launch tvp_core_<profile>.launch.xml` → пакеты из `workspace/src`.

---

## 3. Сборка образа

```bash
export VEHICLE_ID=alpha          # ОБЯЗАТЕЛЬНО (именно export; build.sh — дочерний процесс)
export COMPOSE_BAKE=false        # чтобы был классический вывод сборки (не bake)
export BUILDKIT_PROGRESS=tty     # красивый прогресс вместо plain

cd ~/robot/tvp_docker_image
./build.sh                       # обычная сборка (с кешем)
# режимы:
#   ./build.sh debug       — verbose (--progress=plain)
#   ./build.sh no-cache    — полная пересборка
#   ./build.sh rebuild-*   — частичные (см. tvp_docker_image/README.md)
```
Результат — образ `tvp_image:latest`. Базовый образ большой (autoware), первая
сборка долгая.

**Что делает Dockerfile (кратко):** ставит apt-пакеты (cyclonedds, ecl-* и
diagnostic-updater для kobuki, xacro, foxglove-bridge...), `gtsam` (для lio_sam,
через borglab PPA), Livox-SDK2, pip-зависимости (`transitions osqp scipy`),
`COPY workspace/src /autoware/src`, `colcon build`.

---

## 4. Запуск (CLI `ricar`)

Установка CLI (один раз): `sudo pip install ~/robot/ricar-launch/ricar_launch/.`
Работать из каталога `~/robot/ricar-launch`.

| Команда | Что делает |
|---|---|
| `ricar up <profile>` | поднять профиль (алиас `start`/`run`/`launch`) |
| `ricar down <profile>` | убить контейнеры профиля (SIGKILL, не удаляет; алиас `stop`/`kill`) |
| `ricar clean <profile>` | down + **удалить** контейнеры (нужно после правок, чтобы подхватился новый образ/.env) |
| `ricar restart <profile>` | kill + up без пересоздания (старый образ!) |
| `ricar ps` | статусы |
| `ricar logs <profile>` | логи (алиас `log`); `ricar flogs` — follow |
| `ricar enter <service>` | bash внутри контейнера |
| `ricar exec <service> <cmd>` | выполнить cmd (уже сорсит `/autoware/install/setup.bash`) |
| `ricar commit <service> <image>` | сохранить состояние контейнера в образ |

**Активные профили** (`ricar-launch/docker-compose.yaml`): `vehicle`, `sensing`,
`localization`, `control`, `tools`, `transforms`, `debug`. Остальные autoware-
профили (map/perception/planning/api/...) **закомментированы** (нет пакетов).

**Типовой запуск (вся езда + локализация):**
```bash
ricar up vehicle transforms localization sensing
ricar up control          # cruise control (этап 1)
ricar up tools            # foxglove (порт 8765) + twist_estimator
```

> ⚠️ Не запускай `ricar up all` бездумно — туда теоретически попадут autoware-
> профили; держим только перечисленные.

---

## 5. ★ Live-правки vs ПЕРЕСБОРКА (критично для агента)

`workspace/src` смонтирован volume'ом в контейнер (`tvp_docker_image/docker-compose.yml`).
colcon собран с `--symlink-install`. Поэтому:

- **Правки yaml/launch/params/json** в `workspace/src/**` — **живые без ребилда**.
  Достаточно пересоздать контейнер:
  ```bash
  ricar clean <profile> && ricar up <profile>
  ```
  (`clean`, не `restart` — restart не пересоздаёт контейнер.)

- **РЕБИЛД образа (`./build.sh`) нужен**, если:
  - добавлен **новый пакет** в `workspace/src` (его нет в `install/`);
  - изменён **C++** код (kobuki_core, lio_sam, livox) → нужен colcon build;
  - изменён **Dockerfile** (apt/pip-зависимости, gtsam, ...);
  - новые **msg/srv** (напр. swarm_msgs) — нужны сгенерированные биндинги.
  - Быстрая альтернатива ребилду: `ricar enter debug` →
    `colcon build --packages-select <pkg>` → `ricar commit debug tvp_image:latest`
    → `ricar clean <profile> && ricar up <profile>`.

Правило: **Python-нода уже существующего пакета и любые конфиги — live; новый
пакет / C++ / Dockerfile / msg — ребилд.**

---

## 6. Поток данных и топики

Все топики под namespace робота `/<id>/` (`<id>` = `VEHICLE_ID`, пример: `alpha`):
```
teleop / CC ─► /<id>/cmd_vel ─► cmd_vel_wrapper ─► /<id>/commands/velocity ─► kobuki ─► колёса
kobuki ─► /<id>/odom (nav_msgs/Odometry, ~50 Гц, twist.linear.x=энкодеры, angular.z=гиро)

Livox .70 ─► livox_ros_driver2 ─► rewrite_frames ─► /<id>/sensing/livox/points (frame <id>/lidar_70)
                                                     /<id>/sensing/livox/imu    (frame <id>/imu_70)
        └─► lio_sam ─► /<id>/lio_sam/mapping/odometry (поза лидара) + TF
                    └─► twist_estimator ─► /<id>/sensing/vehicle_velocity_converter/twist[_with_covariance]
```

Профили и что запускают (`tvp_launch/launch/components/tvp_core_*.launch.xml`):
| Профиль | Запускает |
|---|---|
| `vehicle` | kobuki: `kobuki_node/kobuki.launch.py` (kobuki_ros_node + cmd_vel_wrapper). Нужен `/dev/ttyUSB0` |
| `sensing` | Livox: `livox_drivers/all_lidars.launch.py` (драйвер MID360 .70 + rewrite_frames) |
| `localization` | `lio_sam/run.launch.py` (4 ноды lio_sam; rviz off; `rviz:=true` чтобы включить) |
| `transforms` | robot_state_publisher (kobuki URDF, `frame_prefix`) + статик `<id>/base_link→<id>/lidar_70` + статик `<id>/lio_sam_odom→<id>/odom` |
| `control` | CC + удержание в полосе (по умолч. `lateral:=true`): `swarm_cc_mpc_node` → `/<id>/control/long_cmd`, `lane_publisher` → `/<id>/control/pacemaker/path`, `swarm_lat_mpc_node` → `/<id>/cmd_vel`. `lateral:=false` = только CC прямо в `/<id>/cmd_vel` |
| `tools` | foxglove_bridge (порт 8765) + twist_estimator |
| `debug` | `sleep infinity` (для входа/ручной работы) |

Каждый профиль обёрнут в `push-ros-namespace $(var vehicle_id)` и получает
`frame_prefix=$(var vehicle_id)/` (см. раздел Namespacing выше).

---

## 7. TF-дерево

Все фреймы префиксованы `<id>/` (`<id>` = `VEHICLE_ID`, пример: `alpha`):
```
<id>/lio_sam_map                          (lio_sam)
└─ <id>/lio_sam_odom                      (lio_sam SLAM)
     ├─ <id>/base_footprint               (lio_sam: lio_sam_odom→base_footprint)
     │    └─ <id>/base_link               (kobuki URDF: base_footprint→base_link)
     │         ├─ <id>/lidar_70           (статик-mount base_link→lidar_70, z=0.2 ЗАГЛУШКА)
     │         └─ <id>/wheel_*/caster_*/cliff_*/gyro_link   (kobuki URDF, frame_prefix)
     ├─ <id>/odom                         (статик identity ← привязка kobuki)
     │    └─ <id>/base_link_kobuki        (kobuki колёсная odom: odom→base_link_kobuki)
     └─ <id>/lidar_link                   (сырая поза скан-матчинга lio_sam mapOpt; ros-param lidarVizFrame)
```

Принципы (важно при правках, чтобы не словить «tf loop»):
- **kobuki** публикует `odom → base_link_kobuki` (`base_frame: base_link_kobuki`).
- **lio_sam** — `lio_sam_odom → base_footprint` (`baselinkFrame: base_footprint`).
- `base_footprint` — корень kobuki URDF; URDF владеет `base_footprint→base_link→*`.
- kobuki НЕ должен публиковать свои статики `base_link↔base_footprint` (это делает URDF) —
  иначе цикл. (Эти статики уже убраны из `kobuki.launch.py`.)
- Колёсная одометрия kobuki и lio_sam **НЕ сфьюжены** — висят параллельно под общим
  `lio_sam_odom` (через статик identity). Настоящий фьюзинг = `robot_localization` (позже).

Снять дерево: `ros2 run tf2_tools view_frames` (генерит `frames.pdf`).

---

## 8. ★ Тюнинг параметров (по компонентам)

Все конфиги ниже — **live через volume** (правишь → `ricar clean <profile> && ricar up <profile>`).

### Лидар Livox — `workspace/src/livox_drivers/config/`
- `config_mid360_all.json` — **IP**: host `10.10.10.103` (`host_net_info`, 4 поля),
  лидар `10.10.10.70` (`lidar_configs[0].ip`). При смене лидара/хоста — править здесь.
- `rewrite_frames.yaml` — входные топики драйвера (`livox/lidar_10_10_10_70`,
  `livox/imu_10_10_10_70`), выходные **относительные** (`livox/points`, `livox/imu` →
  под namespace `/<id>/sensing/livox/...`), базовые имена фреймов (`lidar_70`, `imu_70` —
  префикс `<id>/` добавляет нода через ros-param `frame_prefix`). При смене IP — синхронно с json.

### LIO-SAM — `workspace/src/lio_sam/config/params.yaml`
- **`imuGravity: 1.0`** ← КРИТИЧНО. MID360 отдаёт accel **в g** (на покое z≈1.0, не 9.8).
  Если поставить 9.81 — lio_sam расходится, позу «швыряет».
- Топики **относительные** (резолвятся под namespace робота):
  `pointCloudTopic: sensing/livox/points`, `imuTopic: sensing/livox/imu`.
- Базовые фреймы (без префикса; `<id>/` добавляет ros-param `framePrefix` из launch):
  `lidarFrame: lidar_70`, `baselinkFrame: base_footprint`,
  `odometryFrame: lio_sam_odom`, `mapFrame: lio_sam_map` (+ C++ `lidarVizFrame: lidar_link`).
- `extrinsicRot`/`extrinsicRPY` = identity (IMU соосен лидару, гравитация на z).
  `extrinsicTrans` = lidar→imu офсет MID360.
- Качество карты/одометрии: `edgeThreshold/surfThreshold`, `*LeafSize` (voxel),
  `surroundingKeyframe*`. `N_SCAN=1`, `Horizon_SCAN`, `sensor: livox`.
- IMU-шумы: `imuAccNoise/imuGyrNoise/...`.

### Cruise Control — `workspace/src/swarm_controller/config/cc_mpc.param.yaml`
> Имена нод/топиков под namespace: `/<id>/control/...` (пример `<id>=alpha`).
- `v_ref` — целевая скорость (m/s). Можно менять на лету:
  `ros2 param set /alpha/control/swarm_cc_mpc_node v_ref 0.2`
- `tau` — лаг отклика скорости kobuki (модель). **Тюнить по step-response**: скачок
  v_cmd 0→0.3, записать v(t) из `/alpha/odom`, фитнуть первый порядок. Если робот перелетает/
  недотягивает v_ref — обычно дело в tau.
- MPC: `p` (горизонт предсказания), `c` (горизонт управления), `s` (подавление рывка —
  больше = глаже), `q_vals=[q_v, q_a, q_j]`, `phi_vals` (сглаживание reference).
- Лимиты: `a_min/a_max`, `v_cmd_min/v_cmd_max`.
- `start: false` — kill-switch. Запуск езды:
  `ros2 param set /alpha/control/swarm_cc_mpc_node start true`
- Кросс-компонентные топики (`odom_topic`, `cmd_vel_topic`, у lateral также `pose_topic`/
  `pacemaker_path_topic`) задаются **из launch** абсолютно с `/$(var vehicle_id)/`
  (`tvp_core_control.launch.xml`) — значения в yaml это лишь дефолты для bare-запуска.

### Lateral (удержание в полосе) — `swarm_controller/config/{lat_mpc,lane}.param.yaml`
- `lat_mpc.param.yaml`: `q_vals=[q_e, q_eθ, q_α, q_int]` (есть интегратор от offset на круге),
  `e_int_limit` (anti-windup), `tau_w`, лимиты `alpha_*`/`w_cmd_*`.
- `lane.param.yaml`: `trajectory` (straight|circle|lanelet), `circle_radius`, `trajectory_file`.
  `frame_id`/`path_topic`/`pose_topic` оверрайдятся из launch с префиксом — в yaml дефолты.

### Kobuki — `workspace/src/kobuki_ros/kobuki_node/config/kobuki_node_params.yaml`
- `device_port: /dev/ttyUSB0`, `base_frame: base_link_kobuki`, `odom_frame: odom`,
  `use_imu_heading: true`, лимиты батареи. **`base_frame` не менять на `base_link`** —
  будет конфликт с lio_sam. Префикс `<id>/` к `odom_frame`/`base_frame` добавляет
  `kobuki.launch.py` (override из `frame_prefix`); cmd_vel_wrapper подписан на
  относительный `cmd_vel` → `commands/velocity` (резолв под `/<id>/`).

### Mount лидара — `workspace/src/tvp_launch/launch/components/tvp_core_transforms.launch.xml`
- Статик `base_link → lidar_70` сейчас **заглушка z=0.2**. Вписать РЕАЛЬНЫЙ офсет
  установки MID360 (xyz + rpy) — иначе плечо в twist_estimator и привязка облака неверны.

### twist_estimator — `workspace/src/autoware_integration_tools/config/twist_estimator.params.yaml`
- Базовые фреймы `odometry_frame: lidar_70` (lio_sam отдаёт позу лидара), `base_link_frame: base_link`
  (префикс `<id>/` добавляет launch через `frame_prefix`; кросс-топики — через `topic_prefix=/<id>`),
  фильтр `filter_cutoff_frequency`, ковариации.

---

## 9. ★ Типовые ошибки и фиксы (по реальной истории)

| Симптом | Причина → фикс |
|---|---|
| `ERROR: VEHICLE_ID is not set` при сборке | переменная не экспортирована → `export VEHICLE_ID=alpha` (именно export) |
| Сборка докера plain/«странный вывод», строки `load local bake definitions` | включён Compose Bake → `export COMPOSE_BAKE=false` и `export BUILDKIT_PROGRESS=tty` |
| `kobuki_core ... core_sensors.hpp: No such file` | файлы `core_sensors.*` терялись из-за `core*` в .gitignore. Уже восстановлены из `~/hsl24/kobuki`. Если повторится — взять оттуда `include/kobuki_core/packets/core_sensors.hpp` и `src/driver/core_sensors.cpp` |
| lio_sam: позу/`lidar_link` **дико швыряет** на стоящем роботе | accel IMU в **g** → в `params.yaml` `imuGravity: 1.0`. Проверка: `ricar exec sensing ros2 topic echo /alpha/sensing/livox/imu --once` → `linear_acceleration.z` должно быть ≈1.0 (не 9.8) |
| `The tf tree is invalid because it contains a loop` (`base_link↔base_footprint`) | дублирующие статики kobuki vs URDF. Уже убраны из `kobuki.launch.py`. После правки `ricar clean vehicle && ricar up vehicle` |
| lio_sam: `Waiting for IMU data ...` | нет потока с лидара → сеть. `ping 10.10.10.70`, `ip a` (host=.103), `ricar exec sensing ros2 topic hz /alpha/sensing/livox/imu` |
| `Waiting for IMU data`, при этом **точки идут, а IMU пуст** | MID360 после реконнекта поднял только point-stream, IMU-push залип (драйвер пишет `successfully enable Livox Lidar imu`, `tcpdump -ni any udp port 56401` показывает пакеты ~200/с, но ROS-топик `/alpha/sensing/livox/imu_10_10_10_70` пуст). **Фикс: передёрнуть питание лидара**, затем `ricar clean sensing && ricar up sensing`. Диагностика, что это именно lidar, а не код: `sudo tcpdump -ni any udp port 56401` (IMU) vs `56301` (точки) |
| `rviz2 process has died exit code -6` | headless без дисплея. rviz в lio_sam off по умолчанию. Для дисплея: `ros2 launch lio_sam run.launch.py rviz:=true` |
| `ModuleNotFoundError: transitions` (foxglove_visualizer) | pip-зависимость → стоит в Dockerfile (`transitions osqp scipy`). Нужен ребилд если пропала |
| `tools` падает на `autoware_control_performance_analysis`/`robot_configs` | автоварная нода, нет в образе. Уже закомментирована в `tvp_core_tools.launch.xml` |
| Контроллер/нода swarm не находится | новый пакет → нужен **ребилд** (см. §5), не просто clean |
| Колёса не едут от teleop/CC | фокус не на окне teleop; ИЛИ CC и teleop оба пишут `/<id>/cmd_vel` — не запускать вместе; ИЛИ нет `start true` у CC; ИЛИ путаница namespace (проверь `ros2 topic list` — всё под `/<id>/`) |
| Угловая скорость в `/<id>/odom` ≈0 при вращении «в воздухе» | НЕ баг: kobuki берёт angular.z из **гироскопа**, который меряет поворот корпуса (в воздухе корпус не вертится). На полу будет |
| `tf` лукапы lio_sam/twist_estimator не находят фрейм (`Could not find transform <id>/...`) | рассинхрон префиксов: базовое имя фрейма различается в livox `rewrite_frames.yaml` / lio_sam `lidarFrame` / статик-mount, либо запущен не весь набор профилей. `/tf` глобальный — проверь `ros2 run tf2_tools view_frames`, все фреймы должны быть `<id>/...` |

Общий приём отладки из README ricar: поднять `debug` (`ricar up debug` → `ricar enter debug`),
починить/собрать внутри, `ricar commit debug tvp_image:latest`, затем `ricar clean <prof> && ricar up <prof>`.

---

## 10. Проверки/диагностика (быстрый набор)

Все топики под `/<id>/` (ниже пример `<id>=alpha`):
```bash
ricar ps                                              # статусы контейнеров
ricar logs <profile>                                  # логи
ricar exec sensing ros2 topic hz /alpha/sensing/livox/points   # лидар стримит?
ricar exec sensing ros2 topic hz /alpha/sensing/livox/imu      # IMU стримит? (~200 Гц)
ricar exec sensing ros2 topic echo /alpha/sensing/livox/imu --once   # accel.z≈1.0?
ricar exec vehicle ros2 topic echo /alpha/odom                 # одометрия kobuki
ricar exec vehicle ros2 topic hz /alpha/odom                   # ~50 Гц
ricar exec localization ros2 topic echo /alpha/lio_sam/mapping/odometry  # поза lio_sam
ros2 run tf2_tools view_frames                           # TF-дерево -> frames.pdf (фреймы alpha/...)
ricar exec vehicle ros2 run foxglove_bridge foxglove_bridge  # foxglove напрямую (ws://<ip>:8765)
```

---

## 11. Roadmap контроллеров (план тестов) и известные TODO

**План (3 этапа):**
1. ✅ **Этап 1 — Cruise Control (CC):** один робот держит `v_ref`. Внедрён:
   `swarm_cc_mpc_node` в профиле `control`. Тест: `ricar up vehicle transforms control`,
   колёса в воздух, `ros2 param set /alpha/control/swarm_cc_mpc_node start true`.
2. ⬜ **Этап 2 — ACC (адаптивный круиз):** второй робот, следование за peer по лидару.
   Уже готово к подключению: `swarm_acc_mpc_node`, `peer_localization`, `swarm_msgs/Telemetry`.
   Инфраструктура **namespacing под 2 робота готова** (см. раздел Namespacing): на втором
   ноуте задать свой `VEHICLE_ID` (напр. `robot2`) — топики/TF/ноды разведутся без коллизий.
3. ⬜ **Этап 3 — Lateral:** боковое управление для обоих роботов. Для одного (лидера)
   lateral уже внедрён (профиль `control`, `lateral:=true`, интегратор от offset на круге).

**Известные TODO / нюансы:**
- `base_link→lidar_70` — заглушка z=0.2. Вписать реальный mount MID360.
- `imuGravity=1.0` — рабочий обход того, что accel в g. «Чище»: домножать accel на 9.81
  в `rewrite_frames` и вернуть `imuGravity=9.81`.
- Одометрии kobuki и lio_sam **не сфьюжены** (параллельны). Для единого `map→odom→base_link`
  — `robot_localization` (EKF), отдельная задача.
- Выход twist_estimator (`/sensing/vehicle_velocity_converter/twist*`) пока никто не потребляет.
- `tau` в CC — нужна идентификация по step-response kobuki.
- foxglove_visualizer слушает несуществующие у нас autoware/caterwil топики — простаивает (не фатально).

---

## 12. Ветка / git

Репо `just-robotics/robot`, ветка **`turtlebot2`**. Изменения этой адаптации —
на ней. `diff_drive` (origin/HEAD) — другой робот, не трогать.
