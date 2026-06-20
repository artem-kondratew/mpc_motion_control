# caterwil_sensor_kit_launch

ROS 2 launch-пакет для запуска сенсоров платформы.

## Назначение

- централизованный запуск sensor kit;
- единая точка входа для последующего расширения (GNSS, камеры, preprocessing);
- совместимость с общей структурой launch-пакетов проекта.

## Содержимое

- `launch/sensing.launch.py` — основной launch-файл сенсоров.

Текущее поведение `sensing.launch.py`:

- подключает `livox_drivers/launch/all_lidars.launch.py`;
- запускает lidar-часть sensor kit.

## Зависимости

Основные runtime-зависимости из `package.xml`:

- `livox_drivers` (используется через include launch);
- `autoware_gnss_poser`;
- `autoware_pointcloud_preprocessor`;
- `autoware_vehicle_velocity_converter`;
- `common_sensor_launch`;
- `topic_tools`;
- `ublox_gps`;
- `usb_cam`.

## Сборка

Из корня workspace:

```bash
colcon build --packages-select caterwil_sensor_kit_launch
source install/setup.bash
```

## Запуск

```bash
ros2 launch caterwil_sensor_kit_launch sensing.launch.py
```

## Примечания

- Пакет не содержит URDF/TF-логику, только запуск сенсорных нод. TF-логика включена в `caterwil_description`.
- Для полного стека запускайте верхнеуровневый launch из `tvp_launch`.
