# Gazebo & ROS

## Запуск Docker-контейнера

```bash
docker build -t <image> .

xhost local:docker

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="$(pwd)/src:/catkin_ws/src:rw" \
    --device=/dev/dri:/dev/dri \
    <image> \
    bash
```

## Использование

Запуск тестовой симуляции:
```bash
roslaunch cottage cottage_blender.launch
```

После запуска можно управлять симуляцией командами ниже. Для этого имеет смысл запустить `tmux` (он уже установлен в контейнере), создать два окна, в одном из них выполнить предыдущую команду, а в другом — управляющие команды.

Просмотр изображения с камеры:
```bash
rosrun image_view image_view image:=/camera/image_raw
```

Получение позиции камеры (или любого другого объекта из текущей симуляции вместо `camera`):
```bash
rosrun camera_controls gms.py camera
```

Передвижение камеры на новую позицию:
```bash
rosrun camera_controls move.py camera [x] [y] [z] [roll] [pitch] [yaw]
```

Сохранение изображения с камеры в текущей директории (название файла создаётся из текущей позиции):
```bash
rosrun camera_controls save_image.py camera
```

Изменение конфигурации камеры через конфигурационный файл:
```bash
rosrun camera_controls set_camera_info.py [camera name] [camera_info.yaml]
```

Для передвижения по траектории и сохранение снимков из каждого узла запустите скрипт `src/camera_controls/scripts/trajectory.sh` и передайте в аргументы файл с траекторией (позиции из 6 чисел на отдельных строках). Снимки сохранятся в текущую директорию, названия снимков будут начинаться с порядкового номера точки в траектории. Пример траектории для облёта дома лежит в `src/camera_controls/test/trajectory.txt`.
```bash
rosrun camera_controls trajectory.sh [trajectory]
```

## Генератор траекторий

Генератор траекторий лежит в `src/dataset_generator/src/trajectory_generator.py`. Поддерживаются два режима:
- `reference` — генерация траектории, близкой к референсной
- `curve` — генерация замкнутой кривой вокруг центра в заданном диапазоне расстояния

Параметр `-s`/`--seed` устанавливает сид рандома. Остальные параметры смотрите в `--help`.

Пример генерации траектории:

```bash
rosrun dataset_generator trajectory_generator.py curve 1 2
```

## Визуализация траектории

Визуализация траектории в Gazebo (отображается только в GUI, не видна на снимках виртуальной камеры):

```bash
rosrun dataset_generator trajectory_visualizer [trajectory]
```

Удаление визуализации:
```bash
gz marker -x
```