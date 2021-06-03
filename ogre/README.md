# Использование

При запуске приложения создаётся 3D-сцена с объектами по умолчанию.
По сцене можно перемещаться с использованием клавиатуры (<kbd>WASD</kbd> + <kbd>QE</kbd>) и мыши для вращения камеры. Если задана траектория, можно перемещаться по её точкам с помощью стрелок <kbd>&larr;</kbd> и <kbd>&rarr;</kbd>. 

При нажатии на пробел в файл `<output>/scene_<position>.png` сохраняется снимок сцены. 

Если в аргументах была задана траектория движения, при нажатии <kbd>Enter</kbd> будет сохранена серия снимков в `<output>/scene_0_<position_0>.png ... <output>/scene_N_<position_N>.png`.

Позиция камеры задаётся в формате `x y x pitch yaw roll`, где `pitch`, `yaw`, `roll` — углы Эйлера, задающие ориентацию камеры. Повороты применяются в порядке `pitch`, `yaw`, `roll`: сначала вокруг оси X, затем вокруг оси Y, затем вокруг оси Z. Ось X направлена вправо от центра, ось Y — вверх, ось Z — в сторону зрителя.

## Параметры запуска

```bash
./SimpleScene ([-t <trajectory>] | [-p <position>...]) [-o <output>] [-c <camera>] [-m <model>]
```

Параметр | Описание 
--- | --- 
`-t <trajectory>` | Задаёт траекторию камеры (набор позиций) в файле `<trajectory>`.
`-p <position>` | Задаёт исходное положение и ориентацию камеры.
`-o <output>` | Задаёт директорию для сохранения снимков. По умолчанию файлы сохраняются в `output/`.
`-c <camera>` | Задаёт параметры камеры через конфигурационный файл (ROS Camera Info). При отсутствии этого параметра используется одна камера без искажений.
`-m <model>` | Задаёт положение объектов на сцене (путь к файлу `.scene`).

## Тестовые сцены
* Куб и шар: [scene/checkerboard.scene](scene/basic.scene)
* Шахматная доска: [scene/checkerboard.scene](scene/checkerboard.scene), [input/trajectory_checkerboard.txt](input/trajectory_checkerboard.txt) 
* Дом: [scene/cottage_blender.scene](scene/cottage_blender.scene)

# Docker

## Сборка 

```bash
docker build -t <image> .
```

## Запуск

```bash
docker run --network host -e DISPLAY=$DISPLAY <image> [ARGS...]
```

Перед запуском контейнера возможно потребуется выполнить 
```bash
xhost local:docker
```

Для сохранения снимков в локальную директорию надо добавить следующую опцию при запуске контейнера:

```bash
--mount type=bind,source=<absolute path to output folder>,target=<container output folder>
```

Также необходимо делать `mount` для всех файлов, которые передаются в аргументы запуска. **Полный пример** запуска контейнера:

```bash
docker run --network host \
    -e DISPLAY=$DISPLAY \
    --mount type=bind,source="$(pwd)"/input,target=/input \
    --mount type=bind,source="$(pwd)"/output,target=/output \
    <image> \
    -t /input/trajectory.txt -c /input/camera_info_autobot10.yaml -o /output
```

# Сборка вручную

## Зависимости

Перед сборкой необходимо вручную установить OGRE: [Guide to building OGRE](https://ogrecave.github.io/ogre/api/1.12/building-ogre.html).

## Сборка 

```bash
cmake CMakeLists.txt
make
```

## Запуск

```bash
./SimpleScene [ARGS...]
```
