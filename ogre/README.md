# Использование

При запуске приложения создаётся 3D-сцена с объектами по умолчанию.
По сцене можно перемещаться с использованием клавиатуры (<kbd>WASD</kbd> + <kbd>QE</kbd>) и мыши для вращения камеры. Если задана траектория, можно перемещаться по её точкам с помощью стрелок <kbd>&larr;</kbd> и <kbd>&rarr;</kbd>. 

При нажатии на пробел в файл `<output>/scene_<position>.png` сохраняется снимок сцены. 

Если в аргументах была задана траектория движения, при нажатии <kbd>Enter</kbd> будет сохранена серия снимков в `<output>/scene_0_<position_0>.png ... <output>/scene_N_<position_N>.png`.

## Параметры запуска

```bash
./SimpleScene ([-t <trajectory>] | [-p <position>...]) [-o <output>]
```

Параметр | Описание 
--- | --- 
`-t <trajectory>` | Задаёт траекторию камеры в файле `<trajectory>`.
`-p <position>` | Задаёт исходное положение и ориентацию камеры.
`-o <output>` | Задаёт директорию для сохранения снимков. По умолчанию файлы сохраняются в `output/`.
`-c <camera>` | <kbd>TODO</kbd> Задаёт параметры камеры через конфигурационный файл. При отсутствии этого параметра используется одна камера без искажений.
`-m <model>` | <kbd>TODO</kbd> Задаёт положение объектов на сцене.

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

Также необходимо делать `mount` для всех файлов, которые передаются в агрументы запуска. Полный пример запуска контейнера:

```bash
docker run --network host \
    -e DISPLAY=$DISPLAY \
    --mount type=bind,source="$(pwd)",target=/input \
    --mount type=bind,source="$(pwd)"/output,target=/output \
    <image> -t /input/trajectory.txt -o /output
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
