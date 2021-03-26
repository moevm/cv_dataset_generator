# Docker

## Сборка 

```bash
docker build -t <image> .
```

## Запуск

```bash
docker run --network host -e DISPLAY=$DISPLAY <image>
```

Перед запуском контейнера возможно потребуется выполнить 
```bash
xhost local:docker
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
./SimpleScene
```

# Использование

По сцене можно перемещаться с использованием клавиатуры (`WASD` + `QE`). При нажатии на пробел в файл `output/scene.png` сохраняется снимок сцены. Также можно задать траекторию во входном потоке, тогда будет сохранена серия снимков в `output/scene0.png ... output/sceneN.png`.
