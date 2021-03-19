# Инструкция по сборке

## Зависимости

Перед сборкой необходимо вручную установить OGRE: [Guide to building OGRE](https://ogrecave.github.io/ogre/api/1.12/building-ogre.html).

## Сборка 

```bash
cmake CMakeLists.txt
make
```

# Запуск

```bash
./SimpleScene
```

По сцене можно перемещаться с использованием клавиатуры (`WASD` + `QE`). При нажатии на пробел в файл `Scene.png` сохраняется снимок сцены.