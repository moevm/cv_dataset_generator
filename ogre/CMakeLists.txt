cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(SimpleScene)

find_package(OGRE 1.11 REQUIRED COMPONENTS Bites RTShaderSystem)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
# find_package(OpenCV 4.1 REQUIRED)

set(CPPS 
    src/Main.cpp 
    src/view/View.cpp 
    src/controller/Config.cpp
    src/controller/Controller.cpp
    src/controller/Trajectory.cpp
    src/model/Model.cpp
    src/model/Position.cpp
    src/model/entities/Entity.cpp
    src/model/entities/Camera.cpp
    src/model/entities/Cube.cpp
    src/model/entities/Ground.cpp
    src/model/entities/Light.cpp
    src/model/entities/Sphere.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${camera_calibration_parsers_INCLUDE_DIRS})
# include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(SimpleScene ${CPPS})

target_link_libraries(SimpleScene OgreBites OgreRTShaderSystem ${camera_calibration_parsers_LIBRARIES})
# target_link_libraries(SimpleScene OgreBites OgreRTShaderSystem ${OpenCV_LIBS})
