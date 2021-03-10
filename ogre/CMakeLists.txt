project(SimpleScene)

find_package(OGRE 1.11 REQUIRED COMPONENTS Bites RTShaderSystem)

add_executable(SimpleScene SimpleScene.cpp)

target_link_libraries(SimpleScene OgreBites OgreRTShaderSystem)
