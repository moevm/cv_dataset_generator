#pragma once
#include "../controller/Config.hpp"
#include "../view/View.hpp"
#include "entities/Camera.hpp"
#include "entities/Light.hpp"
#include "entities/Scene.hpp"
#include <memory>

class Model {
public:
    Model(View&, Config&);
    Camera& getCamera();
    Camera const& getCamera() const;

private:
    View& view;
    std::unique_ptr<Light> light;
    std::unique_ptr<Camera> camera;
    std::unique_ptr<Scene> scene;
};