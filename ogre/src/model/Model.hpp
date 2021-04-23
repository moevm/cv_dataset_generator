#pragma once
#include "../controller/Config.hpp"
#include "../view/View.hpp"
#include "OgreInput.h"
#include "Position.hpp"
#include "entities/Camera.hpp"
#include "entities/Cube.hpp"
#include "entities/Ground.hpp"
#include "entities/Light.hpp"
#include "entities/Sphere.hpp"
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
    std::unique_ptr<Cube> cube;
    std::unique_ptr<Sphere> sphere;
    std::unique_ptr<Ground> ground;
};