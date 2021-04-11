#include "controller/Config.hpp"
#include "controller/Controller.hpp"

int main(int argc, char* argv[]) {
    Config config = argParse(argc, argv);
    Controller controller(std::move(config));
    return 0;
}
