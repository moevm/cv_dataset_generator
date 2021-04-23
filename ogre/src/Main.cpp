#include "controller/Config.hpp"
#include "controller/Controller.hpp"

int main(int argc, char* argv[]) {
    Config config = argParse(argc, argv);
    Controller controller(config);
    return 0;
}
