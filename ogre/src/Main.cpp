#include "controller/Controller.hpp"
#include "model/Model.hpp"
#include "view/View.hpp"

int main(int argc, char* argv[]) {
    View view;
    Model model(view);
    Controller controller(model, view);
    return 0;
}
