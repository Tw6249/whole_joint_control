#include "controller_stepper_app.hpp"

int main(int argc, char** argv) {
    return h1if::runControllerStepper(
        argc,
        argv,
        h1if::createController,
        "h1_controller_stepper");
}
