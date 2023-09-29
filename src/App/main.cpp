#include "header.h"
#include "graphics.h"


int main() {
    if (!Connect::setConnection()) {
        Graphics::finish();
    }

    Graphics::init_graphics();

    std::thread graphics_thr(Graphics::key_proc);
    std::thread connect_thr(Connect::exchange);
    std::thread sighandler_thr(signal, SIGINT, Graphics::sighandler);

    graphics_thr.join();
    connect_thr.join();
    sighandler_thr.join();

    Graphics::finish();
}
