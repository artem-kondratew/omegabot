//
// Created by user on 4/28/23.
//
#if 0
#ifndef OMEGABOT_VISION_H
#define OMEGABOT_VISION_H


#include "header.h"
#include "Connect.h"


class Vision {
private:
    inline static bool processing_flag = false;
public:
    static void init();
    static void processing();
    static void start_processing();
    static void stop_processing();
    static bool is_processing();
};
#endif


#endif //OMEGABOT_VISION_H
