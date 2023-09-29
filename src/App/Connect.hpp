//
// Created by artem-kondratew on 05.03.23.
//

#ifndef OMEGABOT_CONNECT_H
#define OMEGABOT_CONNECT_H


#include "header.h"
#include "../Arduino/Config.h"
#include "Exception.hpp"


inline std::mutex connect_mutex;

inline struct termios SerialPortSettings;


class Connect {
private:
    inline static int Arduino = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    inline static uint8_t command[COMMAND_SIZE];
    inline static uint8_t message[MESSAGE_SIZE];

public:
    static void exchange();
    static void resetCommand();

private:
    static bool openArduino();

public:
    static bool setConnection();
    static void disconnectArduino();

private:
    static uint8_t crc8(const uint8_t pocket[], uint64_t size);
    static void calcCommandCheckSum();
    static uint8_t calcMessageCheckSum(uint8_t buffer[]);

public:
    static void sendCommand();

private:
    static void setTask(uint8_t task);
    static void setValue(uint16_t value);
    static void encodeCommand(uint64_t cmd);

    static void decodeMessage();

public:
    static bool receiveMessage();

public:
    static void stop();
    static void moveForward();
    static void moveBackward();
    static void turnRight();
    static void turnLeft();

    static void beep();
    static void blink();
};


#endif //OMEGABOT_CONNECT_H
