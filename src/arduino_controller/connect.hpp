//
// Created by user on 05.03.23.
//

#ifndef MANIPULATOR_CONNECT_H
#define MANIPULATOR_CONNECT_H


#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <ncurses.h>

#include "../arduino/config.h"


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

    static void setTask(uint8_t task);
    static void setValue(uint16_t value);

public:
    static bool receiveMessage();
    static uint8_t getMessageAnswer();

public:
    static void stop();
    static void moveForward();
    static void moveBackward();
    static void turnRight();
    static void turnLeft();
    static void ledOn();
    static void ledOff();
};


inline struct termios SerialPortSettings;


#endif //MANIPULATOR_CONNECT_H
