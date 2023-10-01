//
// Created by user on 05.03.23.
//

#include "connect.hpp"


void Connect::exchange() {
    auto start_timer = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_timer).count() > int(TIMER)) {
        Connect::sendCommand();
        Connect::receiveMessage();
        start_timer = std::chrono::system_clock::now();
    }
}


void Connect::resetCommand() {
    command[COMMAND_START_BYTE1_CELL] = START_BYTE;
    command[COMMAND_START_BYTE2_CELL] = START_BYTE;
    command[COMMAND_TASK_CELL] = PING_TASK;
    command[COMMAND_VALUE1_CELL] = PING_VALUE1;
    command[COMMAND_VALUE2_CELL] = PING_VALUE2;
    calcCommandCheckSum();
}


bool Connect::openArduino() {
    if (Arduino == -1) {
        Arduino = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (Arduino == -1) {
            return false;
        }
    }

    tcgetattr(Arduino, &SerialPortSettings);

    SerialPortSettings.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;     // 8 bit chars
    SerialPortSettings.c_cflag &= ~(PARENB | PARODD);  // shut off parody
    SerialPortSettings.c_cflag &= ~CSTOPB; //no scts stop
    SerialPortSettings.c_iflag &= ~IGNBRK; //disable break processing
    SerialPortSettings.c_iflag = 0;        // no echo
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
    SerialPortSettings.c_oflag = 0;        // no remapping
    SerialPortSettings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG |IEXTEN);
    SerialPortSettings.c_cc[VMIN] = 0;     // read doesn't block
    SerialPortSettings.c_cc[VTIME] = 0;    // 0s read timeout

    tcsetattr(Arduino,TCSANOW,&SerialPortSettings);

    return true;
}


bool Connect::setConnection() {
    if (!openArduino()) {
        std::cout << "Unable to connect" << std::endl;
        return false;
    }

    resetCommand();
    bool message_flag = false;
    auto start_timer = std::chrono::system_clock::now();
    while (!message_flag) {
        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > int(TIMER)) {
            sendCommand();
            message_flag = receiveMessage();
            start_timer = std::chrono::system_clock::now();
        }
    }
    std::cout << "connected" << std::endl;
    sleep(1);
    return true;
}


void Connect::disconnectArduino() {
    close(Arduino);
}


uint8_t Connect::crc8(const uint8_t pocket[], size_t size) {
    uint8_t BYTE_SIZE = 8;
    uint8_t MSB_MASK = 0x80;
    uint8_t byte;
    uint8_t POLY = 0x7;
    uint8_t crc8 = 0xFF;

    for (size_t cell = 0; cell < size; cell++) {

        byte = pocket[cell];
        crc8 = crc8 ^ byte;

        for (int byte_number = 0; byte_number < BYTE_SIZE; byte_number++) {

            if (crc8 & MSB_MASK) {
                crc8 = (crc8 << 1) ^ POLY;
            }
            else {
                crc8 = crc8 << 1;
            }
        }
    }
    return crc8;
}


void Connect::calcCommandCheckSum() {
    command[COMMAND_CHECKSUM_CELL] = crc8(command, COMMAND_SIZE - 1);
}


uint8_t Connect::calcMessageCheckSum(uint8_t buffer[]) {
    return crc8(buffer, MESSAGE_SIZE);
}


void Connect::sendCommand() {
    if (!openArduino()) {
        return;
    }

    calcCommandCheckSum();
    write(Arduino, command, COMMAND_SIZE);
    resetCommand();
}


void Connect::setTask(uint8_t task) {
    command[COMMAND_TASK_CELL] = task;
}


void Connect::setValue(uint16_t value) {
    command[COMMAND_VALUE1_CELL] = uint8_t(value / 100);
    command[COMMAND_VALUE2_CELL] = uint8_t(value % 100);
}


bool Connect::receiveMessage() {
    if (!openArduino()) {
        return false;
    }

    uint8_t buf[MESSAGE_SIZE];
    read(Arduino, buf, MESSAGE_SIZE);

    if (buf[MESSAGE_START_BYTE1_CELL] == START_BYTE && buf[MESSAGE_START_BYTE2_CELL] == START_BYTE) {
        if (!calcMessageCheckSum(buf)) {
            std::memcpy(message, buf, sizeof(uint8_t) * MESSAGE_SIZE);
            memset(buf,0,MESSAGE_SIZE);
            return true;
        }
    }
    return false;
}


uint8_t Connect::getMessageAnswer() {
    return message[MESSAGE_ANSWER_CELL];
}


void Connect::stop() {
    setTask(STOP_TASK);
    sendCommand();
    receiveMessage();
}


void Connect::moveForward() {
    setTask(MOVE_FORWARD_TASK);
    setValue(DEFAULT_SPEED);
    sendCommand();
    receiveMessage();
}


void Connect::moveBackward() {
    setTask(MOVE_BACKWARD_TASK);
    setValue(DEFAULT_SPEED);
    sendCommand();
    receiveMessage();
}


void Connect::turnRight() {
    setTask(TURN_RIGHT_TASK);
    setValue(DEFAULT_ROTATE_SPEED);
    sendCommand();
    receiveMessage();
}


void Connect::turnLeft() {
    setTask(TURN_LEFT_TASK);
    setValue(DEFAULT_ROTATE_SPEED);
    sendCommand();
    receiveMessage();
}


void Connect::ledOn() {
    setTask(LED_ON_TASK);
    sendCommand();
    receiveMessage();
}


void Connect::ledOff() {
    setTask(LED_OFF_TASK);
    sendCommand();
    receiveMessage();
}
