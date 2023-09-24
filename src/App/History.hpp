//
// Created by user on 3/21/23.
//

#ifndef OMEGABOT_HISTORY_H
#define OMEGABOT_HISTORY_H


#include "header.h"
#include "List.h"


class History {
private:
    inline static List list;
    inline static uint64_t index = 0;
    inline static str current_command;
public:
    static void moveUp();
    static void moveDown();
    static std::string get();
    static void append(const str &command);
    static uint64_t getIndex();
    static void setCurrentCommand(std::string command);
    static void resetIndex();
};


void History::moveUp() {
    if (index == list.getSize()) {
        return;
    }
    index++;
}


void History::moveDown() {
    if (index == 0) {
        return;
    }
    index--;
}


std::string History::get() {
    if (index == 0) {
        return current_command.get();
    }
    return list.getData(index - 1);
}


void History::append(const str &command) {
    list.pushHead(command);
}


uint64_t History::getIndex() {
    return index;
}


void History::setCurrentCommand(std::string command) {
    current_command.set(std::move(command));
}


void History::resetIndex() {
    index = 0;
}


#endif //OMEGABOT_HISTORY_H
