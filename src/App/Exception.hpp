//
// Created by user on 3/23/23.
//

#ifndef OMEGABOT_EXCEPTION_H
#define OMEGABOT_EXCEPTION_H


#include "header.h"


class Exception : std::exception {
private:
    std::string message;
public:
    explicit Exception(std::string _message) {message = std::move(_message);};

    [[maybe_unused]] std::string getMessage() const {return message;};
};


#endif //OMEGABOT_EXCEPTION_H
