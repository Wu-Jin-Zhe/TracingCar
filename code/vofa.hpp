#ifndef __VOFA_HPP_
#define __VOFA_HPP_

#pragma once

#include "zf_common_headfile.hpp"
#include <sstream>
#include <string>

// C 库包含，不用包 extern "C"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

class UartVofa {
    int fd;
public:
    UartVofa(const char* dev);
    ~UartVofa();

    void send(const char* data, int len);

    // 可变参数发送
    template<typename... Args>
    void send_values(Args... args) {
        std::ostringstream oss;
        append_to_stream(oss, args...);
        std::string str = oss.str();
        str += "\n";
        send(str.c_str(), str.length());
    }

private:
    template<typename T>
    void append_to_stream(std::ostringstream &oss, T value) {
        oss << value;
    }

    template<typename T, typename... Args>
    void append_to_stream(std::ostringstream &oss, T value, Args... args) {
        oss << value << ",";
        append_to_stream(oss, args...);
    }
};

#endif