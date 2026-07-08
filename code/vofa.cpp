#include "vofa.hpp"

// 构造/析构
UartVofa::UartVofa(const char* dev) {
    fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd < 0) {
        perror("open uart");
        return;
    }

    struct termios tty;
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(fd, TCSANOW, &tty);
}

UartVofa::~UartVofa() {
    if(fd >= 0) close(fd);
}

void UartVofa::send(const char* data, int len) {
    if(fd >= 0)
        write(fd, data, len);
}