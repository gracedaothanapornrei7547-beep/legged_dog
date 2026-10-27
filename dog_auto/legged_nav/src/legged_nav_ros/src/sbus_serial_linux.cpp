// sbus_serial_linux.cpp —— 串口 100000-baud 打开的隔离 TU
//
// 暴露给上层（sbus_reader.cpp）的接口：
//   extern "C" int sbus_open_100k_8e2(const char* port);
//   extern "C" int sbus_open_100k_8n1(const char* port);
//     成功返回 fd（≥0），失败返回 -1。
//
// 8E2 = 100000 baud, 8 bits, EVEN parity, 2 stop bits (SBUS 标准)
// 8N1 = 100000 baud, 8 bits, NO parity,  1 stop bit  (CP2102 兼容)

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <asm/termbits.h>

static int sbus_open_inner(const char* port, bool parity, bool two_stop) {
    int fd = ::open(port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios2 tio;
    if (::ioctl(fd, TCGETS2, &tio) != 0) { ::close(fd); return -1; }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = 100000;
    tio.c_ospeed = 100000;

    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    if (parity)   { tio.c_cflag |= PARENB; tio.c_cflag &= ~PARODD; }
    else          { tio.c_cflag &= ~PARENB; }
    if (two_stop) { tio.c_cflag |= CSTOPB; }
    else          { tio.c_cflag &= ~CSTOPB; }
    tio.c_cflag |= CREAD | CLOCAL;

    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                     | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    if (parity) tio.c_iflag |= INPCK;

    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;

    if (::ioctl(fd, TCSETS2, &tio) != 0) { ::close(fd); return -1; }
    return fd;
}

extern "C" int sbus_open_100k_8e2(const char* port) {
    return sbus_open_inner(port, true, true);
}

extern "C" int sbus_open_100k_8n1(const char* port) {
    return sbus_open_inner(port, false, false);
}
