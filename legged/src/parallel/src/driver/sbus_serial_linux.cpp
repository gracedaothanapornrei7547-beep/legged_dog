// sbus_serial_linux.cpp —— 串口 100000-baud + 8E2 打开的隔离 TU
//
// 必须把 <asm/termbits.h> 关到独立编译单元，否则它定义的宏会污染
// <chrono> 的模板（std::chrono::duration 编译失败、std::chrono::__is_ratio
// 没成员 'value' 之类）。所以本文件 **只** include 它需要的最少 C 头，
// 不引入任何 C++ 标准库。
//
// 暴露给上层（sbus_reader.cpp）的接口：
//   extern "C" int sbus_open_100k_8e2(const char* port);
//     成功返回 fd（≥0），失败返回 -1（fd 已 close）。
//
// 配置：100000 波特率, 8 数据位, 偶校验, 2 停止位, 仅接收, 非阻塞，
// VMIN=0 / VTIME=1（100ms read 超时）。

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <asm/termbits.h>   // 必须独立 TU；与 <chrono> 等 C++ 头不能并存

extern "C" int sbus_open_100k_8e2(const char* port) {
    int fd = ::open(port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios2 tio;
    // 不能用值初始化 termios2 = {} —— 该结构在 asm/termbits.h 里字段顺序
    // 与 musl/glibc 的 termios 不同；用 ioctl 读出当前配置再改。
    if (::ioctl(fd, TCGETS2, &tio) != 0) {
        ::close(fd);
        return -1;
    }

    // 波特率：清 CBAUD → 置 BOTHER + ispeed/ospeed=100000
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = 100000;
    tio.c_ospeed = 100000;

    // 数据位 8 / 偶校验 / 2 停止位
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag |= PARENB;
    tio.c_cflag &= ~PARODD;        // 偶校验
    tio.c_cflag |= CSTOPB;         // 2 stop bits
    tio.c_cflag |= CREAD | CLOCAL;

    // 输入：开校验，关 break / 软流控等
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                     | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tio.c_iflag |= INPCK;

    // 输出 / 行模式：原样字节，关 echo / 行缓冲
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    // 阻塞特性：非阻塞 + 100ms read 超时（O_NONBLOCK 已设，VTIME 兜底）
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;

    if (::ioctl(fd, TCSETS2, &tio) != 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}
