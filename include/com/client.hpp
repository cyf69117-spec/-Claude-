#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file client.hpp
 *
 * @author Leo
 * @brief 上下位机串口通信协议(基于套接字转换)
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>
#include "utils/servo.hpp"

class Client
{
private:
    static constexpr uint8_t FRAME_HEAD = 0x42;
    static constexpr uint8_t ADDR_CARCTRL = 1;
    static constexpr uint8_t ADDR_BUZZER = 4;
    int socketId = -1;
    int countInit = 0;
    std::thread threadRes;
    std::mutex mtxSend;
    std::atomic<bool> receiving{false};
    std::atomic<bool> connected{false};

    void receiveLoop(int fd)
    {
        char buffer[1024];
        std::string pending;
        while (receiving.load())
        {
            const ssize_t len = recv(fd, buffer, sizeof(buffer), 0);
            if (len > 0)
            {
                // TCP没有消息边界，按实际长度处理，并保留跨包的按键关键字。
                pending.append(buffer, static_cast<size_t>(len));
                size_t pos;
                while ((pos = pending.find("Keypress")) != std::string::npos)
                {
                    keypress.store(true);
                    pending.erase(0, pos + 8);
                }
                if (pending.size() > 7)
                    pending.erase(0, pending.size() - 7);
            }
            else if (len < 0 && errno == EINTR)
                continue;
            else
                break; // 断连或不可恢复错误，不能继续忙循环读取旧缓冲区。
        }
        connected.store(false);
        receiving.store(false);
    }

public:
    Client() = default;
    ~Client() { closeClient(); }
    std::atomic<bool> keypress{false}; // 接收线程写、主线程读

    enum Buzzer
    {
        BUZZER_OK = 0,
        BUZZER_WARNNING,
        BUZZER_FINISH,
        BUZZER_DING,
        BUZZER_START,
    };

    bool start(void)
    {
        if (threadRes.joinable() || socketId >= 0)
            return false;
        const int fd = socket(PF_INET, SOCK_STREAM, 0);
        if (fd < 0)
            return false;
#ifdef SO_NOSIGPIPE
        int noSigpipe = 1;
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &noSigpipe, sizeof(noSigpipe));
#endif
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(8899);
        if (inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) <= 0 ||
            connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        {
            close(fd);
            return false;
        }
        // 内核发送超时作为非阻塞标志之外的第二层保护。
        timeval timeout{0, 20000};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        socketId = fd;
        connected.store(true);
        receiving.store(true);
        try
        {
            threadRes = std::thread(&Client::receiveLoop, this, fd);
        }
        catch (...)
        {
            connected.store(false);
            receiving.store(false);
            close(fd);
            socketId = -1;
            throw;
        }
        return true;
    }

    bool isConnected() const { return connected.load(); }

    void closeClient()
    {
        // 先尝试停车，再shutdown唤醒阻塞接收；join完成后才close，避免描述符复用。
        carControl(0.0f, PWMSERVOMID);
        receiving.store(false);
        {
            std::lock_guard<std::mutex> lock(mtxSend);
            if (socketId >= 0)
                shutdown(socketId, SHUT_RDWR);
        }
        if (threadRes.joinable())
            threadRes.join();
        {
            std::lock_guard<std::mutex> lock(mtxSend);
            if (socketId >= 0)
            {
                close(socketId);
                socketId = -1;
            }
        }
        connected.store(false);
    }

    void convertUnCharToStr(char *data, unsigned char *buff)
    {
        int len = sizeof(buff) / sizeof(buff[0]);
        for (int i = 0; i < len; i++)
        {
            // 格式化输str,每unsigned char 转换字符占两位置%x写输%X写输
            sprintf(data + i * 2, "%02x", buff[i]);
        }
    }

    bool transmit(const uint8_t *buff, int len)
    {
        std::lock_guard<std::mutex> lock(mtxSend);
        if (socketId < 0 || !connected.load() || !buff || len <= 0)
            return false;
        // 限制单次发送预算，处理短写和EINTR，断连不能通过SIGPIPE结束进程。
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
        int sent = 0;
        while (sent < len)
        {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0)
                break;
            pollfd fd{socketId, POLLOUT, 0};
            const int ready = poll(&fd, 1, static_cast<int>(remaining));
            if (ready < 0 && errno == EINTR)
                continue;
            if (ready <= 0 || (fd.revents & (POLLERR | POLLHUP | POLLNVAL)))
                break;
            int flags = MSG_DONTWAIT;
#ifdef MSG_NOSIGNAL
            flags |= MSG_NOSIGNAL;
#endif
            const int chunk = len - sent > 4096 ? 4096 : len - sent;
            const ssize_t n = send(socketId, buff + sent, static_cast<size_t>(chunk), flags);
            if (n > 0)
                sent += static_cast<int>(n);
            else if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
                continue;
            else
                break;
        }
        if (sent == len)
            return true;
        connected.store(false);
        shutdown(socketId, SHUT_RDWR); // 触发接收退出及Boot侧失联停车。
        return false;
    }

    bool carControl(float speed, uint16_t servo)
    {
        countInit = countInit < 50 ? countInit + 1 : 50;
        if (countInit < 50)
            speed = 0.0f;
        uint8_t buff[11]{}; // 保留原协议的额外尾字节，明确初始化为0。
        buff[0] = FRAME_HEAD;
        buff[1] = ADDR_CARCTRL;
        buff[2] = 10;
        std::memcpy(buff + 3, &speed, sizeof(speed));
        std::memcpy(buff + 7, &servo, sizeof(servo));
        for (int i = 0; i < 9; ++i)
            buff[9] += buff[i];
        return transmit(buff, sizeof(buff));
    }

    bool buzzerSound(Buzzer sound)
    {
        uint8_t buff[6]{};
        buff[0] = FRAME_HEAD;
        buff[1] = ADDR_BUZZER;
        buff[2] = 5;
        buff[3] = static_cast<uint8_t>(sound) + 1;
        for (int i = 0; i < 4; ++i)
            buff[4] += buff[i];
        return transmit(buff, sizeof(buff));
    }

    bool sendHeart()
    {
        // Boot以收到消息维持应用存活，3字节心跳不会转发给下位机。
        const uint8_t buff[3]{};
        return transmit(buff, sizeof(buff));
    }
};
