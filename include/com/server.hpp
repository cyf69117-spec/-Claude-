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
 * @file server.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 套接字通信（服务器）
 * @version 0.1
 * @date 2024-09-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <atomic>
#include <mutex>
#include <vector>
#include <cerrno>
#include <poll.h>
#include <fcntl.h>
#include "uart.hpp"

using namespace std;

class Server
{
private:
    int socketId = -1, newSocket = -1;
    std::atomic<bool> receiving{false};
    std::mutex mtxSocket;
    std::vector<uint8_t> pending; // TCP可拆包/粘包，不能按单次recv长度判断协议帧
    struct sockaddr_in address;
    std::thread threadRes; // 接收子线程

    /**
     * @brief 32位数据内存对齐/联合体
     *
     */
    typedef union
    {
        char buffC[4];
        uint8_t buff[4];
        float float32;
        int int32;
    } Bit32Union;

    /**
     * @brief 16位数据内存对齐/联合体
     *
     */
    typedef union
    {
        char buffC[2];
        uint8_t buff[2];
        int int16;
        uint16_t uint16;
    } Bit16Union;

public:
    Server() {};
    ~Server()
    {
        closeServer();
    };

    Uart uart;
    std::atomic<int> countDrop{0}; // 应用程序掉线计数器
    std::atomic<bool> startApp{false}; // 应用程序启动标志

    /**
     * @brief 启动服务器监听
     *
     */
    bool start()
    {
        if (receiving || threadRes.joinable())
            return false;
        if (uart.open("/dev/ttyUSB0") != 0)
            return false;
        uart.startReceive();
        socketId = socket(AF_INET, SOCK_STREAM, 0);
        if (socketId < 0)
            return false;
        int reuse = 1;
        setsockopt(socketId, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        address = {};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(8899);
        if (::bind(socketId, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
            listen(socketId, 3) < 0)
        {
            close(socketId);
            socketId = -1;
            return false;
        }
        fcntl(socketId, F_SETFL, fcntl(socketId, F_GETFL, 0) | O_NONBLOCK);
        receiving = true;
        threadRes = std::thread([this]() {
            while (receiving.load())
            {
                pollfd listener{socketId, POLLIN, 0};
                const int ready = poll(&listener, 1, 100);
                if (!receiving)
                    break;
                if (ready < 0 && errno != EINTR)
                    break;
                if (ready <= 0)
                    continue;
                socklen_t len = sizeof(address);
                const int fd = accept(socketId, reinterpret_cast<sockaddr *>(&address), &len);
                if (fd < 0)
                {
                    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                        continue;
                    break;
                }
                {
                    std::lock_guard<std::mutex> lock(mtxSocket);
                    if (!receiving)
                    {
                        close(fd);
                        break;
                    }
                    newSocket = fd;
                }
#ifdef SO_NOSIGPIPE
                int noSigpipe = 1;
                setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &noSigpipe, sizeof(noSigpipe));
#endif
                fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
                pending.clear();
                while (receiving.load() && receive()) {}
                // 即使应用异常崩溃没能发送停车帧，Boot发现断连也必须停车。
                uart.carControl(0.0f, PWMSERVOMID);
                startApp = false;
                std::lock_guard<std::mutex> lock(mtxSocket);
                close(newSocket);
                newSocket = -1;
            }
        });
        return true;
    }

    /**
     * @brief 注销套接字通信
     *
     */
    void stopApp()
    {
        uart.carControl(0.0f, PWMSERVOMID);
        startApp = false;
        std::lock_guard<std::mutex> lock(mtxSocket);
        if (newSocket >= 0)
            shutdown(newSocket, SHUT_RDWR); // 超时后断开旧连接，恢复执行也不能沿旧链路再次加速
    }

    void closeServer()
    {
        receiving = false;
        {
            std::lock_guard<std::mutex> lock(mtxSocket);
            if (newSocket >= 0)
                shutdown(newSocket, SHUT_RDWR);
            if (socketId >= 0)
                shutdown(socketId, SHUT_RDWR);
        }
        if (threadRes.joinable())
            threadRes.join();
        if (socketId >= 0)
        {
            close(socketId);
            socketId = -1;
        }
        uart.close();
    }

    /**
     * @brief 接收客户端数据
     *
     */
    bool receive()
    {
        uint8_t buffer[1024];
        const ssize_t len = recv(newSocket, buffer, sizeof(buffer), 0);
        if (len < 0 && errno == EINTR)
            return true;
        if (len <= 0)
            return false;
        startApp = true;
        countDrop = 0;
        pending.insert(pending.end(), buffer, buffer + len);
        while (!pending.empty())
        {
            if (pending[0] != USB_FRAME_HEAD)
            {
                pending.erase(pending.begin()); // 兼容不转发给下位机的3字节应用心跳
                continue;
            }
            if (pending.size() < 3)
                break;
            const size_t size = pending[2];
            if (size < USB_FRAME_LENMIN || size > USB_FRAME_LENMAX)
            {
                pending.erase(pending.begin());
                continue;
            }
            if (pending.size() < size)
                break;
            uint8_t checksum = 0;
            for (size_t i = 0; i + 1 < size; ++i)
                checksum += pending[i];
            if (checksum != pending[size - 1])
            {
                pending.erase(pending.begin());
                continue;
            }
            uart.transmitBytes(pending.data(), size);
            pending.erase(pending.begin(), pending.begin() + size);
        }
        return true;
    }

    /**
     * @brief 发送数据
     *
     */
    void transmit(string data)
    {
        std::lock_guard<std::mutex> lock(mtxSocket);
        if (!startApp || newSocket < 0)
            return;
        int flags = MSG_DONTWAIT;
#ifdef MSG_NOSIGNAL
        flags |= MSG_NOSIGNAL;
#endif
        // 按键通知很短；发送不完整时断开连接，触发主机与Boot两侧停车。
        if (send(newSocket, data.data(), data.size(), flags) != static_cast<ssize_t>(data.size()))
            shutdown(newSocket, SHUT_RDWR);
    }
};
