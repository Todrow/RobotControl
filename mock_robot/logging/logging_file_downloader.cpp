#include "logging_file_downloader.h"

#include <iostream>
#include <fstream>
#include <cstring>
#include <thread>
#include <chrono>
#include <stdexcept>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
#endif

LoggingFileDownloader::LoggingFileDownloader(Logger& logger)
    : m_logger(logger) {}

bool LoggingFileDownloader::downloadFile(const std::string& serverIp,
                                         int serverPort,
                                         const std::string& remoteFileName,
                                         const std::string& localFilePath) {
#ifdef _WIN32
    // Инициализация Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        m_logger.error("WSAStartup failed");
        return false;
    }
#endif

    // Создание UDP-сокета
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        m_logger.error("Socket creation failed");
#ifdef _WIN32
        WSACleanup();
#endif
        return false;
    }

    // Настройка адреса сервера
    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    if (inet_pton(AF_INET, serverIp.c_str(), &serverAddr.sin_addr) <= 0) {
        m_logger.error("Invalid server IP address: ", serverIp);
        closeSocket(sock);
        return false;
    }

    // Установка таймаута на приём
    setTimeout(sock, TIMEOUT_MS);

    // Отправка запроса на получение файла
    std::string request = "GET:" + remoteFileName;
    if (sendto(sock, request.c_str(), request.size(), 0,
               reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) < 0) {
        m_logger.error("Failed to send request for file: ", remoteFileName);
        closeSocket(sock);
        return false;
    }
    m_logger.info("Request sent for file: ", remoteFileName);

    // Открытие локального файла для записи
    std::ofstream outFile(localFilePath, std::ios::binary | std::ios::trunc);
    if (!outFile.is_open()) {
        m_logger.error("Cannot open local file for writing: ", localFilePath);
        closeSocket(sock);
        return false;
    }

    // Приём файла
    uint32_t expectedSeq = 0;
    bool isLastChunk = false;
    char buffer[sizeof(uint32_t) + 1 + CHUNK_SIZE];

    while (!isLastChunk) {
        sockaddr_in fromAddr{};
        socklen_t fromLen = sizeof(fromAddr);
        int bytesReceived = recvfrom(sock, buffer, sizeof(buffer), 0,
                                     reinterpret_cast<sockaddr*>(&fromAddr), &fromLen);
        if (bytesReceived < 0) {
            // Таймаут или ошибка
            m_logger.error("Timeout or error receiving data");
            outFile.close();
            closeSocket(sock);
            return false;
        }

        if (bytesReceived < static_cast<int>(sizeof(uint32_t) + 1)) {
            m_logger.error("Received malformed packet (too small)");
            continue;
        }

        // Разбор заголовка
        uint32_t seqNum;
        uint8_t lastFlag;
        std::memcpy(&seqNum, buffer, sizeof(seqNum));
        std::memcpy(&lastFlag, buffer + sizeof(seqNum), 1);
        seqNum = ntohl(seqNum); // сетевой порядок байт

        if (seqNum != expectedSeq) {
            // Если номер не совпадает, отправляем повторный ACK для ожидаемого
            m_logger.debug("Out-of-order packet, expected ", expectedSeq, ", got ", seqNum);
            sendAck(sock, serverAddr, expectedSeq);
            continue;
        }

        // Записываем данные
        size_t dataLen = bytesReceived - sizeof(uint32_t) - 1;
        outFile.write(buffer + sizeof(uint32_t) + 1, dataLen);

        // Отправляем подтверждение
        sendAck(sock, serverAddr, seqNum);
        m_logger.debug("Received chunk ", seqNum, " (", dataLen, " bytes)");

        expectedSeq++;
        if (lastFlag == 1) {
            isLastChunk = true;
        }
    }

    outFile.close();
    closeSocket(sock);
#ifdef _WIN32
    WSACleanup();
#endif
    m_logger.info("File downloaded successfully: ", localFilePath);
    return true;
}

// Вспомогательные функции
void LoggingFileDownloader::sendAck(int sock, const sockaddr_in& serverAddr, uint32_t seqNum) {
    std::string ack = "ACK:" + std::to_string(seqNum);
    sendto(sock, ack.c_str(), ack.size(), 0,
           reinterpret_cast<const sockaddr*>(&serverAddr), sizeof(serverAddr));
}

void LoggingFileDownloader::setTimeout(int sock, int milliseconds) {
#ifdef _WIN32
    DWORD timeout = milliseconds;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
    struct timeval tv;
    tv.tv_sec = milliseconds / 1000;
    tv.tv_usec = (milliseconds % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif
}

void LoggingFileDownloader::closeSocket(int sock) {
#ifdef _WIN32
    closesocket(sock);
#else
    close(sock);
#endif
}