#pragma once

#include <string>
#include "logger.h"

class LoggingFileDownloader {
public:
    // Конструктор принимает ссылку на логгер для записи событий
    explicit LoggingFileDownloader(Logger& logger);

    /**
     * Скачивает файл с Raspberry Pi по UDP.
     * @param serverIp       IP-адрес Raspberry Pi (например "192.168.1.100")
     * @param serverPort     UDP-порт, на котором слушает сервер
     * @param remoteFileName имя файла на Raspberry Pi (например "/home/pi/data.txt")
     * @param localFilePath  путь для сохранения на компьютере
     * @return true при успешном скачивании, false при ошибке
     */
    bool downloadFile(const std::string& serverIp,
                      int serverPort,
                      const std::string& remoteFileName,
                      const std::string& localFilePath);

private:
    static constexpr size_t CHUNK_SIZE = 1024;      // размер одной порции данных
    static constexpr int TIMEOUT_MS = 2000;         // таймаут приёма/отправки
    static constexpr int MAX_RETRIES = 5;           // число повторных попыток (не используется)

    void sendAck(int sock, const sockaddr_in& serverAddr, uint32_t seqNum);
    void setTimeout(int sock, int milliseconds);
    void closeSocket(int sock);

    Logger& m_logger;
};