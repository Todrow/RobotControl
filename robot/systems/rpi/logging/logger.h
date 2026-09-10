#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <stdexcept>

/**
 * Асинхронный потокобезопасный логгер с уровнями Info, Debug, Error.
 * Запись в файл выполняется в отдельном потоке, не блокируя вызывающий код.
 */
class Logger {
public:
    enum class Level { Info, Debug, Error };

    /**
     * Конструктор открывает файл для добавления и запускает фоновый поток.
     * @param filename путь к файлу журнала
     * @throws std::runtime_error если файл не удалось открыть
     */
    explicit Logger(const std::string& filename)
        : m_filename(filename), m_running(true) {
        m_file.open(m_filename, std::ios::app);
        if (!m_file.is_open()) {
            throw std::runtime_error("Failed to open log file: " + m_filename);
        }
        // Запускаем поток только после успешного открытия файла
        m_thread = std::make_unique<std::thread>(&Logger::processQueue, this);
    }

    /**
     * Деструктор корректно останавливает фоновый поток и закрывает файл.
     */
    ~Logger() {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_running = false;
        }
        m_cv.notify_one();
        if (m_thread && m_thread->joinable()) {
            m_thread->join();
        }
        if (m_file.is_open()) {
            m_file.close();
        }
    }

    // Запрет копирования
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /**
     * Записать сообщение уровня INFO.
     * Принимает произвольное количество аргументов, которые будут выведены подряд.
     */
    template<typename... Args>
    void info(Args&&... args) {
        log(Level::Info, std::forward<Args>(args)...);
    }

    /**
     * Записать сообщение уровня DEBUG.
     */
    template<typename... Args>
    void debug(Args&&... args) {
        log(Level::Debug, std::forward<Args>(args)...);
    }

    /**
     * Записать сообщение уровня ERROR.
     */
    template<typename... Args>
    void error(Args&&... args) {
        log(Level::Error, std::forward<Args>(args)...);
    }

private:
    /**
     * Внутренний метод формирования строки сообщения и постановки в очередь.
     */
    template<typename... Args>
    void log(Level level, Args&&... args) {
        // Формируем текст сообщения с меткой времени и уровнем
        std::ostringstream oss;
        oss << formatCurrentTime() << " [" << levelToString(level) << "] ";
        appendArgs(oss, std::forward<Args>(args)...);
        std::string message = oss.str();

        // Добавляем в очередь и уведомляем поток-обработчик
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.push(std::move(message));
        }
        m_cv.notify_one();
    }

    /**
     * Рекурсивно выводит аргументы в поток.
     */
    template<typename T>
    void appendArgs(std::ostringstream& oss, T&& arg) {
        oss << arg;
    }

    template<typename First, typename... Rest>
    void appendArgs(std::ostringstream& oss, First&& first, Rest&&... rest) {
        oss << first;
        appendArgs(oss, std::forward<Rest>(rest)...);
    }

    /**
     * Возвращает текущее время в формате "YYYY-MM-DD HH:MM:SS.mmm".
     */
    std::string formatCurrentTime() {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now;
#ifdef _WIN32
        localtime_s(&tm_now, &time_t_now);
#else
        localtime_r(&time_t_now, &tm_now);
#endif
        std::ostringstream oss;
        oss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) % 1000;
        oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return oss.str();
    }

    /**
     * Преобразует уровень в строковое представление.
     */
    std::string levelToString(Level level) {
        switch (level) {
            case Level::Info:  return "INFO";
            case Level::Debug: return "DEBUG";
            case Level::Error: return "ERROR";
            default:           return "UNKNOWN";
        }
    }

    /**
     * Фоновый поток: ожидает сообщения в очереди и пишет их в файл.
     */
    void processQueue() {
        while (true) {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this] { return !m_queue.empty() || !m_running; });

            // Выход, если очередь пуста и работа завершена
            if (m_queue.empty() && !m_running) {
                break;
            }

            // Переносим все сообщения из очереди в файл
            while (!m_queue.empty()) {
                std::string message = std::move(m_queue.front());
                m_queue.pop();
                // Временно освобождаем мьютекс на время записи
                lock.unlock();
                if (m_file.is_open()) {
                    m_file << message << std::endl;
                    m_file.flush(); // гарантирует немедленную запись на диск
                }
                lock.lock();
            }
        }
    }

private:
    std::string m_filename;
    std::ofstream m_file;
    std::queue<std::string> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::unique_ptr<std::thread> m_thread;
    bool m_running;
};