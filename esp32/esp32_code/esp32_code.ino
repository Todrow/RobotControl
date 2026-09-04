/*
 * ESP32-C3 Super Mini, Arduino IDE, библиотека ESP32Servo.
 * UART1: RX = GPIO9, TX = GPIO10, 115200 бод, 8N1.
 * Подключение: TX отправителя -> GPIO9, общая GND, уровни 3.3 В.
 * GPIO9 также задаёт режим загрузки: при сбросе он должен быть HIGH.
 *
 * Команда: R:100%|L:50%\n (LF обязателен; CRLF тоже допустим).
 * Целые проценты: -100..100; плюс = вперёд, минус = назад, 0 = стоп.
 * Повторяйте команду, например, каждые 50 мс: через 500 мс без
 * корректной полной команды оба двигателя останавливаются.
 */

#include <Arduino.h>
#include <ESP32Servo.h>

const int UART_RX_PIN = 9;
const int UART_TX_PIN = 10;
const uint32_t UART_BAUD = 115200;

// Принято: колесо 1 (GPIO4) — правое, колесо 2 (GPIO5) — левое.
const int ESC_RIGHT_PIN = 4;
const int ESC_LEFT_PIN = 5;
const int RIGHT_DIRECTION = -1;
const int LEFT_DIRECTION = 1;

const int PWM_MIN = 1000;
const int PWM_NEUTRAL = 1500;
const int PWM_MAX = 2000;
// 90% = отклонение на 50 мкс от нейтрали; это команда ESC, не замер RPM.
// R:90%|L:90% даёт 1450/1550 мкс; остальные значения масштабируются линейно.
const int PWM_REFERENCE_PERCENT = 90;
const int PWM_REFERENCE_OFFSET = 50;
const uint32_t COMMAND_TIMEOUT_MS = 500;

Servo escRight, escLeft;
char rxLine[32];
size_t rxLength = 0;
bool discardLine = false;
bool commandActive = false;
uint32_t lastCommandMs = 0;
uint32_t lastByteMs = 0;

void stopMotors() {
  escRight.writeMicroseconds(PWM_NEUTRAL);
  escLeft.writeMicroseconds(PWM_NEUTRAL);
}

bool parsePercent(const char *&cursor, int &percent) {
  int sign = 1;
  if (*cursor == '-' || *cursor == '+') {
    if (*cursor == '-') sign = -1;
    ++cursor;
  }
  if (*cursor < '0' || *cursor > '9') return false;

  int value = 0;
  while (*cursor >= '0' && *cursor <= '9') {
    value = value * 10 + (*cursor - '0');
    if (value > 100) return false;
    ++cursor;
  }
  if (*cursor != '%') return false;
  ++cursor;
  percent = sign * value;
  return true;
}

bool parseCommand(const char *line, int &right, int &left) {
  const char *cursor = line;
  if (*cursor++ != 'R' || *cursor++ != ':') return false;
  if (!parsePercent(cursor, right)) return false;
  if (*cursor++ != '|' || *cursor++ != 'L' || *cursor++ != ':') return false;
  if (!parsePercent(cursor, left)) return false;
  if (*cursor == '\r') ++cursor;
  return *cursor == '\0';
}

void receiveByte(char ch) {
  lastByteMs = millis();
  if (ch == '\n') {
    if (!discardLine && rxLength > 0) {
      rxLine[rxLength] = '\0';
      int right = 0;
      int left = 0;
      // Оба двигателя меняются только после проверки всей строки.
      if (parseCommand(rxLine, right, left)) {
        escRight.writeMicroseconds(PWM_NEUTRAL + RIGHT_DIRECTION * right * PWM_REFERENCE_OFFSET / PWM_REFERENCE_PERCENT);
        escLeft.writeMicroseconds(PWM_NEUTRAL + LEFT_DIRECTION * left * PWM_REFERENCE_OFFSET / PWM_REFERENCE_PERCENT);
        lastCommandMs = millis();
        commandActive = true;
      }
    }
    rxLength = 0;
    discardLine = false;
    return;
  }

  if (discardLine) return;
  // Переполненную/повреждённую строку отбрасываем целиком до LF.
  if (rxLength >= sizeof(rxLine) - 1 ||
      (ch != '\r' && (ch < ' ' || ch > '~'))) {
    rxLength = 0;
    discardLine = true;
    return;
  }
  rxLine[rxLength++] = ch;
}

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  escRight.setPeriodHertz(50);
  escLeft.setPeriodHertz(50);
  escRight.attach(ESC_RIGHT_PIN, PWM_MIN, PWM_MAX);
  escLeft.attach(ESC_LEFT_PIN, PWM_MIN, PWM_MAX);
  stopMotors();

  Serial.begin(115200);
  Serial.println("Инициализация ESC в нейтрали, жду 3 секунды...");
  delay(3000);

  // Начинаем приём после инициализации ESC, без накопления старых команд.
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("Готов: UART RX=9, TX=10. Формат: R:90%|L:90% + LF");
}

void loop() {
  const uint32_t now = millis();
  if (commandActive && static_cast<uint32_t>(now - lastCommandMs) >= COMMAND_TIMEOUT_MS) {
    stopMotors();
    commandActive = false;
  }
  // Не соединяем старую незавершённую команду с байтами после паузы.
  if (rxLength > 0 && static_cast<uint32_t>(now - lastByteMs) >= COMMAND_TIMEOUT_MS) {
    rxLength = 0;
    discardLine = true;
  }

  // Ограничиваем работу за проход, чтобы поток мусора не задерживал таймаут.
  for (size_t count = 0; count < 64 && Serial1.available() > 0; ++count) {
    const int value = Serial1.read();
    if (value < 0) break;
    receiveByte(static_cast<char>(value));
  }
  delay(1);
}
