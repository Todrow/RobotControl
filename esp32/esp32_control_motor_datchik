/*
 * ESP32-C3 Super Mini, Arduino IDE, библиотека ESP32Servo.
 * UART1: RX = GPIO20, TX = GPIO21, 115200 бод, 8N1.
 * Подключение: TX отправителя -> GPIO20, общая GND, уровни 3.3 В.
 * Отладочные сообщения через USB при USB CDC On Boot = Enabled.
 *
 * Команда: R:100%|L:50%\n (LF обязателен; CRLF тоже допустим).
 * Целые проценты: -100..100; плюс = вперёд, минус = назад, 0 = стоп.
 * Повторяйте команду, например, каждые 50 мс: через 500 мс без
 * корректной полной команды оба двигателя останавливаются.
 *
 * Телеметрия звука: раз в 100 мс в ту же линию UART уходит строка
 *   S:<0|1>|N:<счётчик>\n
 * S — текущее состояние компаратора датчика (1 = звук выше порога),
 * N — число срабатываний с момента включения, 0..65535 с переполнением.
 * Линия полнодуплексная: приём команд это не задерживает.
 */
#include <Arduino.h>
#include <ESP32Servo.h>

const int UART_RX_PIN = 20;
const int UART_TX_PIN = 21;
const uint32_t UART_BAUD = 115200;

// Проверено на стенде: сигнал GPIO3 идёт на ПРАВОЕ колесо, GPIO4 — на ЛЕВОЕ.
const int ESC_RIGHT_PIN = 3;
const int ESC_LEFT_PIN = 4;

// Обе связки "ESC + мотор" оказались инвертированы относительно знака команды.
const int RIGHT_DIRECTION = -1;
const int LEFT_DIRECTION = -1;

// Границы, объявленные при attach(); фактический выход всегда внутри
// PWM_NEUTRAL +- PWM_MAX_OFFSET.
const int PWM_MIN = 1000;
const int PWM_NEUTRAL = 1500;
const int PWM_MAX = 2000;

// Ход на 100% от нейтрали, в микросекундах. Сжатая шкала (150 вместо 400)
// повышает разрешение команды: 1% = 1.5 мкс, рабочие проценты уходят из
// самого низа диапазона ESC.
const int PWM_MAX_OFFSET = 150;

// Минимальное отклонение, с которого колесо вообще страгивается.
// На текущем железе порог ниже 5%, поэтому компенсация не нужна.
const int PWM_DEADBAND_US = 0;

// Индивидуальная добавка к отклонению, мкс. Плюс — если это колесо
// трогается позже второго.
const int RIGHT_TRIM_US = 0;
const int LEFT_TRIM_US = 2;

// Масштаб скорости по каналам, 1000 = без изменений.
const int RIGHT_GAIN_PERMILLE = 1000;
const int LEFT_GAIN_PERMILLE = 900;

// --- Датчик звука ---------------------------------------------------------
// Цифровой выход DO модуля (компаратор LM393). GPIO10 не имеет АЦП, поэтому
// уровень громкости отсюда не читается — только "громче/тише порога",
// который задаётся подстроечным резистором на плате датчика.
const int SOUND_PIN = 10;
// Большинство модулей тянут DO в LOW при звуке. Если у твоего наоборот —
// поставь false.
const bool SOUND_ACTIVE_LOW = true;
// Подавление дребезга и многократных срабатываний на одном хлопке.
const uint32_t SOUND_DEBOUNCE_MS = 30;
// Период отправки строки телеметрии.
const uint32_t SOUND_REPORT_PERIOD_MS = 100;

const uint32_t COMMAND_TIMEOUT_MS = 500;

Servo escRight, escLeft;
char rxLine[32];
size_t rxLength = 0;
bool discardLine = false;
bool commandActive = false;
uint32_t lastCommandMs = 0;
uint32_t lastByteMs = 0;

int soundState = 0;
uint16_t soundEvents = 0;
uint32_t lastSoundEdgeMs = 0;
uint32_t lastSoundReportMs = 0;

// Процент -> длительность импульса с учётом мёртвой зоны, поканальной
// добавки, масштаба и направления. Ноль всегда даёт ровную нейтраль.
int pulseFor(int percent, int direction, int trim, int gain) {
  if (percent == 0) return PWM_NEUTRAL;
  const int magnitudePercent = (percent > 0) ? percent : -percent;
  const int span = PWM_MAX_OFFSET - PWM_DEADBAND_US;
  int magnitude = PWM_DEADBAND_US + (magnitudePercent * span) / 100;
  magnitude = magnitude * gain / 1000 + trim;
  if (magnitude > PWM_MAX_OFFSET) magnitude = PWM_MAX_OFFSET;
  if (magnitude < 0) magnitude = 0;
  const int offset = (percent > 0) ? magnitude : -magnitude;
  return PWM_NEUTRAL + direction * offset;
}

void stopMotors() {
  escRight.writeMicroseconds(PWM_NEUTRAL);
  escLeft.writeMicroseconds(PWM_NEUTRAL);
}

// Опрос датчика без блокировки. Смена состояния принимается не чаще
// SOUND_DEBOUNCE_MS, счётчик растёт только по фронту "тишина -> звук".
void pollSound() {
  const uint32_t now = millis();
  const int raw = digitalRead(SOUND_PIN);
  const int active = SOUND_ACTIVE_LOW ? (raw == LOW ? 1 : 0) : (raw == HIGH ? 1 : 0);
  if (active == soundState) return;
  if (static_cast<uint32_t>(now - lastSoundEdgeMs) < SOUND_DEBOUNCE_MS) return;
  lastSoundEdgeMs = now;
  soundState = active;
  if (active) ++soundEvents;
}

// Отправка телеметрии в ту же линию UART. Строка короткая, буфер TX при
// 115200 бод не переполняется, приём команд не задерживается.
void reportSound() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastSoundReportMs) < SOUND_REPORT_PERIOD_MS) return;
  lastSoundReportMs = now;
  char line[24];
  const int written = snprintf(line, sizeof(line), "S:%d|N:%u\n",
                               soundState, static_cast<unsigned>(soundEvents));
  if (written > 0) {
    Serial1.write(reinterpret_cast<const uint8_t *>(line),
                  static_cast<size_t>(written));
  }
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
        escRight.writeMicroseconds(
            pulseFor(right, RIGHT_DIRECTION, RIGHT_TRIM_US, RIGHT_GAIN_PERMILLE));
        escLeft.writeMicroseconds(
            pulseFor(left, LEFT_DIRECTION, LEFT_TRIM_US, LEFT_GAIN_PERMILLE));
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
  // Подтяжка нужна модулям с выходом "открытый коллектор"; на push-pull
  // выходе она безвредна.
  pinMode(SOUND_PIN, SOUND_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  // Без USB CDC объект Serial использует UART0 на тех же GPIO20/21.
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.begin(115200);
  Serial.println("Инициализация ESC в нейтрали, жду 3 секунды...");
#endif
  delay(3000);
  // Начинаем приём после инициализации ESC, без накопления старых команд.
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  lastSoundReportMs = millis();
  lastSoundEdgeMs = millis();
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.println("Готов: UART RX=20, TX=21. Формат: R:90%|L:90% + LF");
  Serial.println("Датчик звука: GPIO10, телеметрия S:<0|1>|N:<счётчик> раз в 100 мс");
#endif
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
  pollSound();
  reportSound();
  delay(1);
}
