 /*
 * Автономный тест двух моторов: ESP32-C3 Super Mini + ESP32Servo.
 * Откройте motor_test.ino в Arduino IDE и загрузите вместо esp32_code.
 * UART и Raspberry Pi для этого теста не нужны.
 *
 * После включения: 3 секунды нейтрали для ESC, затем постоянное движение.
 * LEFT_SPEED_PERCENT и RIGHT_SPEED_PERCENT задают стартовые скорости.
 * Serial Monitor: 115200 бод, Newline или Both NL & CR.
 * Для USB на ESP32-C3 включите USB CDC On Boot = Enabled.
 * Команды: 50 (оба мотора), L:50 (левый), R:45 (правый).
 * Проценты можно писать с символом %: L:50%. Команда 0 = стоп обоих.
 * Диапазон -100..100: плюс = вперёд, минус = назад, 0 = стоп.
 * Примеры (левый, правый): (90, 90) вперёд; (-90, -90) назад;
 * (90, -90) разворот; (0, 0) стоп. Для прямого хода подберите значения
 * отдельно: одинаковая команда ESC не гарантирует одинаковые обороты.
 * Движение продолжается до отключения питания или перепрошивки.
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ===== Настройки теста: скорости обоих моторов =====
constexpr int LEFT_SPEED_PERCENT = 50;
constexpr int RIGHT_SPEED_PERCENT = 50;

static_assert(LEFT_SPEED_PERCENT >= -100 && LEFT_SPEED_PERCENT <= 100,
              "LEFT_SPEED_PERCENT must be within -100..100");
static_assert(RIGHT_SPEED_PERCENT >= -100 && RIGHT_SPEED_PERCENT <= 100,
              "RIGHT_SPEED_PERCENT must be within -100..100");

// Подключение и масштаб команды совпадают с esp32_code.ino.
constexpr int ESC_RIGHT_PIN = 4;
constexpr int ESC_LEFT_PIN = 5;
constexpr int RIGHT_DIRECTION = -1;
constexpr int LEFT_DIRECTION = 1;
constexpr int PWM_MIN = 1000;
constexpr int PWM_NEUTRAL = 1500;
constexpr int PWM_MAX = 2000;
// 90% соответствует отклонению на 50 мкс, а не 90% диапазона PWM/RPM.
constexpr int PWM_REFERENCE_PERCENT = 90;
constexpr int PWM_REFERENCE_OFFSET = 50;
constexpr uint32_t ARMING_DELAY_MS = 3000;

Servo escRight, escLeft;
int leftPercent = LEFT_SPEED_PERCENT;
int rightPercent = RIGHT_SPEED_PERCENT;
int leftPulseUs = PWM_NEUTRAL;
int rightPulseUs = PWM_NEUTRAL;
char commandLine[32];
size_t commandLength = 0;
bool discardCommand = false;
uint32_t lastReportMs = 0;

void printStatus() {
  Serial.print("L: ");
  Serial.print(leftPercent);
  Serial.print("% -> ");
  Serial.print(leftPulseUs);
  Serial.print(" us | R: ");
  Serial.print(rightPercent);
  Serial.print("% -> ");
  Serial.print(rightPulseUs);
  Serial.println(" us");
  lastReportMs = millis();
}

void applySpeeds() {
  leftPulseUs = PWM_NEUTRAL + LEFT_DIRECTION * leftPercent *
                PWM_REFERENCE_OFFSET / PWM_REFERENCE_PERCENT;
  rightPulseUs = PWM_NEUTRAL + RIGHT_DIRECTION * rightPercent *
                 PWM_REFERENCE_OFFSET / PWM_REFERENCE_PERCENT;
  escLeft.writeMicroseconds(leftPulseUs);
  escRight.writeMicroseconds(rightPulseUs);
  printStatus();
}

bool parseInput(const char *cursor) {
  while (*cursor == ' ') ++cursor;
  char motor = 'B';
  if (*cursor == 'L' || *cursor == 'l' || *cursor == 'R' || *cursor == 'r') {
    motor = (*cursor == 'L' || *cursor == 'l') ? 'L' : 'R';
    ++cursor;
    if (*cursor++ != ':') return false;
  }
  while (*cursor == ' ') ++cursor;
  int sign = 1;
  if (*cursor == '-' || *cursor == '+') {
    if (*cursor == '-') sign = -1;
    ++cursor;
  }
  if (*cursor < '0' || *cursor > '9') return false;
  int value = 0;
  while (*cursor >= '0' && *cursor <= '9') {
    value = value * 10 + (*cursor++ - '0');
    if (value > 100) return false;
  }
  if (*cursor == '%') ++cursor;
  while (*cursor == ' ') ++cursor;
  if (*cursor != '\0') return false;
  if (motor != 'R') leftPercent = sign * value;
  if (motor != 'L') rightPercent = sign * value;
  applySpeeds();
  return true;
}

void receiveCommand(char ch) {
  if (ch == '\r' || ch == '\n') {
    if (discardCommand) {
      Serial.println("Ошибка: слишком длинная или повреждённая команда.");
    } else if (commandLength > 0) {
      commandLine[commandLength] = '\0';
      if (!parseInput(commandLine)) {
        Serial.println("Ошибка: введите -100..100, L:50 или R:50.");
      }
    }
    commandLength = 0;
    discardCommand = false;
    return;
  }
  if (discardCommand) return;
  if (commandLength >= sizeof(commandLine) - 1 || ch < ' ' || ch > '~') {
    discardCommand = true;
    return;
  }
  commandLine[commandLength++] = ch;
}

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  escRight.setPeriodHertz(50);
  escLeft.setPeriodHertz(50);
  escRight.attach(ESC_RIGHT_PIN, PWM_MIN, PWM_MAX);
  escLeft.attach(ESC_LEFT_PIN, PWM_MIN, PWM_MAX);

  escRight.writeMicroseconds(PWM_NEUTRAL);
  escLeft.writeMicroseconds(PWM_NEUTRAL);
  Serial.println("Инициализация ESC: L=1500 us, R=1500 us, жду 3 секунды...");
  delay(ARMING_DELAY_MS);

  Serial.println("Команды + Enter: 50 = оба, L:50 = левый, R:45 = правый, 0 = стоп.");
  applySpeeds();
}

void loop() {
  for (size_t count = 0; count < 64 && Serial.available() > 0; ++count) {
    const int value = Serial.read();
    if (value < 0) break;
    receiveCommand(static_cast<char>(value));
  }
  // Выводятся заданные импульсы ESC, а не измеренные обороты моторов.
  if (static_cast<uint32_t>(millis() - lastReportMs) >= 1000) printStatus();
  delay(1);
}
