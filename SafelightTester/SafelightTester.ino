/*!
 * SafelightTester — тестер безопасного света для Ч/Б печати (строгий режим)
 *
 * Единственный критерий: свет безопасен, если ЕГО ИЗЛУЧЕНИЕ ЦЕЛИКОМ >= 625 нм.
 * Любая обнаружимая энергия ниже 625 нм — провал.
 *
 * Каналы AS7343 (18-канальный режим):
 *   Опасные (абс. порог, далеко от красного): F1 405, F2 425, FZ 450, F3 475, F4 515
 *   Опасные (отн. порог — хвосты фильтров):  F5 550, FY 555, FXL 600
 *   Безопасные (>= 625 нм):                   F6 640, F7 690, F8 745, NIR 855
 *
 * Логика вердикта:
 *   - класс A: значение после вычитания темнового уровня должно быть
 *     в пределах шума (абсолютный порог);
 *   - класс B: допускается только "хвост" от красного света — не более
 *     REL_F5_FY от канала F6 (640 нм); FXL — не более REL_FXL от F6,
 *     иначе свет центрирован ниже 625 нм.
 *
 * Режим "максимально строгий": автоэкспозиция подбирает максимальную
 * чувствительность без пересвета; команда 'l' — прогон на максимальной
 * чувствительности (5 замеров), где виден даже слабый сине-зелёный отсвет.
 *
 * Аппаратура: Arduino Uno + AS7343 (I2C 0x39)
 *   VIN->5V, GND->GND, SDA->A4, SCL->A5
 *   опционально: зелёный светодиод на D8 (PASS), красный на D9 (FAIL)
 *
 * Команды по Serial (115200):
 *   d — записать темновой уровень (СВЕТ ВЫКЛЮЧЕН)
 *   s — одиночный тест (свет включён)
 *   l — длинный тест: 5 замеров на максимальной чувствительности
 *   c — непрерывный тест каждые ~4 с (любая клавиша — стоп)
 *   r — сбросить темновой уровень
 *   h — справка
 */

#include <Adafruit_AS7343.h>
#include <EEPROM.h>

#include <LiquidCrystal.h>

// LCD pins
constexpr uint8_t PIN_RS = 6;
constexpr uint8_t PIN_EN = 7;
constexpr uint8_t PIN_DB4 = 8;
constexpr uint8_t PIN_DB5 = 9;
constexpr uint8_t PIN_DB6 = 10;
constexpr uint8_t PIN_DB7 = 11;

// Button
constexpr uint8_t PIN_BUTTON = 2;

LiquidCrystal lcd(PIN_RS, PIN_EN, PIN_DB4, PIN_DB5, PIN_DB6, PIN_DB7);

enum UiState {
  UI_TURN_OFF_LIGHTS,
  UI_WAIT_BUTTON_DARK,
  UI_TURN_ON_SAFELIGHT,
  UI_WAIT_BUTTON_MEASURE,
  UI_MEASURING,
  UI_RESULT
};

UiState uiState = UI_TURN_OFF_LIGHTS;
unsigned long lastBtnPress = 0;
const unsigned long DEBOUNCE_MS = 200;

uint8_t lastVerdict = 2; // 0=PASS, 1=FAIL, 2=no light, 3=not measured
uint16_t lastDomWl = 0;  // доминирующая длина волны

// ---- Пороги (тут настраивается строгость) ----
static const uint16_t SAT_LEVEL = 64000; // порог пересвета канала
static const int ABS_FLOOR = 25;         // абсолютный порог шума после вычитания темнового, counts
static const float REL_F5_FY = 0.08f;    // F5/FY не выше 8% от F6 (хвост красного)
static const float REL_FXL = 1.5f;       // FXL не выше 150% от F6 (иначе свет <= 620 нм)
static const uint16_t DARK_MAX = 2000;   // проверка: темновой замер не должен быть "светлым"

// ---- Каналы в порядке длин волн ----
struct ChanInfo {
  const char *name;
  uint16_t wl;  // нм
  uint8_t idx;  // индекс в 18-канальном буфере
  bool clsA;    // абсолютный порог
  bool clsB;    // относительный порог
};

static const ChanInfo CH[12] = {
  {"F1 ", 405, AS7343_CHANNEL_F1,  true,  false},
  {"F2 ", 425, AS7343_CHANNEL_F2,  true,  false},
  {"FZ ", 450, AS7343_CHANNEL_FZ,  true,  false},
  {"F3 ", 475, AS7343_CHANNEL_F3,  true,  false},
  {"F4 ", 515, AS7343_CHANNEL_F4,  true,  false},
  {"F5 ", 550, AS7343_CHANNEL_F5,  false, true},
  {"FY ", 555, AS7343_CHANNEL_FY,  false, true},
  {"FXL", 600, AS7343_CHANNEL_FXL, false, true},
  {"F6 ", 640, AS7343_CHANNEL_F6,  false, false},
  {"F7 ", 690, AS7343_CHANNEL_F7,  false, false},
  {"F8 ", 745, AS7343_CHANNEL_F8,  false, false},
  {"NIR", 855, AS7343_CHANNEL_NIR, false, false},
};
enum { C_F4 = 4, C_F5 = 5, C_FY = 6, C_FXL = 7, C_F6 = 8, C_F7 = 9, C_F8 = 10 };

// ---- Кандидаты автоэкспозиции: (ATIME, усиление), от максимума вниз ----
struct Exposure { uint8_t atime; as7343_gain_t gain; };
static const Exposure EXPS[] = {
  {150, AS7343_GAIN_2048X},
  {60,  AS7343_GAIN_2048X},
  {20,  AS7343_GAIN_2048X},
  {5,   AS7343_GAIN_2048X},
  {1,   AS7343_GAIN_2048X},
  {1,   AS7343_GAIN_512X},
  {1,   AS7343_GAIN_128X},
  {1,   AS7343_GAIN_32X},
  {1,   AS7343_GAIN_8X},
  {1,   AS7343_GAIN_2X},
  {1,   AS7343_GAIN_1X},
  {1,   AS7343_GAIN_0_5X},
};
#define N_EXPS (sizeof(EXPS) / sizeof(EXPS[0]))

Adafruit_AS7343 as7343;

// ---- Хранение темнового уровня в EEPROM (переживает сброс/перезагрузку) ----
#define EE_MAGIC 0x53
#define EE_ADDR_MAGIC 0
#define EE_ADDR_VALID 1
#define EE_ADDR_ATIME 2
#define EE_ADDR_GAIN 3
#define EE_ADDR_DARK 4 // 12 каналов x 2 байта

uint16_t readings[18];
uint16_t dark[12];
Exposure darkExp;
bool darkValid = false;
bool continuous = false;

static void saveDarkToEeprom() {
  EEPROM.write(EE_ADDR_MAGIC, EE_MAGIC);
  EEPROM.write(EE_ADDR_VALID, 1);
  EEPROM.write(EE_ADDR_ATIME, darkExp.atime);
  EEPROM.write(EE_ADDR_GAIN, (uint8_t)darkExp.gain);
  for (uint8_t i = 0; i < 12; i++) {
    EEPROM.write(EE_ADDR_DARK + i * 2, lowByte(dark[i]));
    EEPROM.write(EE_ADDR_DARK + i * 2 + 1, highByte(dark[i]));
  }
}

static void loadDarkFromEeprom() {
  if (EEPROM.read(EE_ADDR_MAGIC) != EE_MAGIC) return;
  if (EEPROM.read(EE_ADDR_VALID) != 1) return;
  darkExp.atime = EEPROM.read(EE_ADDR_ATIME);
  darkExp.gain = (as7343_gain_t)EEPROM.read(EE_ADDR_GAIN);
  for (uint8_t i = 0; i < 12; i++) {
    dark[i] = EEPROM.read(EE_ADDR_DARK + i * 2) |
              (EEPROM.read(EE_ADDR_DARK + i * 2 + 1) << 8);
  }
  darkValid = true;
}

static void eraseDarkFromEeprom() {
  EEPROM.write(EE_ADDR_MAGIC, 0);
  EEPROM.write(EE_ADDR_VALID, 0);
}

static float gainMult(as7343_gain_t g) {
  float m = 0.5f;
  for (uint8_t i = 0; i < g; i++) m *= 2.0f;
  return m;
}

static void setExp(const Exposure &e) {
  as7343.setATIME(e.atime);
  as7343.setGain(e.gain);
}

static uint16_t maxSpectral(const uint16_t *buf) {
  uint16_t mx = 0;
  for (uint8_t i = 0; i < 12; i++) {
    uint16_t v = buf[CH[i].idx];
    if (v > mx) mx = v;
  }
  return mx;
}

// Подбор экспозиции: максимальная чувствительность без пересвета
static void autoExpose(Exposure &out, bool &overSat, bool &noLight) {
  overSat = false;
  noLight = false;
  for (uint8_t i = 0; i < N_EXPS; i++) {
    setExp(EXPS[i]);
    if (!as7343.readAllChannels(readings)) continue;
    uint16_t mx = maxSpectral(readings);
    if (as7343.isDigitalSaturated() || mx >= SAT_LEVEL) continue;
    out = EXPS[i];
    if (i == 0 && mx < 150) noLight = true;
    return;
  }
  overSat = true;
  setExp(EXPS[N_EXPS - 1]);
  as7343.readAllChannels(readings);
  out = EXPS[N_EXPS - 1];
}

// Масштаб темнового уровня под другую экспозицию (ток тёмный ~ t_int * gain)
static float expFactor(const Exposure &test, const Exposure &base) {
  return ((float)(test.atime + 1) / (float)(base.atime + 1)) *
         (gainMult(test.gain) / gainMult(base.gain));
}

static void captureDark() {
  Exposure e;
  bool overSat, noLight;
  autoExpose(e, overSat, noLight);
  if (maxSpectral(readings) > DARK_MAX) {
    Serial.println(F("ВНИМАНИЕ: для темнового замера СВЕТ ДОЛЖЕН БЫТЬ ВЫКЛЮЧЕН!"));
    Serial.println(F("Замер записан, но он подозрительно яркий."));
  }
  darkExp = e;
  for (uint8_t i = 0; i < 12; i++) dark[i] = readings[CH[i].idx];
  darkValid = true;
  saveDarkToEeprom();
  Serial.print(F("Темновой уровень записан (ATIME="));
  Serial.print(e.atime);
  Serial.print(F(", усиление="));
  Serial.print(gainMult(e.gain));
  Serial.println(F("x), сохранён в EEPROM."));
}

static void printTable(const int32_t *sig, const int32_t *th, const int32_t *raw) {
  Serial.println(F("  канал  нм   raw     сигнал   порог  стат."));
  for (uint8_t i = 0; i < 12; i++) {
    char stat[8] = "   ";
    if (CH[i].clsA || CH[i].clsB) stat[0] = (sig[i] > th[i]) ? '!' : 'o';
    char buf[64];
    snprintf(buf, sizeof(buf), "  %s %4u %6ld %8ld %6ld  [%s]",
             CH[i].name, CH[i].wl, (long)raw[CH[i].idx], (long)sig[i], (long)th[i], stat);
    Serial.println(buf);
  }
}

// ВЕРДИКТ: 0=PASS, 1=FAIL, 2=нет света

// Один тест: измерение + анализ + отчёт. avgReads>1 — усреднение по нескольким
// замерам (длинный режим 'l').
static void runTest(uint8_t avgReads) {
  if (!darkValid) {
    Serial.println(F("НЕТ ТЕМНОВОГО УРОВНЯ: выключите свет и нажмите 'd'."));
    lastVerdict = 3;
    return;
  }

  Exposure e;
  bool overSat, noLight;
  autoExpose(e, overSat, noLight);

  // усреднение
  uint32_t acc[12] = {0};
  for (uint8_t r = 0; r < avgReads; r++) {
    if (r > 0) as7343.readAllChannels(readings);
    for (uint8_t i = 0; i < 12; i++) acc[i] += readings[CH[i].idx];
  }
  int32_t ch[12];
  for (uint8_t i = 0; i < 12; i++) ch[i] = (int32_t)((acc[i] + avgReads / 2) / avgReads);

  Serial.println(F("=========================================="));
  Serial.print(F("Экспозиция: ATIME="));
  Serial.print(e.atime);
  Serial.print(F(", усиление="));
  Serial.print(gainMult(e.gain));
  Serial.print(F("x, время цикла="));
  Serial.print(as7343.getIntegrationTime());
  Serial.println(F(" мс"));

  float k = expFactor(e, darkExp);
  int32_t sig[12], th[12];
  int32_t f6 = 0;
  for (uint8_t i = 0; i < 12; i++) {
    int32_t d = (int32_t)((float)dark[i] * k);
    sig[i] = ch[i] - d;
    if (sig[i] < 0) sig[i] = 0;
    th[i] = 3 * ABS_FLOOR + 3 * d;
  }
  f6 = sig[C_F6];
  for (uint8_t i = 0; i < 12; i++) {
    if (CH[i].clsB) {
      int32_t rel = (CH[i].name[0] == 'F' && CH[i].wl <= 555)
                        ? (int32_t)((float)f6 * REL_F5_FY)
                        : (int32_t)((float)f6 * REL_FXL);
      if (rel > th[i]) th[i] = rel;
    }
  }

  printTable(sig, th, ch);

  // доминирующая длина волны (без NIR — он всегда высокий)
  uint8_t dom = 0;
  for (uint8_t i = 1; i < 11; i++)
    if (sig[i] > sig[dom]) dom = i;
  Serial.print(F("Доминирующая полоса: "));
  Serial.print(CH[dom].name);
  Serial.print(F(" ("));
  Serial.print(CH[dom].wl);
  Serial.println(F(" нм)"));

  // отношения каналов — для оценки «хвоста» ниже 625 нм
  if (f6 > 0) {
    Serial.print(F("Отношения: FXL/F6="));
    Serial.print((float)sig[C_FXL] / f6, 2);
    Serial.print(F(", FY/F6="));
    Serial.print((float)sig[C_FY] / f6, 2);
    Serial.print(F(", F5/F6="));
    Serial.println((float)sig[C_F5] / f6, 3);
    long redBand = sig[C_F6] + sig[C_F7] + sig[C_F8];
    long below = sig[C_FXL] + sig[C_FY] + sig[C_F5];
    if (redBand + below > 0) {
      Serial.print(F("Доля света ниже 625 нм (оценка): "));
      Serial.print(100.0f * below / (redBand + below), 1);
      Serial.println(F("%"));
    }
  }

  if (noLight) {
    Serial.println(F("ВЕРДИКТ: СВЕТА НЕ ОБНАРУЖЕНО (сенсор накрыт или свет выключен)"));
    lastVerdict = 2;
    return;
  }
  if (overSat) {
    Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: пересвет даже на минимальной экспозиции, "));
    Serial.println(F("  отнесите сенсор дальше от источника."));
  }
  if (sig[C_F6] >= 65500) {
    Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: F6 (640 нм) пересвет — относительные пороги завышены."));
  }

  bool fail = false;
  uint8_t bad = 0;
  for (uint8_t i = 0; i < 12; i++) {
    if (!CH[i].clsA && !CH[i].clsB) continue;
    if (sig[i] > th[i]) { fail = true; bad = i; }
  }

  if (!fail) {
    Serial.println(F("ВЕРДИКТ: БЕЗОПАСНО [строгий режим] — излучение >= 625 нм,"));
    Serial.println(F("  сине-зелёного света ниже порога не обнаружено."));
    lastVerdict = 0;
    lastDomWl = CH[dom].wl;
    return;
  }

  Serial.print(F("ВЕРДИКТ: ОПАСНО — обнаружен свет ниже 625 нм (канал "));
  Serial.print(CH[bad].name);
  Serial.print(F(" "));
  Serial.print(CH[bad].wl);
  Serial.println(F(" нм)."));
  Serial.println(F("  Такой свет засвечивает Ч/Б бумагу. Устраните утечку или смените фильтр."));
  lastVerdict = 1;
  lastDomWl = CH[dom].wl;
}

static void runLongTest() {
  Serial.println(F("Длинный тест: 5 замеров на максимальной чувствительности..."));
  runTest(5);
}

static void printHelp() {
  Serial.println(F("Команды:"));
  Serial.println(F("  d - темновой уровень (свет выключен!)"));
  Serial.println(F("  s - одиночный тест"));
  Serial.println(F("  l - длинный тест (5 замеров, макс. чувствительность)"));
  Serial.println(F("  c - непрерывный тест (стоп - любая клавиша)"));
  Serial.println(F("  r - сбросить темновой уровень"));
  Serial.println(F("  h - эта справка"));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("=== SafelightTester: тестер безопасного света (Ч/Б печать) ==="));
  Serial.println(F("Единственный критерий: весь свет >= 625 нм."));
  Serial.println(F("Подключение: VIN=5V, GND=GND, SDA=A4, SCL=A5"));

  if (!as7343.begin()) {
    Serial.println(F("ОШИБКА: AS7343 не найден. Проверьте I2C-подключение."));
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("AS7343 ERROR");
    while (1) { delay(100); }
  }
  Serial.print(F("AS7343 найден. ID="));
  Serial.print(as7343.getPartID(), HEX);
  Serial.print(F(" REV="));
  Serial.println(as7343.getRevisionID());

  as7343.setASTEP(599); // фиксированный шаг интеграции, ATIME меняет время
  as7343.setSMUXMode(AS7343_SMUX_18CH);

  // LCD init
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Turn off all");
  lcd.setCursor(0, 1);
  lcd.print("lights");

  loadDarkFromEeprom();
  if (darkValid) {
    Serial.println(F("Темновой уровень восстановлен из EEPROM."));
  } else {
    Serial.println(F("Темновой уровень не сохранён — снимите его командой 'd'."));
  }

  printHelp();
  Serial.println(F("Порядок работы: 1) выключить свет в комнате, нажать 'd';"));
  Serial.println(F("                2) включить фонарь/лампу, нажать 's' (или 'l')."));
  Serial.println(F("Сенсор держите на уровне бумаги на столе, окном вверх."));
}

void loop() {
  // Button handling for standalone UI
  bool btnPressed = (digitalRead(PIN_BUTTON) == LOW);
  unsigned long now = millis();
  if (btnPressed && (now - lastBtnPress > DEBOUNCE_MS)) {
    lastBtnPress = now;
    handleUiButton();
  }

  if (uiState == UI_MEASURING) {
    runTest(1);
    uiState = UI_RESULT;
    showUiResult();
  }

  // Serial commands (still work for debugging)
  if (Serial.available()) {
    char c = Serial.read();
    continuous = false;
    switch (c) {
      case 'd': captureDark(); break;
      case 's': runTest(1); break;
      case 'l': runLongTest(); break;
      case 'c': continuous = true; Serial.println(F("Непрерывный режим (стоп - любая клавиша)")); break;
      case 'r': darkValid = false; eraseDarkFromEeprom(); Serial.println(F("Темновой уровень сброшен.")); break;
      case 'h': printHelp(); break;
      default: break;
    }
  }
  if (continuous) {
    runTest(1);
    for (uint8_t i = 0; i < 40; i++) { // ~4 с на шаг
      delay(100);
      if (Serial.available()) { continuous = false; break; }
    }
  } else {
    delay(50);
  }
}

static void handleUiButton() {
  switch (uiState) {
    case UI_TURN_OFF_LIGHTS:
    case UI_WAIT_BUTTON_DARK:
      if (!darkValid) {
        captureDark();
      }
      uiState = UI_TURN_ON_SAFELIGHT;
      lcd.clear();
      lcd.print("Turn on");
      lcd.setCursor(0, 1);
      lcd.print("safelight");
      break;

    case UI_TURN_ON_SAFELIGHT:
    case UI_WAIT_BUTTON_MEASURE:
      uiState = UI_MEASURING;
      lcd.clear();
      lcd.print("Measuring...");
      break;

    case UI_RESULT:
      uiState = UI_TURN_OFF_LIGHTS;
      lcd.clear();
      lcd.print("Turn off all");
      lcd.setCursor(0, 1);
      lcd.print("lights");
      break;
  }
}

static void showUiResult() {
  lcd.clear();
  switch (lastVerdict) {
    case 0:
      lcd.print("SAFE");
      lcd.setCursor(0, 1);
      lcd.print(lastDomWl);
      lcd.print("nm OK");
      break;
    case 1:
      lcd.print("UNSAFE!");
      lcd.setCursor(0, 1);
      lcd.print(lastDomWl);
      lcd.print("nm");
      break;
    case 2:
      lcd.print("No light");
      lcd.setCursor(0, 1);
      lcd.print("detected");
      break;
    case 3:
      lcd.print("No dark level");
      lcd.setCursor(0, 1);
      lcd.print("Calibrate first");
      break;
  }
  delay(3000);
  lcd.clear();
  lcd.print("Again? Press btn");
}
