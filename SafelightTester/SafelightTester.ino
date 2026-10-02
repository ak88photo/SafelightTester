/*!
 * SafelightTester — тестер безопасного света для Ч/Б печати (в т.ч. лит)
 *
 * Критерий: в свете не должно быть синего/зелёного, а красный пик не должен
 * уходить в оранжевый. Пороги и их калибровка — в verdict.h.
 *
 * Каналы AS7343 (18-канальный режим):
 *   Узкие сине-зелёные: F1 405, F2 425, FZ 450, F3 475, F4 515, F5 550
 *     -> порог = доля от F6 (утечка фильтров от красного), но не ниже шума;
 *        превышение = ОПАСНО.
 *   Широкие: FY 555, FXL 600 — захватывают плечо красного пика
 *     -> по отношению к F6: ПОГРАНИЧНО (пик ~605–620 нм) или ОПАСНО (оранжевый).
 *   Красные: F6 640, F7 690, F8 745, NIR 855 — в вердикте не участвуют.
 *
 * Вердикты: БЕЗОПАСНО / ПОГРАНИЧНО / ОПАСНО / свет не обнаружен / пересвет.
 * Спектр — не доза: время безопасной работы решает только бумажный тест.
 *
 * Аппаратура: Arduino Uno + AS7343 (I2C 0x39)
 *   VIN->5V, GND->GND, SDA->A4, SCL->A5
 *   LCD 16x2: RS=6, EN=7, DB4..DB7=8..11; кнопка на D2 (на GND)
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

#include "verdict.h"

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

uint8_t lastVerdict = VERDICT_NOT_MEASURED; // VERDICT_* из verdict.h
uint8_t lastWorst = V_N;  // канал, определивший вердикт
uint16_t lastFxlF6 = 0;   // FXL/F6 x100 — где сидит красный пик

// ---- Служебные пороги (пороги вердикта — в verdict.h) ----
static const uint16_t SAT_LEVEL = 64000; // порог пересвета канала
static const uint16_t DARK_MAX = 2000;   // проверка: темновой замер не должен быть "светлым"

// ---- Каналы в порядке длин волн ----
struct ChanInfo {
  const char *name;
  uint16_t wl;  // нм
  uint8_t idx;  // индекс в 18-канальном буфере
};

static const ChanInfo CH[12] = {
  {"F1 ", 405, AS7343_CHANNEL_F1},
  {"F2 ", 425, AS7343_CHANNEL_F2},
  {"FZ ", 450, AS7343_CHANNEL_FZ},
  {"F3 ", 475, AS7343_CHANNEL_F3},
  {"F4 ", 515, AS7343_CHANNEL_F4},
  {"F5 ", 550, AS7343_CHANNEL_F5},
  {"FY ", 555, AS7343_CHANNEL_FY},
  {"FXL", 600, AS7343_CHANNEL_FXL},
  {"F6 ", 640, AS7343_CHANNEL_F6},
  {"F7 ", 690, AS7343_CHANNEL_F7},
  {"F8 ", 745, AS7343_CHANNEL_F8},
  {"NIR", 855, AS7343_CHANNEL_NIR},
};

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

static void printTable(const int32_t *sig, const VerdictOut &o, const int32_t *raw) {
  Serial.println(F("  канал  нм   raw     сигнал   порог  стат."));
  for (uint8_t i = 0; i < 12; i++) {
    char stat[8] = "   ";
    if (o.level[i] == LVL_OK) stat[0] = 'o';
    else if (o.level[i] == LVL_BORDER) stat[0] = '?';
    else if (o.level[i] == LVL_UNSAFE) stat[0] = '!';
    char buf[64];
    snprintf(buf, sizeof(buf), "  %s %4u %6ld %8ld %6ld  [%s]",
             CH[i].name, CH[i].wl, (long)raw[i], (long)sig[i], (long)o.th[i], stat);
    Serial.println(buf);
  }
}

static void printWorst(uint8_t w) {
  Serial.print(F(" (канал "));
  Serial.print(CH[w].name);
  Serial.print(F(" "));
  Serial.print(CH[w].wl);
  Serial.println(F(" нм)."));
}

// Один тест: измерение + анализ + отчёт. avgReads>1 — усреднение по нескольким
// замерам (длинный режим 'l').
static void runTest(uint8_t avgReads) {
  lastWorst = V_N;
  if (!darkValid) {
    Serial.println(F("НЕТ ТЕМНОВОГО УРОВНЯ: выключите свет и нажмите 'd'."));
    lastVerdict = VERDICT_NOT_MEASURED;
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
  int32_t sig[12], d[12];
  for (uint8_t i = 0; i < 12; i++) {
    d[i] = (int32_t)((float)dark[i] * k);
    sig[i] = ch[i] - d[i];
    if (sig[i] < 0) sig[i] = 0;
  }

  VerdictOut o;
  evaluateVerdict(sig, d, o);
  printTable(sig, o, ch);

  // где сидит красный пик: у честного красного ~620–630 нм FXL/F6 ~0.9
  const int32_t f6 = sig[V_F6];
  lastFxlF6 = 0;
  if (f6 > 0 && !noLight) {
    lastFxlF6 = (uint16_t)(100.0f * sig[V_FXL] / f6 + 0.5f);
    Serial.print(F("Отношения к F6: FXL="));
    Serial.print((float)sig[V_FXL] / f6, 2);
    Serial.print(F(" (порог "));
    Serial.print(FXL_BORDER, 2);
    Serial.print(F("), FY="));
    Serial.print((float)sig[V_FY] / f6, 2);
    Serial.print(F(" (порог "));
    Serial.print(FY_BORDER, 2);
    Serial.print(F("), F5="));
    Serial.print((float)sig[V_F5] / f6, 3);
    Serial.print(F(" (порог "));
    Serial.print(LEAK_LIMIT[V_F5], 3);
    Serial.println(F(")"));
  }

  if (noLight) {
    Serial.println(F("ВЕРДИКТ: СВЕТА НЕ ОБНАРУЖЕНО (сенсор накрыт или свет выключен)"));
    lastVerdict = VERDICT_NO_LIGHT;
    return;
  }
  // При пересвете F6 занижен, а пороги, привязанные к нему, — тоже:
  // вердикт ненадёжен, БЕЗОПАСНО не выдаём.
  if (overSat || f6 >= SAT_LEVEL) {
    Serial.println(F("ВЕРДИКТ: ПЕРЕСВЕТ — даже на минимальной экспозиции."));
    Serial.println(F("  Отнесите сенсор дальше от источника и повторите."));
    lastVerdict = VERDICT_TOO_BRIGHT;
    return;
  }

  lastVerdict = o.verdict;
  lastWorst = o.worst;
  switch (o.verdict) {
    case VERDICT_SAFE:
      Serial.println(F("ВЕРДИКТ: БЕЗОПАСНО — синего/зелёного не обнаружено,"));
      Serial.println(F("  красный пик не сдвинут в оранжевый."));
      Serial.println(F("  Допустимое время под светом — только по бумажному тесту."));
      break;
    case VERDICT_BORDERLINE:
      Serial.print(F("ВЕРДИКТ: ПОГРАНИЧНО — пик сдвинут к оранжевому"));
      printWorst(o.worst);
      Serial.println(F("  Для лита и долгой работы — проверить бумажным тестом, снизить яркость."));
      break;
    default:
      Serial.print(F("ВЕРДИКТ: ОПАСНО — обнаружен синий/зелёный/оранжевый свет"));
      printWorst(o.worst);
      Serial.println(F("  Такой свет засвечивает Ч/Б бумагу. Устраните утечку или смените фильтр."));
      break;
  }
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
  Serial.println(F("Критерий: без синего/зелёного, красный пик не в оранжевом."));
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
      // всегда свежий темновой: сохранённый в EEPROM мог устареть
      captureDark();
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

// Строка "FY  555": канал, определивший вердикт
static void lcdPrintWorst() {
  if (lastWorst >= V_N) return;
  lcd.print(' ');
  lcd.print(CH[lastWorst].name);
  lcd.print(' ');
  lcd.print(CH[lastWorst].wl);
}

static void lcdPrintFxlF6() {
  lcd.print("FXL/F6 ");
  lcd.print(lastFxlF6 / 100.0f, 2);
}

static void showUiResult() {
  lcd.clear();
  switch (lastVerdict) {
    case VERDICT_SAFE:
      lcd.print("SAFE");
      lcd.setCursor(0, 1);
      lcdPrintFxlF6();
      break;
    case VERDICT_BORDERLINE:
      lcd.print("BORDER");
      lcdPrintWorst();
      lcd.setCursor(0, 1);
      lcdPrintFxlF6();
      break;
    case VERDICT_UNSAFE:
      lcd.print("UNSAFE!");
      lcdPrintWorst();
      lcd.setCursor(0, 1);
      lcdPrintFxlF6();
      break;
    case VERDICT_NO_LIGHT:
      lcd.print("No light");
      lcd.setCursor(0, 1);
      lcd.print("detected");
      break;
    case VERDICT_TOO_BRIGHT:
      lcd.print("Too bright");
      lcd.setCursor(0, 1);
      lcd.print("Move sensor away");
      break;
    case VERDICT_NOT_MEASURED:
      lcd.print("No dark level");
      lcd.setCursor(0, 1);
      lcd.print("Calibrate first");
      break;
  }
  delay(3000);
  lcd.clear();
  lcd.print("Again? Press btn");
}
