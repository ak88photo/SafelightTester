// verdict.h — логика вердикта SafelightTester без зависимостей от Arduino,
// чтобы её можно было проверять на компьютере (см. test/verdict_test.cpp).
//
// Все "опасные" каналы сравниваются с красным F6 (640 нм), а не в абсолютных
// отсчётах: утечка фильтров AS7343 растёт вместе с яркостью красного, и
// абсолютный порог на максимальной чувствительности ловил её как "синий свет".
// Абсолютный пол шума остаётся — он ловит свет вообще без красного.
#pragma once
#include <stdint.h>

// Порядок каналов — как в CH[] скетча (по длине волны)
enum { V_F1, V_F2, V_FZ, V_F3, V_F4, V_F5, V_FY, V_FXL, V_F6, V_F7, V_F8, V_NIR, V_N };

enum : uint8_t {
  VERDICT_SAFE = 0,
  VERDICT_UNSAFE = 1,
  VERDICT_NO_LIGHT = 2,
  VERDICT_NOT_MEASURED = 3,
  VERDICT_BORDERLINE = 4,
  VERDICT_TOO_BRIGHT = 5,
};

// Уровень канала
enum : uint8_t { LVL_OK = 0, LVL_BORDER = 1, LVL_UNSAFE = 2, LVL_NONE = 255 };

// ---- Пороги (тут настраивается строгость) ----
static const int VERDICT_ABS_FLOOR = 25; // пол шума после вычитания темнового, counts

// Узкие сине-зелёные каналы F1 F2 FZ F3 F4 F5: допустимая доля от F6.
// Откалибровано по замеру 02.10.2026: Cree XM-L RGBW, горит только красный
// кристалл (~620–630 нм). Утечка красного: F1 .007, F2 .011, FZ .010,
// F3 .005, F4 .014, F5 .003. Порог — примерно вдвое выше утечки.
// Превышение = реальный синий/зелёный свет -> ОПАСНО.
static const float LEAK_LIMIT[6] = {0.015f, 0.025f, 0.025f, 0.012f, 0.03f, 0.01f};

// Широкие каналы FY (555) и FXL (600) захватывают плечо красного пика.
// У того же светодиода: FY/F6 = 0.40, FXL/F6 = 0.87.
static const float FY_BORDER = 0.8f;   // выше — заметная жёлто-оранжевая часть
static const float FXL_BORDER = 1.1f;  // выше — пик сдвинут к ~605–620 нм
static const float FXL_UNSAFE = 2.0f;  // выше — оранжевый/янтарный свет

struct VerdictOut {
  uint8_t verdict;     // VERDICT_SAFE / VERDICT_BORDERLINE / VERDICT_UNSAFE
  uint8_t worst;       // канал, определивший вердикт (V_*), V_N если все в норме
  int32_t th[V_N];     // порог для печати (граница ПОГРАНИЧНО для FY/FXL)
  uint8_t level[V_N];  // LVL_* по каждому каналу
};

static inline int32_t vMax(int32_t a, int32_t b) { return a > b ? a : b; }

// sig — сигнал после вычитания темнового (>= 0), d — темновой уровень,
// приведённый к экспозиции замера.
static inline void evaluateVerdict(const int32_t *sig, const int32_t *d, VerdictOut &o) {
  const float f6 = (float)sig[V_F6];
  o.verdict = VERDICT_SAFE;
  o.worst = V_N;
  float worstExcess = 0.0f;

  for (uint8_t i = 0; i < V_N; i++) {
    const int32_t noise = 3 * VERDICT_ABS_FLOOR + 3 * d[i];
    uint8_t lvl = LVL_NONE;
    int32_t th = noise;

    if (i <= V_F5) {
      th = vMax(noise, (int32_t)(f6 * LEAK_LIMIT[i]));
      lvl = sig[i] > th ? LVL_UNSAFE : LVL_OK;
    } else if (i == V_FY) {
      th = vMax(noise, (int32_t)(f6 * FY_BORDER));
      lvl = sig[i] > th ? LVL_BORDER : LVL_OK;
    } else if (i == V_FXL) {
      th = vMax(noise, (int32_t)(f6 * FXL_BORDER));
      const int32_t thUnsafe = vMax(noise, (int32_t)(f6 * FXL_UNSAFE));
      lvl = sig[i] > thUnsafe ? LVL_UNSAFE : (sig[i] > th ? LVL_BORDER : LVL_OK);
    }

    o.th[i] = th;
    o.level[i] = lvl;
    if (lvl == LVL_NONE || lvl == LVL_OK) continue;

    const uint8_t v = (lvl == LVL_UNSAFE) ? VERDICT_UNSAFE : VERDICT_BORDERLINE;
    const float excess = (float)sig[i] / (float)(th > 0 ? th : 1);
    const bool worse = (v == VERDICT_UNSAFE && o.verdict != VERDICT_UNSAFE) ||
                       (v == o.verdict && excess > worstExcess) ||
                       (o.verdict == VERDICT_SAFE);
    if (worse) {
      o.verdict = v;
      o.worst = i;
      worstExcess = excess;
    }
  }
}
