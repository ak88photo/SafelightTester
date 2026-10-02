// Проверка логики вердикта на компьютере, без платы:
//   c++ -std=c++11 -Wall -I SafelightTester test/verdict_test.cpp -o /tmp/verdict_test && /tmp/verdict_test
#include <stdio.h>
#include "verdict.h"

static int failures = 0;

static const char *NAMES[V_N] = {"F1", "F2", "FZ", "F3", "F4", "F5",
                                 "FY", "FXL", "F6", "F7", "F8", "NIR"};

static void expect(const char *name, const int32_t *sig, const int32_t *d,
                   uint8_t verdict, uint8_t worst) {
  VerdictOut o;
  evaluateVerdict(sig, d, o);
  const bool ok = o.verdict == verdict && o.worst == worst;
  printf("%s  %-48s verdict=%u worst=%s\n", ok ? "ok  " : "FAIL", name, o.verdict,
         o.worst < V_N ? NAMES[o.worst] : "-");
  if (!ok) {
    printf("      ожидалось verdict=%u worst=%s\n", verdict, worst < V_N ? NAMES[worst] : "-");
    failures++;
  }
}

int main() {
  // Реальный замер 02.10.2026: Cree XM-L, только красный кристалл, 2 м,
  // прозрачный печатный поликарбонат. ATIME=150, 2048x. Сигнал после темнового.
  //                       F1   F2   FZ  F3   F4  F5    FY   FXL     F6   F7  F8  NIR
  const int32_t red[V_N] = {69, 114, 101, 56, 141, 32, 4168, 9024, 10379, 238, 28, 155};
  const int32_t dark[V_N] = {3, 2, 2, 2, 2, 2, 2, 2, 3, 5, 2, 5};
  expect("красный XM-L (реальный замер)", red, dark, VERDICT_SAFE, V_N);

  // Тот же свет вдвое тусклее — утечка падает вместе с F6
  int32_t half[V_N];
  for (int i = 0; i < V_N; i++) half[i] = red[i] / 2;
  expect("красный XM-L, вдвое тусклее", half, dark, VERDICT_SAFE, V_N);

  // Красный + 5% зелёного (например, недогашенный зелёный кристалл)
  int32_t green[V_N];
  for (int i = 0; i < V_N; i++) green[i] = red[i];
  green[V_F4] += 520;
  green[V_F5] += 300;
  expect("красный + зелёная примесь", green, dark, VERDICT_UNSAFE, V_F5);

  // Красный + слабый синий
  int32_t blue[V_N];
  for (int i = 0; i < V_N; i++) blue[i] = red[i];
  blue[V_FZ] += 400;
  expect("красный + синяя примесь", blue, dark, VERDICT_UNSAFE, V_FZ);

  // Чистый синий светодиод: красного нет, ловит абсолютный пол шума
  const int32_t blueLed[V_N] = {900, 6000, 20000, 9000, 300, 40, 600, 80, 30, 10, 5, 20};
  expect("синий светодиод", blueLed, dark, VERDICT_UNSAFE, V_FZ);

  // Оранжево-красный ~610 нм: FXL/F6 = 1.4
  const int32_t orangeRed[V_N] = {60, 90, 80, 50, 120, 30, 5000, 8400, 6000, 120, 20, 100};
  expect("оранжево-красный ~610 нм", orangeRed, dark, VERDICT_BORDERLINE, V_FXL);

  // Янтарный ~590 нм: FXL/F6 = 4, F5 видит хвост
  const int32_t amber[V_N] = {40, 50, 40, 30, 60, 30, 9000, 12000, 3000, 60, 10, 60};
  expect("янтарный ~590 нм", amber, dark, VERDICT_UNSAFE, V_FXL);

  // Фонарь, где FY заметно выше, чем у красного (жёлто-оранжевая часть)
  int32_t yellowish[V_N];
  for (int i = 0; i < V_N; i++) yellowish[i] = red[i];
  yellowish[V_FY] = 9000;
  expect("красный + жёлто-оранжевая часть (FY/F6=0.87)", yellowish, dark,
         VERDICT_BORDERLINE, V_FY);

  // Темнота / шум — ничего не превышает пол
  const int32_t noise[V_N] = {10, 12, 8, 9, 11, 7, 10, 12, 9, 8, 6, 10};
  expect("только шум", noise, dark, VERDICT_SAFE, V_N);

  printf(failures ? "\nПРОВАЛЕНО: %d\n" : "\nВсе проверки прошли\n", failures);
  return failures ? 1 : 0;
}
