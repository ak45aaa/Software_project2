/*
 * 20C10 - Infrared sensor automatic curve fitting
 *
 * 20_example_1: A0 입력, 반복 측정, 정렬 후 중앙값 선택
 * 20_example_2: ADC 값을 mm 단위 거리로 변환하는 volt_to_distance()
 *
 * 사용법
 * 1. 시리얼 모니터: 1000000 baud, 새 줄(Newline) 또는 Both NL & CR.
 * 2. 방식, 차수, 중앙값 필터의 측정 횟수를 차례로 입력.
 *    빈 줄을 보내면 화면에 표시된 기본값을 사용.
 * 3. 탁구공을 0, 5, 10, 15, 20, 25, 30 cm에 놓고 각각 Enter.
 * 4. 자동으로 출력된 volt_to_distance() 전체를 예제 2의 함수와 교체.
 * 5. 이후 Enter로 거리 검증, R + Enter로 처음부터 다시 보정.
 *
 * 방식 1: 거리 = ADC에 대한 다항식
 * 방식 2: 거리 = (1 / ADC)에 대한 다항식
 * 수치 오차를 줄이기 위해 x를 -1~1로 정규화하고 QR 최소제곱법 사용.
 * 출력 함수에도 정규화 상수를 포함하므로 그대로 복사해서 사용 가능.
 * 추가 라이브러리 불필요. Arduino Uno의 10비트 ADC 기준.
 */

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

#define PIN_IR A0

const uint8_t POINT_COUNT = 7;
const uint8_t MAX_DEGREE = 5;
const uint8_t MAX_SAMPLES = 100;

uint8_t fitMethod = 1;
uint8_t fitDegree = 2;
uint8_t sampleCount = 100;

unsigned int adcValues[POINT_COUNT];
unsigned int samples[MAX_SAMPLES];
float coefficients[MAX_DEGREE + 1];  // 상수항부터 저장
float xCenter = 0.0f;
float xScale = 1.0f;

// 함수 원형을 명시하여 Arduino 스케치 전처리 순서에 의존하지 않음.
bool readLine(char *buffer, uint8_t capacity);
uint8_t readSetting(uint8_t minimum, uint8_t maximum, uint8_t defaultValue);
int compare(const void *a, const void *b);
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose);
float transformAdc(unsigned int adc);
bool fitCurve();
float volt_to_distance(unsigned int a_value);
void printEquation();
void printReport();
void calibrate();

void setup()
{
  Serial.begin(1000000);  // 제공된 예제와 동일
  // Native USB 보드는 최대 3초만 연결을 기다림.
  unsigned long started = millis();
  while (!Serial && millis() - started < 3000UL) {
    delay(1);
  }
  calibrate();
}

void loop()
{
  char line[24];
  Serial.println(F("Enter: measure distance | R + Enter: recalibrate"));
  if (!readLine(line, sizeof(line))) {
    Serial.println(F("Input too long. Try again."));
    return;
  }
  if (line[0] == 'r' || line[0] == 'R') {
    calibrate();
    return;
  }
  if (line[0] != '\0') {
    Serial.println(F("Send an empty line to measure."));
    return;
  }

  unsigned int filtered = ir_sensor_filtered(sampleCount, 0.5f, 0);
  if (fitMethod == 2 && filtered == 0) {
    Serial.println(F("ADC=0: inverse model cannot be evaluated."));
    return;
  }
  Serial.print(F("FLT:"));
  Serial.print(filtered);
  Serial.print(F(" ==> Distance:"));
  Serial.print(volt_to_distance(filtered), 2);
  Serial.println(F(" mm"));

  unsigned int low = adcValues[0];
  unsigned int high = adcValues[0];
  for (uint8_t i = 1; i < POINT_COUNT; ++i) {
    if (adcValues[i] < low) low = adcValues[i];
    if (adcValues[i] > high) high = adcValues[i];
  }
  if (filtered < low || filtered > high) {
    Serial.println(F("Warning: outside calibrated ADC range (extrapolation)."));
  }
}

// CR, LF, CRLF를 모두 처리. CRLF 한 쌍으로 측정이 두 번 되지 않음.
// String 대신 고정 버퍼를 사용하여 Uno의 메모리 단편화를 방지.
bool readLine(char *buffer, uint8_t capacity)
{
  static bool skipLF = false;
  uint8_t length = 0;
  bool overflow = false;

  while (true) {
    if (Serial.available() == 0) {
      delay(1);
      continue;
    }
    char c = (char)Serial.read();
    if (skipLF) {
      skipLF = false;
      if (c == '\n') continue;
    }
    if (c == '\r' || c == '\n') {
      skipLF = (c == '\r');
      buffer[length] = '\0';
      return !overflow;
    }
    if (length < capacity - 1) {
      buffer[length++] = c;
    } else {
      overflow = true;  // 남은 입력은 줄 끝까지 소비
    }
  }
}

uint8_t readSetting(uint8_t minimum, uint8_t maximum, uint8_t defaultValue)
{
  char line[24];
  while (true) {
    Serial.print(F("Range "));
    Serial.print(minimum);
    Serial.print('-');
    Serial.print(maximum);
    Serial.print(F(", default "));
    Serial.print(defaultValue);
    Serial.println(F(": enter a number, then Enter."));

    if (readLine(line, sizeof(line))) {
      if (line[0] == '\0') return defaultValue;
      char *end;
      long value = strtol(line, &end, 10);
      bool hasNumber = end != line;
      while (isspace((unsigned char)*end)) ++end;
      if (hasNumber && *end == '\0' && value >= minimum && value <= maximum) {
        return (uint8_t)value;
      }
    }
    Serial.println(F("Invalid setting. Please try again."));
  }
}

int compare(const void *a, const void *b)
{
  unsigned int left = *(const unsigned int *)a;
  unsigned int right = *(const unsigned int *)b;
  return (left > right) - (left < right);
}

// 예제 1, 2와 같은 필터 인터페이스. 동적 할당 대신 고정 배열 사용.
// position=0.5일 때 정렬된 배열의 n/2 번째 값(예제와 동일).
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  if (n == 0 || n > MAX_SAMPLES || !(position >= 0.0f && position <= 1.0f)) {
    return 0;
  }
  unsigned long started = millis();
  for (unsigned int i = 0; i < n; ++i) {
    samples[i] = analogRead(PIN_IR);
    if (verbose == 1) {
      Serial.print(samples[i]);
      Serial.print(' ');
    }
  }
  qsort(samples, n, sizeof(samples[0]), compare);
  unsigned int index = (unsigned int)(n * position);
  if (index >= n) index = n - 1;
  unsigned int result = samples[index];
  if (verbose == 1) Serial.println();
  if (verbose >= 2) {
    Serial.print(F("Elapsed time: "));
    Serial.print(millis() - started);
    Serial.println(F(" ms"));
  }
  return result;
}

float transformAdc(unsigned int adc)
{
  return fitMethod == 2 ? 1.0f / (float)adc : (float)adc;
}

bool fitCurve()
{
  if (fitMethod == 2 && adcValues[0] == 0) return false;
  float minimum = transformAdc(adcValues[0]);
  float maximum = minimum;
  for (uint8_t i = 0; i < POINT_COUNT; ++i) {
    if (fitMethod == 2 && adcValues[i] == 0) return false;
    float x = transformAdc(adcValues[i]);
    if (x < minimum) minimum = x;
    if (x > maximum) maximum = x;
  }
  xCenter = (minimum + maximum) * 0.5f;
  xScale = (maximum - minimum) * 0.5f;
  if (!(xScale > 0.0f)) return false;

  // A의 각 행 = [1, x, x^2, ...], b = 실제 거리(mm).
  // Householder QR: A^T A를 만들지 않고 최소제곱 문제를 해결.
  float a[POINT_COUNT][MAX_DEGREE + 1];
  float b[POINT_COUNT];
  float v[POINT_COUNT];
  uint8_t columns = fitDegree + 1;
  for (uint8_t i = 0; i < POINT_COUNT; ++i) {
    float x = (transformAdc(adcValues[i]) - xCenter) / xScale;
    a[i][0] = 1.0f;
    for (uint8_t j = 1; j < columns; ++j) a[i][j] = a[i][j - 1] * x;
    b[i] = 50.0f * i;  // 0, 50, 100, ..., 300 mm
  }

  for (uint8_t k = 0; k < columns; ++k) {
    float normSquared = 0.0f;
    for (uint8_t i = k; i < POINT_COUNT; ++i) normSquared += a[i][k] * a[i][k];
    float norm = sqrtf(normSquared);
    if (!(norm > 0.00001f)) return false;  // 중복 등으로 데이터가 부족한 경우

    float alpha = a[k][k] >= 0.0f ? -norm : norm;
    for (uint8_t i = k; i < POINT_COUNT; ++i) v[i] = a[i][k];
    v[k] -= alpha;
    float vSquared = 0.0f;
    for (uint8_t i = k; i < POINT_COUNT; ++i) vSquared += v[i] * v[i];
    float beta = 2.0f / vSquared;

    for (uint8_t j = k + 1; j < columns; ++j) {
      float dot = 0.0f;
      for (uint8_t i = k; i < POINT_COUNT; ++i) dot += v[i] * a[i][j];
      for (uint8_t i = k; i < POINT_COUNT; ++i) a[i][j] -= beta * v[i] * dot;
    }
    float dot = 0.0f;
    for (uint8_t i = k; i < POINT_COUNT; ++i) dot += v[i] * b[i];
    for (uint8_t i = k; i < POINT_COUNT; ++i) b[i] -= beta * v[i] * dot;

    a[k][k] = alpha;
    for (uint8_t i = k + 1; i < POINT_COUNT; ++i) a[i][k] = 0.0f;
  }

  for (int k = columns - 1; k >= 0; --k) {
    float sum = b[k];
    for (uint8_t j = k + 1; j < columns; ++j) sum -= a[k][j] * coefficients[j];
    coefficients[k] = sum / a[k][k];
    if (!isfinite(coefficients[k])) return false;
  }
  return true;
}

float volt_to_distance(unsigned int a_value)
{
  if (fitMethod == 2 && a_value == 0) return -1.0f;
  float x = (transformAdc(a_value) - xCenter) / xScale;
  float distance = coefficients[fitDegree];
  for (int i = fitDegree - 1; i >= 0; --i) {
    distance = distance * x + coefficients[i];  // Horner 방식으로 다항식 계산
  }
  return distance;
}

void printEquation()
{
  Serial.println();
  Serial.println(F("===== COPY THE FUNCTION BELOW ====="));
  Serial.println(F("// Output unit: mm. Use within the calibrated ADC range."));
  Serial.println(F("float volt_to_distance(unsigned int a_value)"));
  Serial.println(F("{"));
  if (fitMethod == 2) {
    Serial.println(F("  if (a_value == 0) return -1.0f; // invalid ADC"));
  }
  Serial.print(F("  const float x = ("));
  if (fitMethod == 2) Serial.print(F("1.0f / (float)a_value"));
  else Serial.print(F("(float)a_value"));
  Serial.print(F(" - ("));
  Serial.print(xCenter, 10);
  Serial.print(F("f)) / "));
  Serial.print(xScale, 10);
  Serial.println(F("f;"));

  Serial.print(F("  return "));
  for (uint8_t i = 0; i < fitDegree; ++i) Serial.print('(');
  Serial.print(coefficients[fitDegree], 8);
  Serial.print('f');
  for (int i = fitDegree - 1; i >= 0; --i) {
    Serial.print(F(" * x + ("));
    Serial.print(coefficients[i], 8);
    Serial.print(F("f))"));
  }
  Serial.println(';');
  Serial.println('}');
  Serial.println(F("===== END OF FUNCTION ====="));
}

void printReport()
{
  float errorSquared = 0.0f;
  float totalSquared = 0.0f;
  bool increasing = true;
  bool decreasing = true;

  Serial.println();
  Serial.println(F("ADC\tActual(mm)\tFitted(mm)\tError(mm)"));
  for (uint8_t i = 0; i < POINT_COUNT; ++i) {
    float actual = 50.0f * i;
    float predicted = volt_to_distance(adcValues[i]);
    float error = predicted - actual;
    errorSquared += error * error;
    totalSquared += (actual - 150.0f) * (actual - 150.0f);
    Serial.print(adcValues[i]);
    Serial.print('\t');
    Serial.print(actual, 0);
    Serial.print(F("\t\t"));
    Serial.print(predicted, 2);
    Serial.print(F("\t\t"));
    Serial.println(error, 2);
    if (i > 0) {
      if (adcValues[i] <= adcValues[i - 1]) increasing = false;
      if (adcValues[i] >= adcValues[i - 1]) decreasing = false;
    }
  }
  Serial.print(F("R^2 = "));
  Serial.println(1.0f - errorSquared / totalSquared, 6);
  Serial.print(F("Calibration RMSE = "));
  Serial.print(sqrtf(errorSquared / POINT_COUNT), 3);
  Serial.println(F(" mm"));
  Serial.println(F("Verify accuracy at additional positions, e.g. 55, 155, 255 mm."));
  if (!increasing && !decreasing) {
    Serial.println(F("Warning: ADC is not strictly monotonic. Check placement/range."));
  }
  if (fitDegree > 3) {
    Serial.println(F("Warning: high degree can overfit seven calibration points."));
  }
}

void calibrate()
{
  char line[24];
  while (true) {
    Serial.println();
    Serial.println(F("=== 20C10: IR SENSOR AUTO CURVE FITTING ==="));
    Serial.println(F("Serial monitor: 1000000 baud, Newline or Both NL & CR."));
    Serial.println(F("Method 1: polynomial in ADC"));
    Serial.println(F("Method 2: polynomial in 1/ADC"));
    fitMethod = readSetting(1, 2, 1);
    Serial.println(F("Polynomial degree (start with 2 or 3):"));
    fitDegree = readSetting(1, MAX_DEGREE, 2);
    Serial.println(F("Samples per position for median filtering:"));
    sampleCount = readSetting(1, MAX_SAMPLES, 100);

    for (uint8_t i = 0; i < POINT_COUNT; ++i) {
      while (true) {
        Serial.print(F("Place ball at "));
        Serial.print(i * 5);
        Serial.print(F(" cm ("));
        Serial.print(i * 50);
        Serial.println(F(" mm), wait until still, then send an EMPTY line."));
        if (!readLine(line, sizeof(line)) || line[0] != '\0') {
          Serial.println(F("Only an empty line triggers measurement."));
          continue;
        }
        adcValues[i] = ir_sensor_filtered(sampleCount, 0.5f, 0);
        if (fitMethod == 2 && adcValues[i] == 0) {
          Serial.println(F("ADC=0 cannot be inverted. Check wiring, then retry."));
          continue;
        }
        Serial.print(F("Saved ADC = "));
        Serial.println(adcValues[i]);
        if (adcValues[i] == 0 || adcValues[i] == 1023) {
          Serial.println(F("Warning: ADC at limit. Check sensor wiring/range."));
        }
        break;
      }
    }

    if (fitCurve()) {
      printEquation();
      printReport();
      return;
    }
    Serial.println(F("Fit failed: insufficient distinct ADC values or unstable data."));
    Serial.println(F("Check sensor/positions or lower the degree. Restarting setup."));
  }
}
