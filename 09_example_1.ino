// Arduino pin assignment
#include <stdio.h>
#include <stdlib.h>

#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.
#define N 30
float filter_arr[N];

// global variables
unsigned long last_sampling_time;   // unit: msec
float dist_prev = _DIST_MAX;        // Distance last-measured
float dist_ema;                     // EMA distance

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);
}

void FIFO(float raw, float arr[]) {
  for (int i = 1; i < N; i++) {
    arr[i - 1] = arr[i];
  }

  arr[N - 1] = raw;
}

int compare(const void *a, const void *b) {
  float fa = *(const float*)a;
  float fb = *(const float*)b;

  if (fa < fb) return -1;
  if (fa > fb) return 1;
  return 0;
}

// 2. 중위값을 계산하는 함수
float getMedian(float arr[], int size) {
  float temp[N];

  for (int i = 0; i < size; i++) {
    temp[i] = arr[i];
  }

  qsort(temp, size, sizeof(float), compare);

  if (size % 2 != 0) {
    return temp[size / 2];
  } else {
    return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
  }
}

void loop() {
  float dist_raw, dist_filtered;
  
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. 
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  // Modify the below if-else statement to implement the range filter
  FIFO(dist_raw, filter_arr);
  dist_filtered = getMedian(filter_arr, N);

  // Modify the below line to implement the EMA equation
  dist_ema = _EMA_ALPHA*dist_filtered + (1-_EMA_ALPHA)*dist_ema;

  // output the distance to the serial port
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",raw:"); Serial.print(min(dist_raw, _DIST_MAX + 100));
  Serial.print(",ema:");  Serial.print(min(dist_ema, _DIST_MAX + 100));
  Serial.print(",median:");  Serial.print(min(dist_filtered, _DIST_MAX + 100));
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  // do something here
  if ((dist_ema < _DIST_MIN) || (dist_ema > _DIST_MAX))
    digitalWrite(PIN_LED, 1);       // LED OFF
  else
    digitalWrite(PIN_LED, 0);       // LED ON

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm

  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - pulseIn(ECHO, HIGH, timeout) returns microseconds (음파의 왕복 시간)
  // - 편도 거리 = (pulseIn() / 1,000,000) * SND_VEL / 2 (미터 단위)
  //   mm 단위로 하려면 * 1,000이 필요 ==>  SCALE = 0.001 * 0.5 * SND_VEL
  //
  // - 예, pusseIn()이 100,000 이면 (= 0.1초, 왕복 거리 34.6m)
  //        = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //        = 100,000 * 0.001 * 0.5 * 346
  //        = 17,300 mm  ==> 17.3m
}
