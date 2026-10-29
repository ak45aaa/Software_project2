#include <Servo.h>

#define PIN_IR    A0
#define PIN_LED   9
#define PIN_SERVO 10

// Use the pulse widths measured in your servo calibration experiment.
#define _DUTY_MIN 500
#define _DUTY_NEU 1950
#define _DUTY_MAX 2400

#define _DIST_MIN 100.0
#define _DIST_MAX 250.0
#define EMA_ALPHA 0.2
#define LOOP_INTERVAL 50UL

Servo myservo;
unsigned long last_loop_time = 0;
float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

// Collect three readings from different loop intervals, not an ADC burst.
int ir_history[3];
byte ir_history_next = 0;
bool ir_history_ready = false;

int median3(int a, int b, int c)
{
  int temp;
  if (a > b) { temp = a; a = b; b = temp; }
  if (b > c) { temp = b; b = c; c = temp; }
  if (a > b) { temp = a; a = b; b = temp; }
  return b;
}

float adcToDistance(int adc)
{
  if (adc <= 9)
    return -1.0;

  // Assignment slide 13: includes the -60 mm offset.
  return ((6762.0 / (adc - 9.0)) - 4.0) * 10.0 - 60.0;
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);  // Assumes HIGH turns the LED on.

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(1000000);
  last_loop_time = millis();
}

void loop()
{
  unsigned long time_curr = millis();
  if (time_curr - last_loop_time < LOOP_INTERVAL)
    return;

  // Prevent back-to-back catch-up readings after a delayed iteration.
  last_loop_time = time_curr;

  int a_value = analogRead(PIN_IR);
  float dist_raw = adcToDistance(a_value);

  if (!ir_history_ready) {
    for (byte i = 0; i < 3; ++i)
      ir_history[i] = a_value;
    ir_history_ready = true;
  } else {
    ir_history[ir_history_next] = a_value;
    ir_history_next = (ir_history_next + 1) % 3;
  }

  // Reject an isolated high or low ADC reading before distance conversion.
  int ir_median = median3(ir_history[0], ir_history[1], ir_history[2]);
  float dist_candidate = adcToDistance(ir_median);

  bool in_range = (dist_candidate >= _DIST_MIN
                  && dist_candidate <= _DIST_MAX);
  digitalWrite(PIN_LED, in_range ? HIGH : LOW);
  if (in_range)
    dist_prev = dist_candidate;

  // Keep the last accepted distance if the median is outside the range.
  dist_ema = EMA_ALPHA * dist_prev + (1.0 - EMA_ALPHA) * dist_ema;

  int duty = (int)(_DUTY_MIN
                  + (dist_ema - _DIST_MIN) * (_DUTY_MAX - _DUTY_MIN)
                  / (_DIST_MAX - _DIST_MIN));
  myservo.writeMicroseconds(duty);

  // IR and dist_raw remain unfiltered for diagnosing the sensor signal.
  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print(",_DIST_MIN:"); Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.println(_DUTY_MAX);
}
