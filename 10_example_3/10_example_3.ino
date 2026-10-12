  /*
 * Challenge 10C16 - Parking barrier (Arduino Uno / HC-SR04 / positional servo)
 * Based on the public-domain example by ArduinoGetStarted.com:
 * https://arduinogetstarted.com/faq/how-to-control-speed-of-servo-motor
 *
 * Connections: servo signal D10, HC-SR04 TRIG D7 / ECHO D8.
 * Servo: suitable external regulated supply, common GND with Arduino.
 * Put a background reflector 40-100 cm across the empty model lane.
 * Serial Monitor: 115200 baud, 's' = sigmoid, 'm' = smoothstep.
 * A new selection takes effect when the NEXT movement starts.
 * This is a classroom model, not a real vehicle safety controller.
 */

#include <Servo.h>
#include <math.h>

const byte PIN_SERVO = 10;
const byte PIN_TRIG = 7;
const byte PIN_ECHO = 8;

// Original example's angles/time. Calibrate the bar before attaching it.
// 30 -> 90 is a 60-degree rotation; use 120 for a 90-degree lift if suitable.
const int CLOSED_ANGLE = 0;
const int OPEN_ANGLE = 90;
const unsigned long MOVING_TIME = 3000UL;
const unsigned long SERVO_INTERVAL = 20UL;
const unsigned long SENSOR_INTERVAL = 60UL;
const unsigned long CLEAR_HOLD_TIME = 1500UL;
const unsigned long ECHO_TIMEOUT_US = 12000UL;
const float DETECT_DISTANCE_CM = 15.0f;
const float CLEAR_DISTANCE_CM = 25.0f;
const byte DETECT_SAMPLES = 3;
const byte CLEAR_SAMPLES = 5;

enum CurveType { SIGMOID, SMOOTHSTEP };
enum GateState { CLOSED, OPENING, OPEN, CLOSING };

Servo myServo;
CurveType selectedCurve = SIGMOID;
CurveType activeCurve = SIGMOID;
GateState gateState = CLOSED;

unsigned long moveStartTime = 0;
unsigned long lastServoTime = 0;
unsigned long lastSensorTime = 0;
unsigned long clearStartTime = 0;
unsigned long lastLogTime = 0;
float startAngle = CLOSED_ANGLE;
float stopAngle = CLOSED_ANGLE;
int currentAngle = CLOSED_ANGLE; // Last commanded angle, not measured position.
float distanceCm = -1.0f;
bool sensorValid = false;
bool vehiclePresent = false;
bool pathClear = false;
bool countingClearTime = false;
byte nearCount = 0;
byte farCount = 0;

// Normalize the logistic curve so f(0)=0 and f(1)=1 exactly.
float sigmoidEase(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  const float k = 10.0f;
  const float low = 1.0f / (1.0f + expf(k * 0.5f));
  const float high = 1.0f - low;
  const float value = 1.0f / (1.0f + expf(-k * (t - 0.5f)));
  return (value - low) / (high - low);
}

// Cubic derived from f(0)=0, f(1)=1, f'(0)=f'(1)=0.
float smoothstepEase(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

void beginMove(int targetAngle, GateState newState, unsigned long now) {
  startAngle = currentAngle; // Reversal starts at the last commanded angle.
  stopAngle = targetAngle;
  moveStartTime = now;
  activeCurve = selectedCurve; // Do not change the curve halfway through.
  gateState = newState;
  countingClearTime = false;
}

void updateServo(unsigned long now) {
  if (gateState != OPENING && gateState != CLOSING) return;
  const unsigned long progress = now - moveStartTime;
  if (progress >= MOVING_TIME) {
    currentAngle = (int)stopAngle;
    myServo.write(currentAngle); // Always command the exact endpoint.
    gateState = (gateState == OPENING) ? OPEN : CLOSED;
    countingClearTime = false;
    return;
  }
  if (now - lastServoTime < SERVO_INTERVAL) return;
  
  lastServoTime = now;
  const float t = (float)progress / (float)MOVING_TIME;
  const float eased = (activeCurve == SIGMOID)
                          ? sigmoidEase(t) : smoothstepEase(t);
  currentAngle = (int)lroundf(startAngle + (stopAngle - startAngle) * eased);
  myServo.write(currentAngle);
}

void updateSensor(unsigned long now) {
  if (now - lastSensorTime < SENSOR_INTERVAL) return;
  lastSensorTime = now;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  const unsigned long duration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  distanceCm = duration * 0.0343f / 2.0f;
  sensorValid = (duration != 0 && distanceCm >= 2.0f && distanceCm <= 200.0f);
  if (!sensorValid) {
    distanceCm = -1.0f;
    nearCount = farCount = 0;
    pathClear = false; // No echo does NOT prove that the vehicle passed.
    return;
  }

  if (distanceCm <= DETECT_DISTANCE_CM) {
    farCount = 0;
    pathClear = false;
    if (nearCount < DETECT_SAMPLES) nearCount++;
    if (nearCount >= DETECT_SAMPLES) vehiclePresent = true;
  } else if (distanceCm >= CLEAR_DISTANCE_CM) {
    nearCount = 0;
    if (farCount < CLEAR_SAMPLES) farCount++;
    if (farCount >= CLEAR_SAMPLES) {
      vehiclePresent = false;
      pathClear = true;
    }
  } else {
    nearCount = farCount = 0;
    pathClear = false; // Hysteresis band: keep the presence latch.
  }
}

void updateGate(unsigned long now) {
  switch (gateState) {
    case CLOSED:
      if (vehiclePresent) beginMove(OPEN_ANGLE, OPENING, now);
      break;
    case OPENING:
      break; // Finish lifting even if the car leaves the sensing area.
    case OPEN:
      if (!pathClear) {
        countingClearTime = false;
      } else if (!countingClearTime) {
        clearStartTime = now;
        countingClearTime = true;
      } else if (now - clearStartTime >= CLEAR_HOLD_TIME) {
        beginMove(CLOSED_ANGLE, CLOSING, now);
      }
      break;
    case CLOSING:
      // A near/uncertain reading immediately interrupts lowering.
      if (!pathClear) beginMove(OPEN_ANGLE, OPENING, now);
      break;
  }
}

const char* stateName() {
  switch (gateState) {
    case CLOSED: return "CLOSED";
    case OPENING: return "OPENING";
    case OPEN: return "OPEN";
    default: return "CLOSING";
  }
}

void updateSerial(unsigned long now) {
  while (Serial.available() > 0) {
    const char command = Serial.read();
    if (command == 's' || command == 'S') selectedCurve = SIGMOID;
    if (command == 'm' || command == 'M') selectedCurve = SMOOTHSTEP;
  }
  if (now - lastLogTime < 100UL) return;
  lastLogTime = now;
  const bool moving = (gateState == OPENING || gateState == CLOSING);
  const CurveType loggedCurve = moving ? activeCurve : selectedCurve;
  Serial.print(now); Serial.print(',');
  Serial.print(distanceCm, 1); Serial.print(',');
  Serial.print(currentAngle); Serial.print(',');
  Serial.print(stateName()); Serial.print(',');
  Serial.print(loggedCurve == SIGMOID ? "sigmoid" : "smoothstep");
  Serial.print(','); Serial.println(sensorValid ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  myServo.write(CLOSED_ANGLE);
  myServo.attach(PIN_SERVO);
  Serial.println("ms,distance_cm,command_angle,state,curve,valid");
}

void loop() {
  updateSensor(millis());
  const unsigned long now = millis(); // Re-read time after bounded pulseIn().
  updateGate(now);
  updateServo(now);
  updateSerial(now);
}