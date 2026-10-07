int period;
float duty;

void set_period(int p) {
  period = p;
}

void set_duty(float d) {
  duty = 1 - d / 100.0;
}

void setup() {
  pinMode(7, OUTPUT);
  set_period(100);   // PWM 한 주기 = 100 us
  set_duty(100);
}

void run_pwm(int pwm_pin) {
  digitalWrite(pwm_pin, HIGH);
  delayMicroseconds(period * duty);

  digitalWrite(pwm_pin, LOW);
  delayMicroseconds(period - period * duty);
}

void loop() {

  // 밝아짐
  for (int duty_value = 0; duty_value <= 100; duty_value += 2) {
    set_duty(duty_value);

    for (int j = 0; j < 100; j++) {
      run_pwm(7);
    }
  }

  // 어두워짐
  for (int duty_value = 100; duty_value >= 0; duty_value -= 2) {
    set_duty(duty_value);

    for (int j = 0; j < 100; j++) {
      run_pwm(7);
    }
  }
}
