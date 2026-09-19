int period;
float duty;


void set_period(int p) {
  period = p;
}

void set_duty(float d) {
  duty = 1 - d/100.0;
}

void setup() {
  pinMode(7, OUTPUT);
  set_period(100);
  set_duty(100);
}

void run_pwm(int pwm_pin) {
  digitalWrite(pwm_pin, HIGH);
  delayMicroseconds(period*duty);
  digitalWrite(pwm_pin, LOW);
  delayMicroseconds(period-period*duty);
}

void loop() {

  for (int i = 0; i <= 100; i += 2) {
    set_duty(i);
    for (int i=0; i<50; i++ {
      run_pwm(7);
    }
  }

  for (int i = 100; i >= 0; i -=2) {
    set_duty(i);
    for (int i=0; i<50; i++ {
      run_pwm(7);
    }
  }

  // period가 10ms면 1초동안 triangle 파형으로 출력하려면 duty가 1씩 올라갈수가 없어서 2씩 올림
}
