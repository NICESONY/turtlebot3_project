#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);   // 시리얼 통신 시작
  servo1.attach(9);
  servo2.attach(10);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '1') {
      dump();  // dump 동작 수행
    }
  }
}

void dump() {
  // 0도 → 45도까지 천천히 이동(단, 45도에서 시작하는 점에 유의)
  for (int angle = 0; angle <= 45; angle++) {
    servo1.write(angle);
    servo2.write(angle);
    delay(25); // 회전 속도(이 값이 작을수록 빠르고, 클수록 느림)
  }

  delay(2000);

  // 45도 → 0도까지 복귀
  for (int angle = 45; angle >= 0; angle--) {
    servo1.write(angle);
    servo2.write(angle);
    delay(25);
  }

  delay(2000);
}
