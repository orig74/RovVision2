#include <Servo.h>

Servo thrusters[8];

void setup() {
  thrusters[0].attach(A0);
  thrusters[1].attach(A1);
  thrusters[2].attach(A2);
  thrusters[3].attach(A3);
  thrusters[4].attach(8);
  thrusters[5].attach(9);
  thrusters[6].attach(10);
  thrusters[7].attach(11);
  for (int t_ind = 0; t_ind < 8; t_ind++) {
    thrusters[t_ind].writeMicroseconds(1500);
  }
  
  Serial.begin(115200);
}


void loop() {
  static char serial_buff[10];
  static long itts_since_last = 0;
  int sbuff_ind = 0;
  
  if (Serial.available() >= 10) {
    Serial.readBytes(serial_buff, 10);
    itts_since_last = 0;
  } else {
    delayMicroseconds(100);
    itts_since_last++;
  }

  if (itts_since_last < 1000) {
    for (int t_ind = 0; t_ind < 8; t_ind++) {
      int thrust_vl_us = 1000 + 3.92 * serial_buff[t_ind + 1];
      thrusters[t_ind].writeMicroseconds(thrust_vl_us);
    }
  } else {
    for (int t_ind = 0; t_ind < 8; t_ind++) {
      thrusters[t_ind].writeMicroseconds(1500);
    }
  }
}
