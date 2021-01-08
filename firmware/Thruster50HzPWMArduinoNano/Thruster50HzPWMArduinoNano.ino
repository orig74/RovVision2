#include <Servo.h>


#define PWM_MIDPOINT 1490
#define TIMEOUT 500000


typedef struct {
  uint8_t header;
  int8_t motors[8];
  uint8_t tail;  
} message_t;


Servo thrusters[8];

void setup() {
  for (int t_ind = 0; t_ind < 8; t_ind++) {
    thrusters[t_ind].writeMicroseconds(PWM_MIDPOINT);
  }
  thrusters[0].attach(11);
  thrusters[1].attach(10);
  thrusters[2].attach(9);
  thrusters[3].attach(8);
  thrusters[4].attach(A3);
  thrusters[5].attach(A2);
  thrusters[6].attach(A1);
  thrusters[7].attach(A0);
  delay(1000);
  
  Serial.begin(115200);
}


message_t  message_buff;
unsigned long last_msg_us = micros();
void loop() {
  if (Serial.available() >= 10) {
    Serial.readBytes((char *)&message_buff, sizeof(message_t));
    
    if (message_buff.header == 255 && message_buff.tail == 255) {
      last_msg_us = micros();
      for (int t_ind = 0; t_ind < 8; t_ind++) {
	int microseconds = PWM_MIDPOINT + 4 * message_buff.motors[t_ind];
        thrusters[t_ind].writeMicroseconds(microseconds);
      }
      
    } else {
      // Error flushing
      Serial.println("Serial Error!");
      while(Serial.available()) 
        {
          Serial.read();
        }
    }
  }

  if ((micros() - last_msg_us) > TIMEOUT) {
    for (int t_ind = 0; t_ind < 8; t_ind++) {
      thrusters[t_ind].writeMicroseconds(PWM_MIDPOINT);
     }
  }
}
