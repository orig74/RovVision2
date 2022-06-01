#include <EEPROM.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <driver/adc.h>
#include "MS5837.h"

#define STATUS_LED_PIN 2
#define LIGHTS_PIN 16
#define CAM_SERVO_PIN 17
int thruster_pins[8] = {12, 13, 14, 27, 25, 26, 33, 32};
int thruster_dirs[8] = {-1, -1, 1, 1, -1, 1, 1, -1};
#define CURRENT_ADC_PIN 15
#define VOLTAGE_ADC_PIN 4
#define MOISTURE_SENSE_PIN 5
#define THRUSTERS_CURRENT_PIN_1 35
#define THRUSTERS_CURRENT_PIN_2 34

#define SENSOR_UPDATE_FPS 3
#define SENSOR_EVENT_MICROS (1000000/SENSOR_UPDATE_FPS)

#define SERIAL_BAUDRATE 500000
#define MAINLOOP_PERIOD_US 10

#define N_SERIAL_TX_BYTES 11
#define N_SERIAL_RX_BYTES 20
byte msg_buff[N_SERIAL_RX_BYTES];
byte serial_count;

Servo thrusters[8];
#define PWM_MIDPOINT 1490
#define PWM_RANGE_H 500
Servo lights;
Servo cam_servo;
int servo_offset = 0;

unsigned long prev_sensor_event_us;
unsigned long prev_loop_us;
unsigned long no_data_cnt;

MS5837 DepthSensor;


uint16_t unpack_16bit(byte* byte_buff, int byte_idx) {
  return (uint16_t) byte_buff[byte_idx+1] | ((uint16_t) byte_buff[byte_idx] << 8);
}

uint16_t CalcChksm(byte* tx_buff, int buff_len) {
  uint16_t chksm = 0;
  for (int idx=0; idx < buff_len; idx++) {
    chksm += (uint16_t) tx_buff[idx];
  }
  return chksm;
}

bool CheckSerialMsgCHKSM(byte* c_buff)
{
  uint16_t msg_chksum = unpack_16bit(c_buff, N_SERIAL_RX_BYTES - 2);
  uint16_t chksm_calc = CalcChksm(c_buff, N_SERIAL_RX_BYTES - 2);
  return chksm_calc == msg_chksum && msg_chksum;
}

int ScaleUint16(uint16_t in_val, int out_min, int out_max) {
  return out_min + (int) ((float) (out_max - out_min) * (float) in_val / 65535);
}


void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(THRUSTERS_CURRENT_PIN_1, INPUT);
  pinMode(THRUSTERS_CURRENT_PIN_2, INPUT);
  pinMode(CURRENT_ADC_PIN, INPUT);
  pinMode(VOLTAGE_ADC_PIN, INPUT);
  pinMode(MOISTURE_SENSE_PIN, INPUT);

  lights.attach(LIGHTS_PIN, 1100, 1900);
  lights.writeMicroseconds(1100);
  cam_servo.attach(CAM_SERVO_PIN);
  for (int t_ind = 0; t_ind < 8; t_ind++) {
    thrusters[t_ind].attach(thruster_pins[t_ind]);
    thrusters[t_ind].writeMicroseconds(PWM_MIDPOINT);
  }
  cam_servo.write(90);
  
  Serial.begin(SERIAL_BAUDRATE);

  Wire.begin();
  while (!DepthSensor.init()) {
    //Serial.println("Init failed!");
    //Serial.println("Are SDA/SCL connected correctly?");
    //Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    delay(500);
  }
  DepthSensor.setModel(MS5837::MS5837_30BA);
  DepthSensor.setFluidDensity(1029); // kg/m^3 (997 for freshwater, 1029 for seawater)

  digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
  // ArduinoOTA.handle();

  while (micros() < prev_loop_us + MAINLOOP_PERIOD_US) {}
  prev_loop_us = micros();

  // Serial RX
  while (Serial.available()) {
    // Roll input buffer if full
    if (serial_count == N_SERIAL_RX_BYTES) {
      for (int i=0; i < N_SERIAL_RX_BYTES-1; i++) msg_buff[i] = msg_buff[i+1];
      serial_count--;
    }
    byte c = Serial.read();
    msg_buff[serial_count++] = c;
  }

  // Check checksum on last bits in buffer to see if valid msg
  if (serial_count > N_SERIAL_RX_BYTES - 1) {
    if (CheckSerialMsgCHKSM(msg_buff)) {
      // Valid outputs message recieved, update outputs
      no_data_cnt = 0;
      digitalWrite(STATUS_LED_PIN, HIGH);
      
      // Thrusters
      for (int thrstr_idx=0; thrstr_idx < 8; thrstr_idx++) {
        uint16_t thrst_16b = unpack_16bit(msg_buff, 2 * thrstr_idx);
        int thrst_int = ScaleUint16(thrst_16b, -PWM_RANGE_H, PWM_RANGE_H);
        int thrst_us = thruster_dirs[thrstr_idx] * thrst_int + PWM_MIDPOINT;
        thrusters[thrstr_idx].writeMicroseconds(thrst_us);
      }
      
      // Lights
      byte lights_8b = msg_buff[16];
      lights.writeMicroseconds(1100 + (int) 3.14 * (float) lights_8b);
      
      // Camera Servo
      byte servo_8b = msg_buff[17];
      cam_servo.write(90 + (int) (0.3 * ((float) servo_8b - 127)));

      // Clear input buffer
      for (int i=0; i < N_SERIAL_RX_BYTES; i++) msg_buff[i] = '\0';
      serial_count = 0;
    } else {
      //Serial.println("INVALID_CHKSM");
      no_data_cnt++;
    }
  } else no_data_cnt++;

  // If no serial messages for N loops write outputs to OFF
  if (no_data_cnt > 10000) {
    no_data_cnt = 0;
    for (int thrstr_idx=0; thrstr_idx < 8; thrstr_idx++) {
        thrusters[thrstr_idx].writeMicroseconds(PWM_MIDPOINT);
    }
    lights.writeMicroseconds(1100);
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  // Sensor read task
  if((micros() - prev_sensor_event_us) > SENSOR_EVENT_MICROS) {
    prev_sensor_event_us = micros();

    DepthSensor.read();
    float depth_m = DepthSensor.depth();
    float temp_c = DepthSensor.temperature();
    uint16_t depth_u16 = (uint16_t) min(max((int) round(depth_m*200), 0), 65536);
    uint16_t temp_u16 = (uint16_t) min(max((int) round(temp_c*200), 0), 65536);
    int adc_volt = analogRead(VOLTAGE_ADC_PIN);
    int adc_amps = analogRead(CURRENT_ADC_PIN);
    bool leaking_now = digitalRead(MOISTURE_SENSE_PIN);
    byte message[N_SERIAL_TX_BYTES] = {lowByte(depth_u16), highByte(depth_u16),
                       lowByte(temp_u16), highByte(temp_u16),
                       lowByte(adc_volt), highByte(adc_volt),
                       lowByte(adc_amps), highByte(adc_amps), (byte) leaking_now, 0, 0};
    uint16_t chksm = CalcChksm(message, N_SERIAL_TX_BYTES-2);
    message[N_SERIAL_TX_BYTES-2] = chksm & 0xFF;
    message[N_SERIAL_TX_BYTES-1] = chksm >> 8;
    Serial.write(message, N_SERIAL_TX_BYTES);
  }
}
