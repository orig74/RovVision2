#include <EEPROM.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <driver/adc.h>
#include "MS5837.h"
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 3 // seconds
#define NODATA_TIMEOUT_LOOPS 100

#define STATUS_LED_PIN 2
#define GRIPPER_PIN 16
#define GRIPPER_ROT_PIN 17
int thruster_pins[8] = {12, 13, 14, 27, 25, 26, 33, 32};
int thruster_dirs[8] = {-1, -1, 1, 1, -1, 1, 1, -1};
#define CURRENT_ADC_PIN 15
#define VOLTAGE_ADC_PIN 4
#define MOISTURE_SENSE_PIN 5
#define THRUSTERS_CURRENT_PIN_1 35
#define THRUSTERS_CURRENT_PIN_2 34

#define SENSOR_UPDATE_FPS 10
#define SENSOR_EVENT_MICROS (1000000/SENSOR_UPDATE_FPS)

#define SERIAL_BAUDRATE 500000
#define MAINLOOP_PERIOD_US 1000

#define N_SERIAL_TX_BYTES 15
#define N_SERIAL_RX_BYTES 20
byte msg_buff[N_SERIAL_RX_BYTES];
byte serial_count;

Servo thrusters[8];
#define PWM_MIDPOINT 1488
#define PWM_RANGE_H 500
Servo gripper;
Servo gripper_rot;
int servo_offset = 0;

unsigned long prev_sensor_event_us;
unsigned long prev_loop_us;
unsigned long prev_led_toggle_ms;
unsigned long no_data_cnt;
bool disconnected = false;

MS5837 DepthSensor;
bool depth_sensor_ok=false;


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
  return (chksm_calc == msg_chksum) && msg_chksum;
}

int ScaleUint16(uint16_t in_val, int out_min, int out_max) {
  return out_min + (int) ((float) (out_max - out_min) * (float) in_val / 65535);
}

int sample_adc(int PIN, int num_s) {
  long adc_val=0;
  for (int s=0; s<num_s; s++) adc_val += (long) analogRead(PIN);
  return (int) (adc_val / num_s);
}


void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(THRUSTERS_CURRENT_PIN_1, INPUT);
  pinMode(THRUSTERS_CURRENT_PIN_2, INPUT);
  pinMode(CURRENT_ADC_PIN, INPUT);
  pinMode(VOLTAGE_ADC_PIN, INPUT);
  pinMode(MOISTURE_SENSE_PIN, INPUT);

  gripper.attach(GRIPPER_PIN, 1100, 1900);
  gripper.writeMicroseconds(1900);
  gripper_rot.attach(GRIPPER_ROT_PIN);
  gripper_rot.writeMicroseconds(1418);
  for (int t_ind = 0; t_ind < 8; t_ind++) {
    thrusters[t_ind].attach(thruster_pins[t_ind]);
    thrusters[t_ind].writeMicroseconds(PWM_MIDPOINT);
  }
  
  Serial.begin(SERIAL_BAUDRATE);
  Wire.begin();

  esp_task_wdt_reset();
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
  digitalWrite(STATUS_LED_PIN, HIGH);
}

unsigned long loop_itt=0;
void loop() {
  esp_task_wdt_reset();

  // INIT Depth sensor if not already
  if (!depth_sensor_ok and loop_itt%500==0) {
    if (DepthSensor.init()) {
      depth_sensor_ok = true;
      DepthSensor.setModel(MS5837::MS5837_30BA);
      DepthSensor.setFluidDensity(1029); // kg/m^3 (997 for freshwater, 1029 for seawater)
    }
  }

  while (micros() < prev_loop_us + MAINLOOP_PERIOD_US) {}
  prev_loop_us = micros();
  loop_itt++;

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
      disconnected = false;
      
      // Thrusters
      for (int thrstr_idx=0; thrstr_idx < 8; thrstr_idx++) {
        uint16_t thrst_16b = unpack_16bit(msg_buff, 2 * thrstr_idx);
        int thrst_int = ScaleUint16(thrst_16b, -PWM_RANGE_H, PWM_RANGE_H);
        int thrst_us = thruster_dirs[thrstr_idx] * thrst_int + PWM_MIDPOINT;
        thrusters[thrstr_idx].writeMicroseconds(thrst_us);
      }
      
      // Gripper
      byte gripper_8b = msg_buff[16];
      if (gripper_8b > 127)
          gripper.writeMicroseconds(1100);
      else
          gripper.writeMicroseconds(1900);
      byte servo_8b = msg_buff[17];
      unsigned int servo_us = 1020 + (unsigned int) (800 * (float) servo_8b / 255);
      gripper_rot.writeMicroseconds(servo_us);

      // Clear input buffer
      for (int i=0; i < N_SERIAL_RX_BYTES; i++) msg_buff[i] = '\0';
      serial_count = 0;
    } else {
      //Serial.println("INVALID_CHKSM");
      no_data_cnt++;
    }
  } else no_data_cnt++;

  // If no serial messages for N loops write outputs to OFF
  if (no_data_cnt > NODATA_TIMEOUT_LOOPS) {
    no_data_cnt = 0;
    disconnected = true;
    for (int thrstr_idx=0; thrstr_idx < 8; thrstr_idx++) {
        thrusters[thrstr_idx].writeMicroseconds(PWM_MIDPOINT);
    }
  }

  // Blue status LED and thruster init
  if (disconnected) {
      if (millis() - prev_led_toggle_ms > 1000) {
          prev_led_toggle_ms = millis();
          digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
      }   
  } else {
	digitalWrite(STATUS_LED_PIN, HIGH);
  }

  // Sensors read task 
  if((micros() - prev_sensor_event_us) > SENSOR_EVENT_MICROS) {
    prev_sensor_event_us = micros();

    // Read depth sensor if available
    float depth_m = -1;
    float temp_c = -1;
    if (depth_sensor_ok) {
      DepthSensor.read();
      depth_m = DepthSensor.depth();
      temp_c = DepthSensor.temperature();
    }
    uint16_t depth_u16 = (uint16_t) min(max((int) round(depth_m*200), 0), 65536);
    uint16_t temp_u16 = (uint16_t) min(max((int) round(temp_c*200), 0), 65536);
    int adc_volt = sample_adc(VOLTAGE_ADC_PIN, 5);
    int adc_amps = sample_adc(CURRENT_ADC_PIN, 5);
    int adc_amps_esc1 = sample_adc(THRUSTERS_CURRENT_PIN_1, 5);
    int adc_amps_esc2 = sample_adc(THRUSTERS_CURRENT_PIN_2, 5);
    bool leaking_now = digitalRead(MOISTURE_SENSE_PIN);

    byte message[N_SERIAL_TX_BYTES] = {lowByte(depth_u16), highByte(depth_u16),
                                       lowByte(temp_u16), highByte(temp_u16),
                                       lowByte(adc_volt), highByte(adc_volt),
                                       lowByte(adc_amps), highByte(adc_amps), 
                                       lowByte(adc_amps_esc1), highByte(adc_amps_esc1),
                                       lowByte(adc_amps_esc2), highByte(adc_amps_esc2), 
                                       (byte) leaking_now, 0, 0};
    uint16_t chksm = CalcChksm(message, N_SERIAL_TX_BYTES-2);
    message[N_SERIAL_TX_BYTES-2] = chksm & 0xFF;
    message[N_SERIAL_TX_BYTES-1] = chksm >> 8;
    Serial.write(message, N_SERIAL_TX_BYTES);
  }
}
