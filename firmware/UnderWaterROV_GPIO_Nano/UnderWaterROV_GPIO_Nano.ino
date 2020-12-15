#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

#define LIGHTS_PWM_PIN 9
#define INDICATOR_LED 5
#define TRIGER_PIN 6
#define CURRENT_ADC_PIN A1
#define VOLTAGE_ADC_PIN A0

//Camera trigger stuff
#define TRIG_FPS_DEFAULT 10
#define SENSOR_UPDATE_FPS 5
#define SENSOR_EVENT_MICROS (1000000/SENSOR_UPDATE_FPS)

#define SERIAL_BAUD_RATE 115200
#define SERIAL_TX_DELAY 500000


Servo Lights;
MS5837 DepthSensor;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();
  pinMode(INDICATOR_LED, OUTPUT);
  pinMode(TRIGER_PIN, OUTPUT);
  Lights.attach(LIGHTS_PWM_PIN, 1100, 1900);
  Lights.writeMicroseconds(1100);

  while (!DepthSensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(1000);
  }
  DepthSensor.setModel(MS5837::MS5837_30BA);
  DepthSensor.setFluidDensity(1029); // kg/m^3 (997 for freshwater, 1029 for seawater)
}

void loop() {
  static boolean rec, start_trig, trigger_state;
  static float light_power, prev_light_power;
  static unsigned long prev_trig_event_us, prev_sensor_event_us;
  static byte cur_state;
  static int adc_volt;
  static int adc_amps;
  static unsigned long trigger_event_us;
  byte bt;

  if (trigger_event_us == 0) trigger_event_us = (unsigned long) (500000 / TRIG_FPS_DEFAULT);

  // SERIAL RX
  while (Serial.available() > 0) {
      bt = Serial.read();

      // Update trigger fps
      if (bt > 10 && bt < 64) trigger_event_us = 500000 / (unsigned long) (bt - 10);

      switch(bt) {
          case 7:
              light_power = 1;
              break;
          case 6:
              light_power = 0.8;
              break;
          case 5:
              light_power = 0.6;
              break;
          case 4:
              light_power = 0.4;
              break;
          case 3:
              light_power = 0.2;
              break;
          case 2:
              light_power = 0;
              break;
          case 1:
              start_trig = true;
              break;
          case 0:
              start_trig = false;
              break;
          default:
              break;
      }
  }

  // Update light PWM
  if (light_power != prev_light_power) {
    Lights.writeMicroseconds(1100 + (int) round(800*light_power));
    prev_light_power = light_power;
  }

  // Task to trigger cameras
  if ((micros() - prev_trig_event_us) >= trigger_event_us) {
      prev_trig_event_us += trigger_event_us;
 
      if (!trigger_state && true) { //start_trig
          // trigger low and currently triggering
          digitalWrite(TRIGER_PIN, HIGH);
          digitalWrite(INDICATOR_LED, HIGH);
          trigger_state = true;
      }
      else {
          digitalWrite(TRIGER_PIN, LOW);
          digitalWrite(INDICATOR_LED, LOW);

          trigger_state = false;
      }
  }

  // Sensor read task
  if((micros() - prev_sensor_event_us) >= SENSOR_EVENT_MICROS) {
    prev_sensor_event_us += SENSOR_EVENT_MICROS;

    DepthSensor.read();
    float depth_m = DepthSensor.depth();
    float temp_c = DepthSensor.temperature();
    uint16_t depth_u16 = (uint16_t) min(max(round(depth_m*200), 0), 65536);
    uint16_t temp_u16 = (uint16_t) min(max(round(temp_c*200), 0), 65536);
    adc_volt = analogRead(VOLTAGE_ADC_PIN);
    adc_amps = analogRead(CURRENT_ADC_PIN);
    byte message[9] = {255, lowByte(depth_u16), highByte(depth_u16),
                            lowByte(temp_u16), highByte(temp_u16),
                            lowByte(adc_volt), highByte(adc_volt),
                            lowByte(adc_amps), highByte(adc_amps)};
    Serial.write(message, 9);
  }
}
