#include <Servo.h> 
#include <Wire.h>
#include "MS5837.h"

#define LIGHTS_PWM_PIN 11
#define INDICATOR_LED 5
#define TRIGER_PIN 6
#define CURRENT_ADC_PIN A0
#define VOLTAGE_ADC_PIN A1

//Camera trigger stuff
#define TRIG_FPS 10
#define TRIGGER_RATE_MICROS (1000000/TRIG_FPS)
#define TRIGGER_RATE_MICROS_HALF (TRIGGER_RATE_MICROS/2)

#define SERIAL_BAUD_RATE 115200
#define SERIAL_TX_DELAY 500000

#define BATT_AMP_OFFSET 0.330
#define BATT_AMP_PERVOLT 37.8788
#define ADC_VOLTAGE_MUL 0.0048828
#define BATT_VOLT_MULT 11


Servo Lights;
MS5837 DepthSensor;


void setup() {
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();
  pinMode(INDICATOR_LED, OUTPUT);
  digitalWrite(INDICATOR_LED, HIGH);
  pinMode(TRIGER_PIN, OUTPUT);
  Lights.attach(LIGHTS_PWM_PIN, 1100, 1900);
  Lights.writeMicroseconds(1100);
  
  while (!DepthSensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(2000);
  }
  DepthSensor.setModel(MS5837::MS5837_30BA);
  DepthSensor.setFluidDensity(1029); // kg/m^3 (997 for freshwater, 1029 for seawater)
}


void loop() {
  static boolean rec, start_trig, trigger_state;
  static float light_power;
  static unsigned long prev_trigger_micros;
  static unsigned long prev_serial_tx_micros;
  
  byte bt;
  float adc_volt, adc_amps;
  byte batt_voltage, batt_current;
  unsigned long time_us = micros();
  
  // SERIAL RX
  while (Serial.available() > 0) {
      bt = Serial.read();
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
  
  //SERIAL TX
  if ((time_us - prev_serial_tx_micros) > SERIAL_TX_DELAY) {
    prev_serial_tx_micros = time_us;
    
    DepthSensor.read();
    float depth_m = DepthSensor.depth();
    byte depth_byte = (byte) min(max(round(depth_m*10), 0), 255);
    adc_volt = (float) analogRead(VOLTAGE_ADC_PIN);
    adc_amps = (float) analogRead(CURRENT_ADC_PIN);
    batt_voltage = (byte) min(round(10 * adc_volt * ADC_VOLTAGE_MUL * BATT_VOLT_MULT), 254);
    batt_current = (byte) min(round(10 * (adc_amps * ADC_VOLTAGE_MUL - BATT_AMP_OFFSET) * BATT_AMP_PERVOLT), 254);
    byte messege[4] = {255, depth_byte, batt_voltage, batt_current};
    Serial.write(messege, 4);
  }
  
  // Task to trigger cameras
  if ((micros() - prev_trigger_micros) > TRIGGER_RATE_MICROS_HALF) {
      prev_trigger_micros += TRIGGER_RATE_MICROS_HALF;

      if (!trigger_state && start_trig) {
          // trigger low and currently triggering
          digitalWrite(TRIGER_PIN, HIGH);
          trigger_state = true;
      }
      else {
          // trigger high (always bring lines low even if trigger has been turned off)
          digitalWrite(TRIGER_PIN, LOW);
          trigger_state = false;
      }
  }
  
  Lights.writeMicroseconds(1100 + (int) round(800*light_power));

}
