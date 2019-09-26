#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

#define LIGHTS_PWM_PIN 9
#define INDICATOR_LED 5
#define TRIGER_PIN 6
#define CURRENT_ADC_PIN A1
#define VOLTAGE_ADC_PIN A0

//Camera trigger stuff
#define TRIG_FPS 10
#define TRIGGER_RATE_MICROS (1000000/TRIG_FPS)
#define TRIGGER_RATE_MICROS_HALF (TRIGGER_RATE_MICROS/2)

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

unsigned int cnt1=0;
unsigned int cnt2=0;

void loop() {
  static boolean rec, start_trig, trigger_state;
  static float light_power;
  static unsigned long prev_trigger_micros;
  static unsigned long prev_serial_tx_micros;
  static byte cur_state;
  static int adc_volt;
  static int adc_amps; 

  byte bt;
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

  // Task to trigger cameras
  if ((micros() - prev_trigger_micros) > TRIGGER_RATE_MICROS_HALF) {
      prev_trigger_micros += TRIGGER_RATE_MICROS_HALF;
 
      if (!trigger_state && true) { //start_trig
          // trigger low and currently triggering
          digitalWrite(TRIGER_PIN, HIGH);
          digitalWrite(INDICATOR_LED, HIGH);
          trigger_state = true;

          if(cnt1%2==0){
            DepthSensor.read();
            float depth_m = DepthSensor.depth();
            byte depth_byte = (byte) min(max(round(depth_m*10), 0), 255);
            byte messege[6] = {255, depth_byte, lowByte(adc_volt), highByte(adc_volt), lowByte(adc_amps), highByte(adc_amps};
            Serial.write(messege, 6);)
          }
          cnt1++;
      }
      else {
          // trigger high (always bring lines low even if trigger has been turned off)
          if (cnt2%10==0)
            adc_volt = analogRead(VOLTAGE_ADC_PIN);
            adc_amps = analogRead(CURRENT_ADC_PIN);
          digitalWrite(TRIGER_PIN, LOW);
          digitalWrite(INDICATOR_LED, LOW);
          Lights.writeMicroseconds(1100 + (int) round(800*light_power));

          trigger_state = false;
          cnt2++;
      }


  }


}
