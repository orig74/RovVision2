#define SERIAL_MSG_L 17
#define DSHOT_TIMEOUT_C 20000
#define SERIAL_MSG_START_B 0b10010000

const uint16_t short_pulse = uint64_t(F_BUS) * 625 / 1000000000;
const uint16_t long_pulse = uint64_t(F_BUS) * 1250 / 1000000000;
const uint16_t bit_length = uint64_t(F_BUS) * 1670 / 1000000000;

void setup() {
  DDRD = B11111111;
  FTM0_CNT = 0;
  FTM0_SC = FTM_SC_CLKS(1);
  Serial.begin(115200);
}


void loop() {
  static byte portD_buff[16];
  static char serial_buff[16];
  static unsigned long serial_timeout;

  while ((Serial.read() & 0b11110000) != SERIAL_MSG_START_B) {}  // Keep reading bytes until the start nibble is found
  // wait for full message and transfer following bytes to DShot registers:
  Serial.readBytes(serial_buff, 16);
  serial_timeout = 0;
  // PORTB and PORTC registers, bits 1->4
  for (int buff_idx = 0; buff_idx < 16; buff_idx++) {
    portD_buff[buff_idx] = serial_buff[buff_idx]; 
  }
 
  while (Serial.available() < SERIAL_MSG_L){
    delayMicroseconds(20);
    for (int frame_bit = 0; frame_bit < 16; frame_bit++) {
      while (FTM0_CNT < bit_length) {}
      FTM0_CNT = 0x00;
      PORTD = 0b11111111;
      while (FTM0_CNT < short_pulse) {}
      PORTD = portD_buff[frame_bit];
      while (FTM0_CNT < long_pulse) {}
      PORTD = 0b00000000;
    }
    if (serial_timeout++ > DSHOT_TIMEOUT_C) break;
  }
}
