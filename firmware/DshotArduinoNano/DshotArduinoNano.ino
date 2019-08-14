#define START_ZERO_C 0x1A
#define ZERO_ONE_C 0x4B
#define BIT_C 0x66
#define SERIAL_MSG_L 17
#define DSHOT_TIMEOUT_C 20000
#define SERIAL_MSG_START_B 0b10010000

void setup() {
  DDRB = DDRB | B00001111;
  DDRC = DDRC | B00001111;
  TCCR0B = 0x01;  //No prescalars
  //Serial.begin(250000);
  Serial.begin(115200);
}


void loop() {
  static byte portB_buff[16];
  static byte portC_buff[16];
  static char serial_buff[16];
  long serial_timeout;

  while (Serial.read() != SERIAL_MSG_START_B) {}  // Keep reading bytes until the start nibble is found
  // wait for full message and transfer following bytes to DShot registers:
  Serial.readBytes(serial_buff, 16);
  serial_timeout = 0;
  // PORTB and PORTC registers, bits 1->4
  for (int buff_idx = 0; buff_idx < 16; buff_idx++) {
    portB_buff[buff_idx] = 0b11110000 | (serial_buff[buff_idx] >> 4); 
    portC_buff[buff_idx] = 0b11110000 | serial_buff[buff_idx]; 
  }
 
  while (Serial.available() < SERIAL_MSG_L){
    delayMicroseconds(10);
    for (int frame_bit = 0; frame_bit < 16; frame_bit++) {
      while (TCNT0 < BIT_C) {}
      TCNT0 = 0x00;
      PORTB = 0b11111111;
      PORTC = 0b11111111;
      while (TCNT0 < START_ZERO_C) {}
      PORTB &= portB_buff[frame_bit];
      PORTC &= portC_buff[frame_bit];
      while (TCNT0 < ZERO_ONE_C) {}
      PORTB = 0b00000000;
      PORTC = 0b00000000;
    }
    if (++serial_timeout > DSHOT_TIMEOUT_C) break;
    delayMicroseconds(10);
  }
}
