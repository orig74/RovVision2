void setup() {
  DDRB = DDRB | B00001111;
  DDRC = DDRC | B00001111;
  TCCR0B = 0x01;  //No prescalars
  //Serial.begin(250000);
  Serial.begin(115200);
}

// get serial to write into this
//word throttle_vals[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //{1048, 1048, 1048, 1048, 1048, 1048, 1048, 1048};

word computeChecksum(word packet) {
  return (packet << 4) | (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xf; 
}

void loop() {
  static byte portB_buff[16];
  static byte portC_buff[16];
  static char serial_buff[16];

  while (!Serial.read() == 0b10010000) {}  // Keep reading bytes until the start nibble is found
  // wait for full message and transfer following bytes to DShot registers:
  Serial.readBytes(serial_buff, 16);
  // PORTB and PORTC registers, bits 1->4
  for (int buff_idx = 0; buff_idx < 16; buff_idx++) {
    portB_buff[buff_idx] = 0b11110000 | (serial_buff[buff_idx] >> 4); 
    portC_buff[buff_idx] = 0b11110000 | serial_buff[buff_idx]; 
  }

  
  while (true){
    delayMicroseconds(10);
    for (int frame_bit = 0; frame_bit < 16; frame_bit++) {
      while (TCNT0 < 0x66) {}
      TCNT0 = 0x00;
      PORTB = 0b11111111;
      PORTC = 0b11111111;
      while (TCNT0 < 0x1A) {}
      PORTB &= portB_buff[frame_bit];
      PORTC &= portC_buff[frame_bit];
      while (TCNT0 < 0x4B) {}
      PORTB = 0b00000000;
      PORTC = 0b00000000;
    }
    if (Serial.available() > 16) break;
    delayMicroseconds(10);
  }
}

