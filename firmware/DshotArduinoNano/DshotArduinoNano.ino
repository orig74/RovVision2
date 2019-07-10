void setup() {
  DDRB = DDRB | B11111111;
  DDRC = DDRC | B11111111;
  TCCR0B = 0x01;  //No prescalars
  Serial.begin(115200);
}

// get serial to write into this
word throttle_vals[8] = {1048, 1048, 1048, 1048, 1048, 1048, 1048, 1048}; //{0, 0, 0, 0, 0, 0, 0, 0};

word computeChecksum(word packet) {
  return (packet << 4) | (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xf; 
}

void fillDShotReg(byte* reg, word* messages) {
  for (int frame_idx = 0; frame_idx < 16; frame_idx++){
       *(reg + frame_idx) = bitRead(messages[0], frame_idx) | 
                            bitRead(messages[1], frame_idx) << 1 |
                            bitRead(messages[2], frame_idx) << 2 |
                            bitRead(messages[3], frame_idx) << 3 | 
                            bitRead(messages[4], frame_idx) << 4 | 
                            bitRead(messages[5], frame_idx) << 5 | 
                            bitRead(messages[6], frame_idx) << 6 | 
                            bitRead(messages[7], frame_idx) << 7;
  }
}

void loop() {
  static byte dshotREG[16];
  static word messages[8];

  if (Serial.available() > 2) {
    throttle_vals[0] = (word) Serial.parseInt();
    Serial.println(throttle_vals[0]);
  }
    
  for (int msg_idx = 0; msg_idx < 8; msg_idx++) {
    messages[msg_idx] = computeChecksum(throttle_vals[msg_idx] << 1);
  }
  fillDShotReg(dshotREG, messages);
  
  while (true){
    for (int frame_bit = 0; frame_bit < 16; frame_bit++) {
      while (TCNT0 < 0x66) {}
      TCNT0 = 0x00;
      PORTB = 0b11111111;
      PORTC = 0b11111111;
      while (TCNT0 < 0x1F) {}
      PORTB &= dshotREG[15 - frame_bit];
      PORTC &= dshotREG[15 - frame_bit] >> 6;
      while (TCNT0 < 0x4C) {}
      PORTB = 0b00000000;
      PORTC = 0b00000000;
      TCNT2 = 0x00;
    }
    if (Serial.available()) break;
    delayMicroseconds(2000);
  }
}

