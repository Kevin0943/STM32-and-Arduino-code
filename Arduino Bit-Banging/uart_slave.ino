/*
以UART模擬暫存器讀寫
Slave模式
*/

#define TX_PIN D5
#define RX_PIN D6
#define BIT_DURATION_US 104

uint8_t registers[3] = {1, 2, 3}; // 3個模擬暫存器

void setup() {
  Serial.begin(9600);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  digitalWrite(TX_PIN, HIGH);  // TX閒置時為High
  Serial.println("\n\n\n\n\nUART Slaves ready.");
  print_registers();
}

bool wait_for_RX_high() { // 等待RX被拉高
  unsigned long start = micros();
  while (digitalRead(RX_PIN) == LOW) {
    if (micros() - start > 2000){ // 若超時則跳出
      Serial.println("wait_for_clock_high ERROR");
      return false;
    }
  }
  return true;
}

bool wait_for_RX_low() { // 等待RX被拉低
  unsigned long start = micros();
  while (digitalRead(RX_PIN) == HIGH) {
    if (micros() - start > 2000){ // 若超時則跳出
      Serial.println("wait_for_clock_low ERROR");
      return false;
    }
  }
  return true;
}

void uart_send_byte(uint8_t b) { // 傳送 1 byte data
  digitalWrite(TX_PIN, LOW); // start bit
  delayMicroseconds(BIT_DURATION_US);
  for (int i = 0; i < 8; i++) { // 傳送data
    digitalWrite(TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DURATION_US);
  }
  digitalWrite(TX_PIN, HIGH); // stop bit
  delayMicroseconds(BIT_DURATION_US);
}

uint8_t uart_receive_byte() { // 接收 1 byte data
  wait_for_RX_low(); // 等待對方產生start bit
  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);
  uint8_t val = 0;
  for (int i = 0; i < 8; i++) { // 接收data
    val |= (digitalRead(RX_PIN) << i);
    delayMicroseconds(BIT_DURATION_US);
  }
  wait_for_RX_high(); // 等待對方產生stop bit
  delayMicroseconds(BIT_DURATION_US);
  return val;
}

void print_registers() { // 秀出所有暫存器的值
  Serial.println("Current registers:");
  Serial.print("[ ");
  for (int i = 0; i < 3; i++) {
      Serial.print(registers[i], HEX);
      Serial.print(" ");
  }
  Serial.println("]");
}

void loop() {
  static bool prevRX = HIGH;
  bool currRx = digitalRead(RX_PIN);

  if(prevRX == HIGH && currRx == LOW){ // 若RX偵測到start bit
    uint8_t reg_rw = uart_receive_byte(); // 接收暫存器位址 + rw bit
    bool is_read = reg_rw & 1;
    uint8_t reg = reg_rw >> 1;
    if(is_read){ // 若收到的是讀取指令
      uart_send_byte(registers[reg]); // 回傳暫存器之值
      Serial.print("\n\n\n\n\nRead from reg ");
      Serial.print(reg, HEX);
      Serial.print(" → 0x");
      Serial.println(registers[reg], HEX);
      print_registers();
    }
    else{ // 若收到的是寫入指令
      uint8_t data = uart_receive_byte(); // 接收欲寫入的值
      registers[reg] = data; // 寫入暫存器
      Serial.print("\n\n\n\n\nWrite to reg ");
      Serial.print(reg);
      Serial.print(": 0x");
      Serial.println(data, HEX);
      print_registers();
    }
  }
  prevRX = currRx;
}
