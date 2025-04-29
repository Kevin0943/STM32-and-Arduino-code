/*
以UART模擬暫存器讀寫
Master模式
*/

#define TX_PIN D5
#define RX_PIN D6
#define BIT_DURATION_US 104

void setup() {
  Serial.begin(9600);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  digitalWrite(TX_PIN, HIGH);  // TX閒置時為High
  Serial.println("\n\n\n\n\nUART Master Ready.");
  Serial.println("Type: w <device> <reg> <value> OR r <device> <reg>");
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
  // while (digitalRead(RX_PIN) == HIGH);// Wait start bit
  wait_for_RX_low(); // 等待對方產生start bit
  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);
  uint8_t val = 0;
  for (int i = 0; i < 8; i++) { // 接收data
    val |= (digitalRead(RX_PIN) << i);
    delayMicroseconds(BIT_DURATION_US);
  }
  wait_for_RX_high(); // 等待對方產生stop bit
  delayMicroseconds(BIT_DURATION_US);// Let stop bit go
  return val;
}

void loop() {
  if (Serial.available()){ // 若序列埠監控視窗有內容輸入
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("w")){ // 若輸入開頭為w，代表為寫入指令


      int reg;
      unsigned int val;
      sscanf(cmd.c_str(), "w %d %x", &reg, &val);

      uart_send_byte((reg << 1) | 0); // 傳送暫存器位址 + rw bit
      uart_send_byte(val); // 傳送欲寫入的值
      Serial.print("\nWrote to reg ");
      Serial.print(reg);
      Serial.print(": 0x");
      Serial.println(val, HEX);
    }

    if (cmd.startsWith("r")){ // 若輸入開頭為r，代表為讀取指令
      int reg;
      unsigned int val;
      sscanf(cmd.c_str(), "r %d", &reg);

      uart_send_byte((reg << 1) | 1); // 傳送暫存器位址 + rw bit
      uint8_t data = uart_receive_byte(); // 接收欲讀取的值
      Serial.print("\nRead from reg ");
      Serial.print(reg);
      Serial.print(": 0x");
      Serial.println(data, HEX);
    }
  }
}
