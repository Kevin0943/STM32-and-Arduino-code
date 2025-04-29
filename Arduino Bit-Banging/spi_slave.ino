/*
以SPI模擬暫存器讀寫
Slave模式
CPOL=0, CPHA=0
*/

#define MOSI D5
#define MISO D6
#define SCK  D7
#define NSS   D8
#define Slave_number 0

uint8_t registers[3] = {1, 2, 3}; // 3個模擬暫存器

bool wait_for_clock_high() { // 等待SCK被拉高
  unsigned long start = micros();
  while (digitalRead(SCK) == LOW) {
    if (micros() - start > 2000){ // 若超時則跳出
      Serial.println("wait_for_clock_high ERROR");
      return false;
    }
  }
  return true;
}

bool wait_for_clock_low() { // 等待SCK被拉低
  unsigned long start = micros();
  while (digitalRead(SCK) == HIGH) {
    if (micros() - start > 2000){ // 若超時則跳出
      Serial.println("wait_for_clock_low ERROR");
      return false;
    }
  }
  return true;
}

uint8_t TransmitReceive_byte(uint8_t data_write) { // 同時收發 1 byte data
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  uint8_t data_read = 0;
  for (int i = 0; i < 8; i++) { // 收發data
    digitalWrite(MISO, (data_write & 0x80) ? HIGH : LOW);
    data_write <<= 1;
    if (!wait_for_clock_high()) break;
    data_read <<= 1;
    if (digitalRead(MOSI)) data_read |= 1;
    if (!wait_for_clock_low()) break;
  }
  return data_read;

}

void print_registers() { // 秀出所有暫存器的值
  Serial.println("Current registers:");
  Serial.print("Slave ");
  Serial.print(Slave_number);
  Serial.print(": [ ");
  for (int i = 0; i < 3; i++) {
      Serial.print(registers[i], HEX);
      Serial.print(" ");
  }
  Serial.println("]");
}

void setup() {
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  Serial.begin(9600);
  Serial.println("\n\n\n\n\nSPI Slaves ready.");
  print_registers();
}

void loop() {
  if(!digitalRead(NSS)){ // 若NSS被拉低
    wait_for_clock_low(); // 確認SCK為Low (SCK閒置時應為Low)
    uint8_t reg_rw = TransmitReceive_byte(0); // 接收暫存器位址 + rw bit
    bool is_read = reg_rw & 1;
    uint8_t reg = reg_rw >> 1;

    if (is_read) { // 若收到的是讀取指令
      if (reg >= 0 && reg < 3) { // 若接收的暫存器位址合法
        TransmitReceive_byte(registers[reg]); // 回傳暫存器之值
        Serial.print("\n\n\n\n\nRead from device ");
        Serial.print(Slave_number);
        Serial.print(", register ");
        Serial.print(reg, HEX);
        Serial.print(" → 0x");
        Serial.println(registers[reg], HEX);
        print_registers();
      }
      else { // 若接收的暫存器位址非法
        Serial.print("\nReg No\n");
        TransmitReceive_byte(0xDD); // 無效暫存器
      }
    }
    else { // 若收到的是寫入指令
      uint8_t data = TransmitReceive_byte(0); // 接收欲寫入的值
      if (reg >= 0 && reg < 3) { // 若接收的暫存器位址合法
        registers[reg] = data; // 寫入暫存器
        Serial.print("\n\n\n\n\nWrite to device ");
        Serial.print(Slave_number);
        Serial.print(", reg ");
        Serial.print(reg);
        Serial.print(": 0x");
        Serial.println(data, HEX);
        print_registers();
      }
    }
  }
}
