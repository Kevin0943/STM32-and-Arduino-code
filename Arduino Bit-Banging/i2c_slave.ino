/*
以I2C模擬暫存器讀寫
Slave模式
*/

#define SDA_PIN D4
#define SCL_PIN D3
#define Slave_number 0

uint8_t registers[3] = {1, 2, 3}; // 3個模擬暫存器

bool wait_for_clock_high() { // 等待SCL被拉高
  unsigned long start = micros();
  while (digitalRead(SCL_PIN) == LOW) {
    if (micros() - start > 2000){ // 若超時則跳出
      Serial.println("wait_for_clock_high ERROR");
      return false;
    }
  }
  return true;
}

bool wait_for_clock_low() { // 等待SCL被拉低
  unsigned long start = micros();
  while (digitalRead(SCL_PIN) == HIGH) {
    if (micros() - start > 2000){ // 若超時則跳出
      Serial.println("wait_for_clock_low ERROR");
      return false;
    }
  }
  return true;
}

uint8_t read_byte() { // 接收 1 byte data
  uint8_t data = 0;
  pinMode(SDA_PIN, INPUT);
  for (int i = 0; i < 8; i++) { // 接收data
    if (!wait_for_clock_high()) break;
    data <<= 1;
    if (digitalRead(SDA_PIN)) data |= 1;
    if (!wait_for_clock_low()) break;
  }
  return data;
}

void send_ack() { // 回傳ACK
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  wait_for_clock_high();
  wait_for_clock_low();
  pinMode(SDA_PIN, INPUT);
}

void send_byte(uint8_t data) { // 傳送 1 byte data
  pinMode(SDA_PIN, OUTPUT);

  for (int i = 0; i < 8; i++) { // 傳送data
    digitalWrite(SDA_PIN, (data & 0x80) ? HIGH : LOW);
    data <<= 1;
    wait_for_clock_high();
    wait_for_clock_low();
  }

  pinMode(SDA_PIN, INPUT);
  wait_for_clock_high(); // 等待Master回傳NACK
  wait_for_clock_low();
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
  pinMode(SCL_PIN, INPUT);
  pinMode(SDA_PIN, INPUT);
  Serial.begin(9600);
  Serial.println("\n\n\n\n\nI2C Slaves ready.");
  print_registers();
}

void loop() {
  static bool prevSDA = HIGH;
  bool currSDA = digitalRead(SDA_PIN);
  bool currSCL = digitalRead(SCL_PIN);

  if (prevSDA == HIGH && currSDA == LOW && currSCL == HIGH) { // 若偵測到start bit
    wait_for_clock_low();
    uint8_t addr_rw = read_byte(); // 接收設備位址 + rw bit
    bool is_read = addr_rw & 1;
    uint8_t addr = addr_rw >> 1;

    if (addr == Slave_number) { // 若接收的設備位址和自己符合
      send_ack();

      uint8_t reg = read_byte(); // 接收暫存器位址
      send_ack();

      if (is_read) { // 若收到的是讀取指令
          int8_t regIndex = -1;

          if (reg == 0x00) regIndex = 0;
          else if (reg == 0x01) regIndex = 1;
          else if (reg == 0x02) regIndex = 2;

          if (regIndex >= 0 && regIndex < 3) { // 若接收的暫存器位址合法
            send_byte(registers[regIndex]); // 回傳暫存器之值
            Serial.print("\n\n\n\n\nRead from device ");
            Serial.print(addr);
            Serial.print(", register ");
            Serial.print(reg, HEX);
            Serial.print(" → 0x");
            Serial.println(registers[regIndex], HEX);
            print_registers();
          }
          else { // 若接收的暫存器位址非法
            Serial.print("\nReg No\n");
            send_byte(0xDD);
          }
      }
        else { // 若收到的是寫入指令
        uint8_t data = read_byte(); // 接收欲寫入的值
        send_ack();
        if (reg < 3 && reg >= 0) { // 若接收的暫存器位址合法
          registers[reg] = data; // 寫入暫存器
          Serial.print("\n\n\n\n\nWrite to device ");
          Serial.print(addr);
          Serial.print(", reg ");
          Serial.print(reg);
          Serial.print(": 0x");
          Serial.println(data, HEX);
          print_registers();
        }
      }
    }
    else{ // 若接收的設備位址和自己不符合，則空等不干涉SDA
      for(int i=0; i<19; i++){
        wait_for_clock_high();
        wait_for_clock_low();
      }
    }
  }

  prevSDA = currSDA;
}
