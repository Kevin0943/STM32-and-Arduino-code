/*
以I2C模擬暫存器讀寫
Master模式
*/

#define SDA_PIN D4
#define SCL_PIN D3

void i2c_delay() { // delay
  delayMicroseconds(50);
}

void i2c_start() { // 產生start bit
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  i2c_delay();
  digitalWrite(SDA_PIN, LOW);
  i2c_delay();
  digitalWrite(SCL_PIN, LOW);
  i2c_delay();
}

void i2c_stop() { // 產生stop bit
  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, HIGH);
  i2c_delay();
  digitalWrite(SDA_PIN, HIGH);
  i2c_delay();
}

bool i2c_write_byte(uint8_t data) { // 傳送 1 byte data
  pinMode(SDA_PIN, OUTPUT);
  for (int i = 0; i < 8; i++) { // 傳送data
    digitalWrite(SDA_PIN, (data & 0x80) ? HIGH : LOW);
    data <<= 1;
    digitalWrite(SCL_PIN, HIGH);
    i2c_delay();
    digitalWrite(SCL_PIN, LOW);
    i2c_delay();
  }

  pinMode(SDA_PIN, INPUT);
  digitalWrite(SDA_PIN, HIGH); // pull-up，交由Slave控制SDA

  i2c_delay();
  digitalWrite(SCL_PIN, HIGH);
  i2c_delay();
  bool ack = digitalRead(SDA_PIN) == LOW; // 接收ACK or NACK
  digitalWrite(SCL_PIN, LOW);
  i2c_delay();
  return ack;
}

uint8_t i2c_read_byte(bool send_ack) { // 接收 1 byte data
  uint8_t data = 0;
  pinMode(SDA_PIN, INPUT);
  for (int i = 0; i < 8; i++) { // 接收data
    data <<= 1;
    digitalWrite(SCL_PIN, HIGH);
    i2c_delay();
    if (digitalRead(SDA_PIN)) data |= 1;
    digitalWrite(SCL_PIN, LOW);
    i2c_delay();
  }

  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, send_ack ? LOW : HIGH); // 回傳ACK or NACK
  i2c_delay();
  digitalWrite(SCL_PIN, HIGH);
  i2c_delay();
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, INPUT);
  i2c_delay();

  return data;
}

void setup() {
  Serial.begin(9600);
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  Serial.println("\n\n\n\n\nI2C Master Ready.");
  Serial.println("Type: w <device> <reg> <value> OR r <device> <reg>");
}

void loop() {
  if (Serial.available()) { // 若序列埠監控視窗有內容輸入
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("w")) { // 若輸入開頭為w，代表為寫入指令
      int dev, reg;
      unsigned int val;
      sscanf(cmd.c_str(), "w %d %d %x", &dev, &reg, &val);

      i2c_start(); // 產生start bit
      bool b = i2c_write_byte((dev << 1) | 0); // 傳送設備位址 + rw bit
      i2c_write_byte(reg); // 傳送暫存器位址
      i2c_write_byte((uint8_t)val); // 傳送欲寫入的值
      i2c_stop(); // 產生stop bit

      if(b){ // 若傳送設備位址後收到ACK
      Serial.print("\nWrote to slave ");
      Serial.print(dev);
      Serial.print(", reg ");
      Serial.print(reg);
      Serial.print(": 0x");
      Serial.println(val, HEX);
      }
      else{ // 若傳送設備位址後收到NACK
        Serial.print("\nSlave ");
        Serial.print(dev);
        Serial.print(" is not found\n");
      }
    }

    if (cmd.startsWith("r")) { // 若輸入開頭為r，代表為讀取指令
      int dev, reg;
      sscanf(cmd.c_str(), "r %d %d", &dev, &reg);

      i2c_start(); // 產生start bit
      bool b = i2c_write_byte((dev << 1) | 1); // 傳送設備位址 + rw bit
      i2c_write_byte(reg); // 傳送暫存器位址
      uint8_t data = i2c_read_byte(false); // 接收欲讀取的值
      i2c_stop(); // 產生stop bit

      if(b){ // 若傳送設備位址後收到ACK
      Serial.print("\nRead from slave ");
      Serial.print(dev);
      Serial.print(", reg ");
      Serial.print(reg);
      Serial.print(": 0x");
      Serial.println(data, HEX);
      }
      else{ // 若傳送設備位址後收到NACK
        Serial.print("\nSlave ");
        Serial.print(dev);
        Serial.print(" is not found\n");
      }
    }
  }
}
