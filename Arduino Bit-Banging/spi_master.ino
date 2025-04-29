/*
以SPI模擬暫存器讀寫
Master模式
CPOL=0, CPHA=0
*/

#define MOSI D5
#define MISO D6
#define SCK  D7
#define NSS   D8

void spi_delay() { // delay
  delayMicroseconds(100);
}

void spi_start() { // 拉低NSS，開始通訊
  digitalWrite(NSS, LOW);
  spi_delay();
}

void spi_stop() { // 拉高NSS，結束通訊
  digitalWrite(NSS, HIGH);
  spi_delay();
}

uint8_t spi_TransmitReceive_byte(uint8_t data_write) { // 同時收發 1 byte data
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  uint8_t data_read = 0;
  for (int i = 0; i < 8; i++) { // 收發data
    data_read <<= 1;
    digitalWrite(MOSI, (data_write & 0x80) ? HIGH : LOW);
    data_write <<= 1;
    digitalWrite(SCK, HIGH);
    spi_delay();
    if (digitalRead(MISO)) data_read |= 1;
    digitalWrite(SCK, LOW);
    spi_delay();
  }
  return data_read;
}

void setup() {
  Serial.begin(9600);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(NSS, OUTPUT);
  digitalWrite(SCK, LOW);
  digitalWrite(NSS, HIGH);
  Serial.println("\n\n\n\n\nSPI Master Ready.");
  Serial.println("Type: w <device> <reg> <value> OR r <device> <reg>");

}

void loop() {
  if (Serial.available()){ // 若序列埠監控視窗有內容輸入
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("w")){ // 若輸入開頭為w，代表為寫入指令
      int dev, reg;
      unsigned int val;
      sscanf(cmd.c_str(), "w %d %d %x", &dev, &reg, &val);

      spi_start(); // 拉低NSS，開始通訊
      spi_TransmitReceive_byte((reg << 1) | 0); // 傳送暫存器位址 + rw bit
      spi_TransmitReceive_byte((uint8_t)val); // 傳送欲寫入的值
      spi_stop(); // 拉高NSS，結束通訊
      Serial.print("\nWrote to slave ");
      Serial.print(dev);
      Serial.print(", reg ");
      Serial.print(reg);
      Serial.print(": 0x");
      Serial.println(val, HEX);
    }

    if (cmd.startsWith("r")){ // 若輸入開頭為r，代表為讀取指令
      int dev, reg;
      unsigned int val;
      sscanf(cmd.c_str(), "r %d %d", &dev, &reg);

      spi_start(); // 拉低NSS，開始通訊
      spi_TransmitReceive_byte((reg << 1) | 1); // 傳送暫存器位址 + rw bit
      uint8_t data = spi_TransmitReceive_byte(0); // 接收欲讀取的值
      spi_stop(); // 拉高NSS，結束通訊
      Serial.print("\nRead from slave ");
      Serial.print(dev);
      Serial.print(", reg ");
      Serial.print(reg);
      Serial.print(": 0x");
      Serial.println(data, HEX);
    }
  }

}
