#include <Wire.h>

byte buffer[4;
]
void setup() {
  // put your setup code here, to run once:
  Wire.begin(0x04);
  Wire.onReceive(receiveEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(buffer)
}

void receiveEvent(int numBytes) {
    uint8_t i = 0;
    while(Wire.available()){
      byte in = Wire.read();
      Serial.print(in);
      buffer[i] = in;
      i++;
      Serial.println(" ");
    }
  }
}
