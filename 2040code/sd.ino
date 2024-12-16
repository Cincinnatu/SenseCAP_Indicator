#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define ADC0  26

const float Vref = 3.3;
int cnt = 0;

String SDDataString = "";

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '1') {
      digitalWrite(21, HIGH);
      Serial.println("Pin 21 set to HIGH");
    } else if (inChar == '2') {
      digitalWrite(21, LOW);
      Serial.println("Pin 21 set to LOW");
    }else if (inChar == '3') {
      digitalWrite(23, LOW);
      Serial.println("Pin 23 set to LOW");
    }else if (inChar == '4') {
      digitalWrite(23, HIGH);
      Serial.println("Pin 23 set to HIGH");
    }else if (inChar == '5') {
      digitalWrite(24, HIGH);
      Serial.println("Pin 24 set to HIGH");
    }else if (inChar == '6') {
      digitalWrite(24, LOW);
      Serial.println("Pin 24 set to LOW");
    }
     
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
      Serial.println("esp32:");
      Serial.println(inChar);
      digitalWrite(24, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  const int chipSelect = 13;
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);

  if (!SD.begin(chipSelect, 1000000, SPI1)) {
    Serial.println("Card failed, or not present");
  } else {
    Serial.println("card initialized.");
  }
  pinMode(21, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(22, INPUT);
  digitalWrite(21, HIGH);
  //digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);
  delay(3000);

}

void loop() {
  int state = digitalRead(22); // 读取引脚状态
  if (state == LOW) {
    Serial.println("Pin22 is LOW ");
  } else {
    Serial.println("Pin22 is HIGH ");
  }

   delay(1000);
  uint16_t adc0_data = analogRead(ADC0);
  float Voltage = adc0_data * (Vref / 1024);
  Serial.println(Voltage * 2);
  float adjustedVoltage = Voltage * 2; 
  if(adjustedVoltage <= 3 && state == LOW)
  {
    digitalWrite(24, LOW);
    Serial.println("Pin 24 set to LOW");

  }
  delay(1000);
  SDDataString = "";
  Serial.printf("\r\n\r\n--------- start measure %d-------\r\n", cnt);

  SDDataString += String(adjustedVoltage, 3);
  SDDataString += ',';

  cnt++;
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(SDDataString);
    dataFile.close();
    Serial.println(adjustedVoltage, 3);
    // print to the serial port too:
    Serial.print("sd write: ");
    Serial.println(SDDataString);
  } else {
    Serial.println("error opening datalog.txt");
  }

}
