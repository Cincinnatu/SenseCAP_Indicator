
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define ADC0  26

const float Vref = 3.3;
int cnt = 0;
int espiqr = 0;
String SDDataString = "";
int powerset = 0;

const int chipSelect = 13;

// void serialEvent() {
//   while (Serial.available()) {
//     char inChar = (char)Serial.read();
//     if (inChar == '1') {
//       digitalWrite(21, HIGH);
//       Serial.println("Pin 21 set to HIGH");
//     } else if (inChar == '2') {
//       digitalWrite(21, LOW);
//       Serial.println("Pin 21 set to LOW");
//     } else if (inChar == '3') {
//       digitalWrite(23, LOW);
//       Serial.println("Pin 23 set to LOW");
//     } else if (inChar == '4') {
//       digitalWrite(23, HIGH);
//       Serial.println("Pin 23 set to HIGH");
//     } else if (inChar == '5') {
//       digitalWrite(24, HIGH);
//       Serial.println("Pin 24 set to HIGH");
//     } else if (inChar == '6') {
//       digitalWrite(24, LOW);
//       Serial.println("Pin 24 set to LOW");
//     }
//   }
// }

void serialEvent1() {
  while (Serial1.available()) {
    int inChar = (int)Serial1.read();
    Serial.println("esp32:"); 
    Serial.println(inChar);
    if (espiqr == 1 && inChar == 3) {
      powerset = 1;
      digitalWrite(24, LOW);
      digitalWrite(23, LOW);
      digitalWrite(21, LOW);
    }
  }
}

void setup() {
  pinMode(24, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(22, INPUT);

  digitalWrite(21, HIGH);

  Serial.begin(115200);
  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);

  // Initialize SD card
  if (!SD.begin(chipSelect, SPI1)) {
    Serial.println("SD Card initialization failed!");
    return;
  }
  Serial.println("SD Card initialized.");

  espiqr = 1;
}

void loop() {
  int state = digitalRead(22); // Read pin state
  uint16_t adc0_data = analogRead(ADC0);
  float Voltage = adc0_data * (Vref / 1023.0);  // Corrected ADC voltage calculation
  float adjustedVoltage = Voltage * 2;

  // Print adjusted voltage to serial monitor
  Serial.println(adjustedVoltage, 3);

  // Check voltage threshold and set Pin 24 accordingly
  if (adjustedVoltage > 3.4) {
    digitalWrite(24, HIGH);
    Serial.println("Pin 24 set to HIGH");
  }
  if(adjustedVoltage <= 3.4 && adjustedVoltage > 2)  
  {
    digitalWrite(24, LOW);
    Serial.println("Pin 24 set to LOW");
  }

  // Prepare data to store on SD card
  SDDataString = "";
  SDDataString += String(adjustedVoltage, 3);
  SDDataString += ',';

  // Print measurement info
  Serial.printf("\r\n\r\n--------- start measure %d-------\r\n", cnt);

  // Write to SD card
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println(SDDataString);
    dataFile.close();
    Serial.print("SD write: ");
    Serial.println(SDDataString);
  } else {
    Serial.println("Error opening datalog.txt");
  }

  // Handle power down if powerset is activated
  if (powerset == 1) {
    Serial.println("power down");
    digitalWrite(24, LOW);
  }

  // Print pin 22 state
  if (state == LOW) {
    Serial.println("Pin22 is LOW");
  } else {
    Serial.println("Pin22 is HIGH");
  }
  cnt++;
  // Use delay or non-blocking delay
  delay(1000); // Delay for 1 second to prevent serial overloading
}
