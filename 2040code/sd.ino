#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
//#include <SensirionI2CScd4x.h>


#define ADC1  26

//SensirionI2CScd4x scd4x;
String SDDataString = "";

void sensor_power_on(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}

// void sensor_scd4x_init(void) {
//   uint16_t error;
//   char errorMessage[256];

//   //scd4x.begin(Wire);

//   // stop potentially previously started measurement
//   //error = scd4x.stopPeriodicMeasurement();
//   //if (error) {
//     //Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
//     //errorToString(error, errorMessage, 256);
//     //Serial.println(errorMessage);
//   //}

//   // Start Measurement
//   error = scd4x.startPeriodicMeasurement();
//   if (error) {
//     Serial.print("Error trying to execute startPeriodicMeasurement(): ");
//     //errorToString(error, errorMessage, 256);
//     Serial.println(errorMessage);
//   }
// }

void sensor_scd4x_get(void) {
  uint16_t error;
  char errorMessage[256];

  Serial.print("sensor scd4x: ");
  // Read Measurement
  uint16_t co2;
  float temperature;
  float humidity;
  //error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    Serial.print("Error trying to execute readMeasurement(): ");
    //errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (co2 == 0) {
    Serial.println("Invalid sample detected, skipping.");
  } else {
    Serial.print("Co2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.println(humidity);
  }

  SDDataString += "scd4x,";
  if (error) {
    SDDataString += "-,-,-,";
  } else {
    SDDataString += String(co2);
    SDDataString += ',';
    SDDataString += String(temperature);
    SDDataString += ',';
    SDDataString += String(humidity);
    SDDataString += ',';
  }
}

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
    }
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();

      Serial.println(inChar);

  }
}

int sensorValue = 0;  // 变量用于存储读取的值

int cnt = 0;
void setup() {
  Serial.begin(115200);

  sensor_power_on();
  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);
  // Wire.setSDA(20);
  // Wire.setSCL(21);
  // Wire.begin();

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
  digitalWrite(23, HIGH);
  
  digitalWrite(24, HIGH);
  delay(3000);
  //sensor_scd4x_init();
}

const float Vref = 3.3;

void loop() {


int state = digitalRead(22); // 读取引脚状态
  if (state == LOW) {
    Serial.println("Pin22 is LOW ");
  } else {
    Serial.println("Pin22 is HIGH ");
  }

  //Serial1.println(sensorValue);  // 打印出传感器的值
  delay(1000);
  uint16_t adc0_data = analogRead(ADC1);
  float Voltage = adc0_data * (Vref / 1024);
  Serial.println(Voltage * 2);
  float adjustedVoltage = Voltage * 2; 
  delay(1000);
  SDDataString = "";
  Serial.printf("\r\n\r\n--------- start measure %d-------\r\n", cnt);

  SDDataString += String(adjustedVoltage, 3);
  SDDataString += ',';

  cnt++;
  sensor_scd4x_get();
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
