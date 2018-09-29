/*
   Dieses Programm berechnet die Kalibrierwerte für den Gyroskop.
   Während der Kalibrierung muss sich der Sensor in Ruhe befinden.
*/

#include <Wire.h> //Bibliothek fuer I2C-Bus-Kommunikation
int gyroX, gyroY, gyroZ;
int accX, accY, accZ;
int temp;
long gyroXCal, gyroYCal, gyroZCal;

void setup() {
  Serial.begin(115200);          //Startet Serielle uebertragung, Baudrate 115200
  Wire.begin();                 //I2C Bus beitreten, Adresse optional fuer Master
  TWBR = 12;                    //I2C-Bus-Frequenz auf 400kHz einstellen
  delay(250);                   //warten bis die MPU6050 startet
  
  /* Setup MPU6050 */
  Wire.beginTransmission(0x68); //Starte Kommunikation mit MPU6050
  Wire.write(0x6B);             //Sende Register, auf welches geschrieben werden soll (0x6B = PWR_MGMT_1)
  Wire.write(0x00);             //Setze Register 0x6B zu Null (holt die MPU6050 aus dem sleep mode raus)
  Wire.endTransmission();       //Beende die Übertragung
  
  /* Gyroskop konfigurieren */
  Wire.beginTransmission(0x68); //Starte Kommunikation mit MPU6050
  Wire.write(0x1B);             //Das GYRO_CONFIG register (0x1B) soll beschrieben werden
  Wire.write(0x08);             //Setze die bits im register auf 00001000 (FS_SEL=1, +/- 500 deg/s full scale range)
  Wire.endTransmission();       //Beende die Übertragung
  
  /* Beschleunigungssensor konfigurieren */
  Wire.beginTransmission(0x68); //Starte Kommunikation mit MPU6050
  Wire.write(0x1C);             //Das ACCEL_CONFIG register (0x1C) soll beschrieben werden
  Wire.write(0x10);             //Setze die bits im register auf 00010000 (AFS_SEL=2, +/- 8g full scale range)
  Wire.endTransmission();       //Beende die Übertragung
 
  /* Kalibrierungsroutine: Der Kalibrierungswert ist der Mittelwert aus 2000 Messwerte */
  Serial.println("Starte Kalibrierung...");
  for  (int i = 0; i < 2000; i++) {
    getMPUData(); //Sensordaten einlesen
    gyroXCal += gyroX;
    gyroYCal += gyroY;
    gyroZCal += gyroZ;
    delay(3);
  }
  gyroXCal /= 2000;
  gyroYCal /= 2000;
  gyroZCal /= 2000;
  Serial.println("----------------------------------------------");
  Serial.println("Kalibrierung beendet. Kalibrierwerte:");
  Serial.print("GyroXCal: ");
  Serial.print(gyroXCal);
  Serial.print('\t');
  Serial.print("GyroYCal: ");
  Serial.print(gyroYCal);
  Serial.print('\t');
  Serial.print("GyroZCal: ");
  Serial.println(gyroZCal);
}

void getMPUData() {
  Wire.beginTransmission(0x68); //Starte Kommunikation mit MPU6050
  Wire.write(0x3B);             //Sende Startregister, der auszulesenden Daten
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);   //14 bytes werden angefordert
  while (Wire.available() < 14);
  accX = Wire.read() << 8 | Wire.read(); //Low und High Byte in accX Variable zusammenfuegen
  accY = Wire.read() << 8 | Wire.read(); //Low und High Byte in accY Variable zusammenfuegen
  accZ = Wire.read() << 8 | Wire.read(); //Low und High Byte in accZ Variable zusammenfuegen
  temp = Wire.read() << 8 | Wire.read(); //Low und High Byte in temp Variable zusammenfuegen
  gyroX = Wire.read() << 8 | Wire.read(); //Low und High Byte in gyroX Variable zusammenfuegen
  gyroY = Wire.read() << 8 | Wire.read(); //Low und High Byte in gyroY Variable zusammenfuegen
  gyroZ = Wire.read() << 8 | Wire.read(); //Low und High Byte in gyroZ Variable zusammenfuegen
}

void loop() {
  /* wird nicht benötigt */
}

