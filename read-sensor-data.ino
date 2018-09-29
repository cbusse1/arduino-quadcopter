/*
   Dieses Programm zeigt wie die Sensordaten aus den Registern des MPU6050 Sensors ausgelesen werden koennen.
   Hierzu wird eine I2C Kommunikation mit dem Sensor aufgebaut
   Die I2C-Slave-Adresse der MPU6050 ist 0x68, wenn der AD0 Pin mit Ground verbunden ist (Dies ist bei der MultiWii-Flugsteuerungsplatine der Fall)
   siehe auch Datenblatt "MPU6050-Product Specification", S. 33

   Nach dem Einschalten ist die MPU6050 im sleep mode und die Messbereiche sind standardmaeßig auf +/- 250 deg/s (Gyro) und +/- 2g (Beschleunigungssensor) eingestellt
   (siehe Datenblatt S. 12-13). Diese Einstellung werden in diesem Programm auf +/- 500 deg/s und +/- 8g konfiguriert.

   MPU-6050 Six-Axis (Gyro + Accelerometer) https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
   Datenblatt: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
   Register uebersicht: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

*/

#include <Wire.h> //Bibliothek fuer I2C-Bus-Kommunikation
int gyroX, gyroY, gyroZ;  //Variablen zum Einlesen der Sensor-Rohdaten
long accX, accY, accZ;    //Variablen zum Einlesen der Sensor-Rohdaten
int temp;                 //Variablen zum Einlesen der Sensor-Rohdaten (Temperatur)
float roll_rate, pitch_rate, yaw_rate; //Variablen für Sensordaten: Winkelgeschwindigkeiten (Drehraten) in °/s
float accel_x, accel_y, accel_z;       //Variablen für Sensordaten: Translatorische Beschleunigungen in m/s^2
long gyroXCal, gyroYCal, gyroZCal;     //Kalibrierwerte für Gyroskop

void setup() {
  Serial.begin(115200);          //Startet Serielle uebertragung, Baudrate 57600
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

  /* Kalibrierwerte */
  gyroXCal = -105;
  gyroYCal = -44;
  gyroZCal = -54;
}

void loop() {
  Wire.beginTransmission(0x68); //Starte Kommunikation mit MPU6050
  Wire.write(0x3B);             //Sende Startregister, der auszulesenden Daten
  Wire.endTransmission();

  /*
    Die Messwerte der MPU6050 sind mit 16bit aufgeloest.
    Jeder Messwert ist auf zwei Registern abgelegt. Ein Register enthaelt ein byte an Daten.
    Mittels bitschift-Operation koennen die Daten aus den zwei Registern wieder zusammengefuegt werden
  */
  Wire.requestFrom(0x68, 14);   //14 bytes werden angefordert
  while (Wire.available() < 14);
  accX = Wire.read() << 8 | Wire.read(); //Low und High Byte in accX Variable zusammenfuegen
  accY = Wire.read() << 8 | Wire.read(); //Low und High Byte in accY Variable zusammenfuegen
  accZ = Wire.read() << 8 | Wire.read(); //Low und High Byte in accZ Variable zusammenfuegen
  temp = Wire.read() << 8 | Wire.read(); //Low und High Byte in temp Variable zusammenfuegen
  gyroX = Wire.read() << 8 | Wire.read(); //Low und High Byte in gyroX Variable zusammenfuegen
  gyroY = Wire.read() << 8 | Wire.read(); //Low und High Byte in gyroY Variable zusammenfuegen
  gyroZ = Wire.read() << 8 | Wire.read(); //Low und High Byte in gyroZ Variable zusammenfuegen

  /* Kalibrierwert subtrahieren */
  gyroX -= gyroXCal;
  gyroY -= gyroYCal;
  gyroZ -= gyroZCal;

  /* Transformation der Sensordaten, siehe Koordinatensystem.pdf */
  gyroX ^= gyroY;
  gyroY ^= gyroX;
  gyroX ^= gyroY;
  gyroZ = -gyroZ;
  accX ^= accY; //x = x ^ y; https://betterexplained.com/articles/swap-two-variables-using-xor/
  accY ^= accX; //y = x ^ y;
  accX ^= accY; //x = x ^ y;
  accX = -accX;
  accY = -accY;

  /* Umrechnung der Sensordaten in physikalische Einheiten */
  roll_rate = (float)gyroX / 65.5; //Rollrate in °/s umrechnen
  pitch_rate = (float)gyroY / 65.5; //Nickrate in °/s umrechnen
  yaw_rate = (float)gyroZ / 65.5; //Gierrate in °/s umrechnen
  accel_x  = (float)accX / 4096 * 9.81; //x-Beschleunigung in m/s^2
  accel_y  = (float)accY / 4096 * 9.81; //y-Beschleunigung in m/s^2
  accel_z  = (float)accZ / 4096 * 9.81; //z-Beschleunigung in m/s^2

  /* Sensordaten im Serial-Monitor ausgeben */
  Serial.print("Acc XYZ: ");
  Serial.print(accel_x);
  Serial.print("\t");
  Serial.print(accel_y);
  Serial.print("\t");
  Serial.print(accel_z);
  Serial.print("\t");
  Serial.print("Gyro XYZ: ");
  Serial.print(roll_rate);
  Serial.print("\t");
  Serial.print(pitch_rate);
  Serial.print("\t");
  Serial.println(yaw_rate);
  delay(50);
}
