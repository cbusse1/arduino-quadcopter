/*
   Diese Programm liest die Sensordaten des MPU-6050 ein und
   berechnete die Lagewinkel (Roll- , Nick- und Gierwinkel) mit drei verschiedenen Methoden:
   1. Berechnung aus Beschleunigungsmessdaten
   2. Berechnung durch Integration der Gyroskopdaten
   3. Berechnung mittels Komplementärfilter
*/

#include <Wire.h> //Bibliothek fuer I2C-Bus-Kommunikation
#define LOOP_TIME 4000  //Schleifenzeit (=Abtastzeit) in Mikrosekunden [us]
uint32_t Ts = LOOP_TIME; //Sampletime in Mikrosekunde
float Ts_sec; //Sampletime in Sekunde
unsigned long cycleTime, currentTime, previousTime;

// *************** Sensordaten-Variablen *************************** //
int gyroX, gyroY, gyroZ;  //Variablen zum Einlesen der Sensor-Rohdaten
long accX, accY, accZ;    //Variablen zum Einlesen der Sensor-Rohdaten
long p1, p2, r1, r2;      //Variablen zur Auswertung der Winkelfunktionen
int temp;                 //Variablen zum Einlesen der Sensor-Rohdaten (Temperatur)
float roll_rate, pitch_rate, yaw_rate; //Variablen für Sensordaten: Winkelgeschwindigkeiten (Drehraten) in °/s
float accel_x, accel_y, accel_z;       //Variablen für Sensordaten: Translatorische Beschleunigungen in m/s^2
long gyroXCal, gyroYCal, gyroZCal;     //Kalibrierwerte für Gyroskop
float pitchAcc, rollAcc;               //Variablen für Lagewinkel, berechnet aus Beschleunigungsmessdaten
float pitchGyro, rollGyro, yawGyro;    //Variablen für Lagewinkel, berechnet aus Gyroskopdaten
float angle_pitch, angle_roll;         //Variablen für Lagewinkel, berechnet mit Komplementärfilter


/* Komplementärfilter-Variablen */
float fg = 0.1; //Grenzfrequenz des Filters in Hz
float Tau, a1, a2; //Zeitkonstante und Variablen zur Berechnung der Faktoren

/* Debug-Variablen zur Ausgabe im Serial-Monitor */
int idx;
int serialCounter;



// *************** SETUP-ROUTINE *************************** //
void setup() {
  Serial.begin(115200);          //Startet Serielle uebertragung
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

  /* Komplementärfilter Variablen initialisieren */
  Tau = 1 / (2 * 3.14 * fg);  //Zeitkonstante des Filters
  Ts_sec = (float)Ts / 1e6; //Umrechnung von us in s
  a1 = Tau / (Tau + Ts_sec);  //Berechnung der Filterfaktoren
  a2 = Ts_sec / (Tau + Ts_sec);

  currentTime = micros();
  previousTime = currentTime;
}



// *************** HAUPTPROGRAMMSCHLEIFE *************************** //
void loop() {
  /* Sensordaten einlesen */
  getMPUData();

  /* Winkelberechnung aus Beschleungiungsdaten */
  p1 = -accX;
  p2 = sqrt((accY * accY) + (accZ * accZ));
  r1 = accY;
  r2 = accZ;
  if ( (p1 != 0) && (p2 != 0) ) {    //Verhindern, dass atan2() ungültige Werte (nan) liefert
    pitchAcc = atan2(p1, p2) * 57.296; //Nickwinkel in Grad
  }
  if ( (r1 != 0) && (r2 != 0) ) {    //Verhindern, dass atan2() ungültige Werte (nan) liefert
    rollAcc = atan2(r1 , r2) * 57.296; //Rollwinkel in Grad
  }

  /* Winkelberechnung aus Gyroskopsdaten (wird für den Komplementärfilter nicht benötigt, hier nur zum Vergleich) */
  pitchGyro += pitch_rate * Ts_sec;  //Integration Nickwinkel
  rollGyro += roll_rate * Ts_sec;   //Integration Rollwinkel
  yawGyro += yaw_rate * Ts_sec;    //Integration Gierwinkel

  /* Winkelberechnung mit Komplementärfilter */
  angle_pitch = a1 * (angle_pitch + Ts_sec * pitch_rate) + a2 * pitchAcc;
  angle_roll = a1 * (angle_roll + Ts_sec * roll_rate) + a2 * rollAcc;

  /* Für die Integration wird eine konstante SampleTime bzw. LOOP_TIME benötigt */
  while (1) {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    #if defined(LOOP_TIME)
        if (cycleTime >= LOOP_TIME) break;  //LOOP_TIME oben am Anfang des Skripts definieren!
    #else
        break;
    #endif
  }
  previousTime = currentTime;

  /* Sensordaten im Serial-Monitor ausgeben */
  debugSerial();
}



// *************** SENSORDATEN AUSLESEN  *************************** //
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
  //  accel_x  = (float)accX / 4096 * 9.81; //x-Beschleunigung in m/s^2
  //  accel_y  = (float)accY / 4096 * 9.81; //y-Beschleunigung in m/s^2
  //  accel_z  = (float)accZ / 4096 * 9.81; //z-Beschleunigung in m/s^2
}



// *************** AUSGABE SERIELLE SCHNITTSTELLE   *************************** //
/* Die gemessenen und berechneten Daten werden über die Serielle Schnittstelle ausgegeben
   und können über Tools/Serial-Monitor oder Tools/Serial-Plotter beobachtet werden.
   Die Daten werden nur im jeden 5ten Schleifendurchgang versendet (4ms * 5 = 20 ms -> 50 Hz).
   Dadurch werden Verzögerung in der Anzeige im Serial-Monitor vermieden.
*/
void debugSerial() {
  idx = 1;
  if (serialCounter == 1) {
    //    Serial.print(cycleTime);
    //    Serial.print("\t");
    Serial.print(pitchAcc);
    Serial.print("\t");
    Serial.print(pitchGyro);
    Serial.print("\t");
    Serial.print(angle_pitch);
    Serial.print("\t");
    Serial.print(rollAcc);
    Serial.print("\t");
    Serial.print(rollGyro);
    Serial.print("\t");
    Serial.println(angle_roll);
  }
  if (serialCounter == (4 + idx)) {
    serialCounter = 0;
  }
  serialCounter++;
}

