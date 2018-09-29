/*
   Dies ist das vollständige Flugsteuerungsprogramm für den Quadrocopter.
   Die Kalibrierwerte für das Gyroskop, Reglerparameter, Sampletime
   und Filterparameter sind voreingestellt, dass der Quadrocopter bereits flugtauglich ist.
*/

#include <ArduPID.h> //Bibliothek für PID-Algorithmus mit Tustin-Transformation, Quelle: https://github.com/Tellicious/ArduPID-Library/tree/master/ArduPID
#include <Wire.h> //Bibliothek fuer I2C Bus Kommunikation

#define LOOP_TIME 4000  //Schleifenzeit (=Abtastzeit) in Mikrosekunden [us]
uint32_t Ts = LOOP_TIME; //Sampletime in Mikrosekunde
uint32_t Ts_ms = Ts / 1e3; //Sampletime in Millisekunde
float Ts_sec; //Sampletime in Sekunde
unsigned long cycleTime, currentTime, previousTime;

// *************** Regler-Variablen *************************** //
double kp = 5;  //für Roll- und Pitch-Fluglagenregler
double ki = 0;
double kd = 0;
double kp_rate = 1; //für Roll- und Pitch-Drehratenregler
double ki_rate = 0;
double kd_rate = 0.0;
double kp_yaw = 1.5; //für Gierratenregler (Yaw)
double ki_yaw = 0;
double kd_yaw = 0;
double N = 20; //derivative filter constant D(s)=s/(1+s/N), a good rule is: N>10*Kd/Kp (also avoid too large values)

double w_roll, w_pitch, w_yaw_rate, w_roll_rate, w_pitch_rate;            //Führungsgrößen, w_roll (w_phi), w_roll_rate (w_phi_dot),...
double u_roll, u_pitch, u_yaw;                                            //PID-Ausgangsgrößen, Reglerstellgrößen  u_phi, u_theta, u_psi
double e_roll, e_pitch , e_yaw , e_roll_rate , e_pitch_rate , e_yaw_rate; //PID-Eingangsgrößen (Regelfehler e_roll)

PID PID1(&w_roll_rate, kp, ki, kd, N, Ts_ms);                 //Roll-Fluglageregler
PID PID1_rate(&u_roll, kp_rate, ki_rate, kd_rate, N, Ts_ms);  //Roll-Drehratenregler
PID PID2(&w_pitch_rate, kp, ki, kd, N, Ts_ms);                //Nick-Fluglageregler
PID PID2_rate(&u_pitch, kp_rate, ki_rate, kd_rate, N, Ts_ms); //Nick-Drehratenregler
PID PID3_rate(&u_yaw, kp_yaw, ki_yaw, kd_yaw, N, Ts_ms);      //Gier-Drehratenregler

int u_gas, u_1, u_2, u_3, u_4; //Motor-Stellgrößen

// *************** Sensordaten-Variablen *************************** //
int gyroX, gyroY, gyroZ;  //Variablen zum Einlesen der Sensor-Rohdaten
long accX, accY, accZ;    //Variablen zum Einlesen der Sensor-Rohdaten
long p1, p2, r1, r2;      //Variablen zur Auswertung der Winkelfunktionen
int temp;                 //Variablen zum Einlesen der Sensor-Rohdaten (Temperatur)
float roll_rate, pitch_rate, yaw_rate; //Variablen für Sensordaten: Winkelgeschwindigkeiten (Drehraten) in °/s
float accel_x, accel_y, accel_z;       //Variablen für Sensordaten: Translatorische Beschleunigungen in m/s^2
long gyroXCal, gyroYCal, gyroZCal;     //Kalibrierwerte für Gyroskop
float pitchAcc, rollAcc;               //Variablen für Lagewinkel, berechnet aus Beschleunigungsmessdaten
float angle_pitch, angle_roll;         //Variablen für Lagewinkel, berechnet mit Komplementärfilter

/* Komplementärfilter-Variablen */
float fg = 0.1; //Grenzfrequenz des Filters in Hz
float Tau, a1, a2;  //Zeitkonstante und Variablen zur Berechnung der Faktoren

// *************** RC-Empfänger-Variablen *************************** //
byte stateCh1, stateCh2, stateCh3, stateCh4, stateCh5, stateCh6;
int rxInputCh1, rxInputCh2, rxInputCh3, rxInputCh4, rxInputCh5, rxInputCh6; //Variablen für gemessene PWM-Pulsbreite in us
unsigned long rxTimer1, rxTimer2, rxTimer3, rxTimer4, rxTimer5, rxTimer6;
unsigned long interruptTime;
float stick_sensitivity = 15.0; //Skalierungsfaktor für den Stellweg der Steuerknüppel, stick_sensitivy = 15 -> 492/15 = ca. 33 Grad Stellweg

/* Debug-Variablen zur Ausgabe im Serial-Monitor */
int idx;
int serialCounter;

/* Statussvariablen */
int pid_status = 0;
int arm_status = 0;

// *************** SETUP-ROUTINE *************************** //
void setup() {
  /* Setup Serial */
  Serial.begin(115200);          //Startet Serielle uebertragung
  /* I2C-Bus */
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

  /*Kalibrierwerte Gyroskop */
  gyroXCal = -82;
  gyroYCal = -45;
  gyroZCal = -55;

  /* Setup PinChangeInterrupt Ports  */
  PCICR |= (1 << PCIE2); // PCIE2 bit im PCICR Register setzen um die Konfiguration der PinChangeInterrupts im PCMSK2 Register freizuschalten
  PCMSK2 |= (1 << PCINT18); //PCINT18 bit setzen um PinChangeInterrupt auf Digital Input Port 2 zu aktivieren
  PCMSK2 |= (1 << PCINT20); //PCINT20 bit setzen um PinChangeInterrupt auf Digital Input Port 4 zu aktivieren
  PCMSK2 |= (1 << PCINT21); //PCINT21 bit setzen um PinChangeInterrupt auf Digital Input Port 5 zu aktivieren
  PCMSK2 |= (1 << PCINT22); //PCINT22 bit setzen um PinChangeInterrupt auf Digital Input Port 6 zu aktivieren
  PCMSK2 |= (1 << PCINT23); //PCINT22 bit setzen um PinChangeInterrupt auf Digital Input Port 7 zu aktivieren

  /* Setup Channel 6 = AUX2 */
  PCICR |= (1 << PCIE0); // PCIE0 bit im PCICR Register setzen um die Konfiguration der PinChangeInterrupts im PCMSK0 Register freizuschalten
  PCMSK0 |= (1 << PCINT0); //PCINT0 bit setzen um PinChangeInterrupt auf Digital Input Port 7 zu aktivieren

  /* PWM Setup */
  pinMode(3, OUTPUT); //Pin 3 als Output Konfigurieren
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);  //blaue LED
  cli(); //Interrupts deaktivieren
  TCCR2A = _BV(WGM20);
  TCCR2B = _BV(CS22);
  TCCR1A = (1 << WGM11); // phase correct mode, ohne prescaler
  TCCR1A = (1 << WGM10); // phase correct mode, ohne prescaler
  TCCR1B = (1 << CS11) | (1 << CS10);
  TCCR1A |= _BV(COM1A1); // Pin D9 mit timer 1 verbinden (channel A)
  TCCR1A |= _BV(COM1B1); // Pin D10 mit timer 1 verbinden (channel B)
  TCCR2A |= _BV(COM2A1); // Pin D11 mit timer 2 verbinden (channel A)
  TCCR2A |= _BV(COM2B1); // Pin D3 mit timer 2 verbinden (channel B)
  sei(); //Interrupt aktivieren

  /* PID Stellgrößenbegrenzung*/
  PID1.SetSaturation(-1000, 1000);      //untere und obere Regler-Stellgrößenbeschränkung setzen
  PID1_rate.SetSaturation(-1000, 1000);
  PID2.SetSaturation(-1000, 1000);
  PID2_rate.SetSaturation(-1000, 1000);
  PID3_rate.SetSaturation(-1000, 1000);

  /* Komplementärfilter Variablen initialisieren */
  Tau = 1 / (2 * 3.14 * fg);  //Zeitkonstante des Filters
  Ts_sec = (float)Ts / 1e6; //Umrechnung von us in s
  a1 = Tau / (Tau + Ts_sec);  //Berechnung der Filterfaktoren
  a2 = Ts_sec / (Tau + Ts_sec);

  currentTime = micros();
  previousTime = currentTime;
  serialCounter = 1;
}

// *************** HAUPTPROGRAMMSCHLEIFE *************************** //
void loop() {

  // *************** Sensordaten-Auswertung *************************** //
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

  /* Winkelberechnung mit Komplementärfilter */
  angle_pitch = a1 * (angle_pitch + Ts_sec * pitch_rate) + a2 * pitchAcc;
  angle_roll = a1 * (angle_roll + Ts_sec * roll_rate) + a2 * rollAcc;

  // *************** Regler-Berechnung *************************** //
  /* Roll-Führungsgröße */
  /* Die Messung der Eingangs-PWM-Pulsbreiten hat eine Genauigkeit von 4 us,
    deswegen wird ein Totband von 16us um die Mittelstellung implementiert */
  w_roll = 0;
  if (rxInputCh1 > 1508)w_roll = (rxInputCh1 - 1508) / stick_sensitivity;
  else if (rxInputCh1 < 1492) w_roll = (rxInputCh1 - 1492) / stick_sensitivity;
  /* Roll-Lagerregler */
  e_roll = w_roll - angle_roll;       //Regelfehler berechnen
  PID1.SetTunings(kp, ki, kd, N);     //PID-Parameter aktualisieren
  PID1.Compute(e_roll);               //PID-Ausgangsgröße berechnen (w_roll_rate)
  /* Roll-Drehratenregler */
  if (kp == 0) {
    w_roll_rate = w_roll * 6.0;   //Lageregler inaktiv, Drehratenvorgabe
  }
  e_roll_rate = w_roll_rate - roll_rate;                //Regelfehler berechnen
  PID1_rate.SetTunings(kp_rate, ki_rate, kd_rate, N);   //PID-Parameter aktualisieren
  PID1_rate.Compute(e_roll_rate);                       //PID-Ausgangsgröße berechnen (u_roll)

  /* Nick-Führungsgröße */
  w_pitch = 0;
  if (rxInputCh2 > 1508)w_pitch = (rxInputCh2 - 1508) / stick_sensitivity;
  else if (rxInputCh2 < 1492) w_pitch = (rxInputCh2 - 1492) / stick_sensitivity;
  /* Nick-Lagerregler */
  e_pitch = w_pitch - angle_pitch;
  PID2.SetTunings(kp, ki, kd, N);
  PID2.Compute(e_pitch);                       //PID-Ausgangsgröße berechnen (w_pitch_rate)
  /* Nick-Drehratenregler */
  if (kp == 0) {
    w_pitch_rate = w_pitch * 6.0;
  }
  e_pitch_rate = w_pitch_rate - pitch_rate;
  PID2_rate.SetTunings(kp_rate, ki_rate, kd_rate, N);
  PID2_rate.Compute(e_pitch_rate);             //PID-Ausgangsgröße berechnen (u_pitch)

  /* Gierrate-Führungsgröße */
  w_yaw_rate = 0;
  if (rxInputCh4 > 1508)w_yaw_rate = (rxInputCh4 - 1508) / 3.0;
  else if (rxInputCh4 < 1492) w_yaw_rate = (rxInputCh4 - 1492) / 3.0;
  /* Gier-Drehratenregler */
  e_yaw_rate = w_yaw_rate - yaw_rate;
  PID3_rate.SetTunings(kp_yaw, ki_yaw, kd_yaw, N);
  PID3_rate.Compute(e_yaw_rate);               //PID-Ausgangsgröße berechnen (u_yaw)

  // *************** Motoransteuerung *************************** //
  /* Gassignal auf 1000 und 2000 us begrenzen */
  u_gas = rxInputCh3;
  if (u_gas < 1000) u_gas = 1000;
  if (u_gas > 2000) u_gas = 2000;
  if (rxInputCh5 < 1000) rxInputCh5 = 1000;
  if (rxInputCh5 > 2000) rxInputCh5 = 2000;

  /* Motoren  mit Kippschalter scharf schalten  */
  if (rxInputCh6 > 1500) {
    arm_status = 1;
    digitalWrite(13, HIGH);   //blaue LED an = Motoren aktiv
  }
  if (rxInputCh6 < 1500) {
    arm_status = 0;
    digitalWrite(13, LOW);
  }

  if (u_gas > 1200) {                  //Regler wird aktiv, wenn die Gasvorgabe größer 1200 us ist
    if (u_gas > 1800) u_gas = 1800;    //PID braucht Spielraum zum regeln

    /* Motoransteuerungsmatrix */
    u_1 = u_gas + u_roll + u_pitch + u_yaw;
    u_2 = u_gas - u_roll + u_pitch - u_yaw;
    u_3 = u_gas - u_roll - u_pitch + u_yaw;
    u_4 = u_gas + u_roll - u_pitch - u_yaw;
  }
  else {
    /* PID Reset */
    PID1.Reset();
    PID1_rate.Reset();
    PID2.Reset();
    PID2_rate.Reset();
    PID3_rate.Reset();
    u_1 = u_gas;
    u_2 = u_gas;
    u_3 = u_gas;
    u_4 = u_gas;
  }

  if (arm_status == 0) { //Motoren inaktiv
    /* PID Reset */
    PID1.Reset();
    PID1_rate.Reset();
    PID2.Reset();
    PID2_rate.Reset();
    PID3_rate.Reset();
    u_1 = 1000;
    u_2 = 1000;
    u_3 = 1000;
    u_4 = 1000;
    //kp_rate = mapfloat(rxInputCh5, 1000, 2000, 0, 5);    //Reglere-Parameter mit Drehpoti verstellen (nur möglich wenn Motoren inaktiv)
  }

  /* Motorstellsignal-Begrenzung */
  if (u_1 < 1000) u_1 = 1000;
  if (u_1 > 2000) u_1 = 2000;
  if (u_2 > 2000) u_2 = 2000;
  if (u_2 < 1000) u_2 = 1000;
  if (u_3 > 2000) u_3 = 2000;
  if (u_3 < 1000) u_3 = 1000;
  if (u_4 > 2000) u_4 = 2000;
  if (u_4 < 1000) u_4 = 1000;

  /* Duty cycle (Pulsbreite) im Timer-Compare Register setzen */
  OCR2B = u_1 >> 3; //D3 - Vorne Links
  OCR1A = u_2 >> 3; //D9 - Vorne Rechts
  OCR1B = u_3 >> 3; //D10 - Hinten Rechts
  OCR2A = u_4 >> 3; //D11 - Hinten Links

  /* Warte bis die LOOP_TIME erreicht ist, bevor der nächste Schleifendurchlauf startet */
  while (1) {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    #if defined(LOOP_TIME)
        if (cycleTime >= LOOP_TIME) break;
    #else
        break;
    #endif
  }
  previousTime = currentTime;

  /* Daten zum Debuggen an Serielle Schnittstelle senden */
  //debugSerial();
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

// *************** EMPFÄNGER-SIGNALE EINLESEN *************************** //
/* InterruptServiceRoutine zum einlesen der PWM-Signale vom Empfänger */
ISR(PCINT2_vect) {
  interruptTime = micros();
  /************** Channel 1 = Rollen (roll) **************/
  if (PIND & B00000100) {
    if (stateCh1 == 0) {
      stateCh1 = 1;
      rxTimer1 = interruptTime;
    }
  }
  else if (stateCh1 == 1) {
    stateCh1 = 0;
    rxInputCh1 = interruptTime - rxTimer1;
  }
  /************** Channel 2 = Nicken (pitch) **************/
  if (PIND & B00010000) {
    if (stateCh2 ==  0) {
      stateCh2 = 1;
      rxTimer2 = interruptTime;
    }
  }
  else if (stateCh2 == 1) {
    stateCh2 = 0;
    rxInputCh2 = interruptTime - rxTimer2;
  }
  /************** Channel 3 = Gas (throttle) **************/
  if (PIND & B00100000) {
    if (stateCh3 ==  0) {
      stateCh3 = 1;
      rxTimer3 = interruptTime;
    }
  }
  else if (stateCh3 == 1) {
    stateCh3 = 0;
    rxInputCh3 = interruptTime - rxTimer3;
  }
  /************** Channel 4 = Gieren (yaw) **************/
  if (PIND & B01000000) {
    if (stateCh4 ==  0) {
      stateCh4 = 1;
      rxTimer4 = interruptTime;
    }
  }
  else if (stateCh4 == 1) {
    stateCh4 = 0;
    rxInputCh4 = interruptTime - rxTimer4;
  }
  /************** Channel 5 = AUX1 **************/
  if (PIND & B10000000 ) {
    if (stateCh5 ==  0) {
      stateCh5 = 1;
      rxTimer5 = interruptTime;
    }
  }
  else if (stateCh5 == 1) {
    stateCh5 = 0;
    rxInputCh5 = interruptTime - rxTimer5;
  }
}

/************** Channel 6 = AUX2 **************/
ISR(PCINT0_vect) {
  interruptTime = micros();
  //Channel 6
  if (PINB & B00000001 ) {
    if (stateCh6 ==  0) {
      stateCh6 = 1;
      rxTimer6 = interruptTime;
    }
  }
  else if (stateCh6 == 1) {
    stateCh6 = 0;
    rxInputCh6 = interruptTime - rxTimer6;
  }
}

// *************** MAP-FUNKTION  *************************** //
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// *************** AUSGABE SERIELLE SCHNITTSTELLE   *************************** //
void debugSerial() {
  idx = 1;
  if (serialCounter == 1) {
    Serial.print(cycleTime);
    Serial.print(F("\t"));
    Serial.print(arm_status);
  }
  if (serialCounter == (2 + idx)) {
    Serial.print(F("\t"));
    Serial.print(angle_roll);
  }
  if (serialCounter == (3 + idx)) {
    Serial.print(F("\t"));
    Serial.print(angle_pitch);
  }
  if (serialCounter == (4 + idx)) {
    Serial.print(F("\t"));
    Serial.print(w_roll);
  }
  if (serialCounter == (5 + idx)) {
    Serial.print(F("\t"));
    Serial.print(w_pitch);
  }
  if (serialCounter == (6 + idx)) {
    Serial.print(F("\t"));
    Serial.println(kp_rate);
    serialCounter = 0;
  }
  serialCounter++;
}

