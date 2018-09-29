/*
   Dieses Programm wertet das Gassignal u_gas (Kanal 3) vom RC-Empfänger aus
   und sendet die PWM-Stellsignale an die Brushless-Regler (Leistungsstellglieder).
   Weiter kann dieses Programm genutzt werden um:
   1. die Stellglieder zu kalibrieren
   2. die alle Empfängerkanäle zu testen (es werden alle eingelesen, aber nur Kanal 3 zur Motoransteuerung genutzt)
   WARNUNG: Vor dem Testen Propeller von den Motoren entfernen!
*/

#define LOOP_TIME 4000  //Schleifenzeit (=Abtastzeit) in Mikrosekunden [us]
unsigned long cycleTime, currentTime, previousTime;

/* Empfänger-Variablen */
byte stateCh1, stateCh2, stateCh3, stateCh4, stateCh5, stateCh6;
int rxInputCh1, rxInputCh2, rxInputCh3, rxInputCh4, rxInputCh5, rxInputCh6; //Variablen für gemessene PWM-Pulsbreite in us
unsigned long rxTimer1, rxTimer2, rxTimer3, rxTimer4, rxTimer5, rxTimer6;
unsigned long interruptTime;

int u_gas, u_1, u_2, u_3, u_4; //Motor-Stellgrößen

/* Debug Serial */
int serialCounter;
int idx;



// *************** SETUP-ROUTINE *************************** //
void setup() {
  Serial.begin(115200);          //Startet Serielle uebertragung

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
  /* Es werden Timer 1 und 2 zur Erzeugung der PWM-Signale genutzt.
    Die Frequenz beträgt f=490 Hz.
    Timer 2 ist ein 8-bit Timer
    TImer 1 ist ein 16-bit Timer der aber in 8-bit betrieben wird
  */
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

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

  /* Initialisierung */
  currentTime = micros();
  previousTime = currentTime;
  serialCounter = 1;
}



// *************** HAUPTPROGRAMMSCHLEIFE *************************** //
void loop() {
  /* Gassignal auf 1000 und 2000 us begrenzen */
  u_gas = rxInputCh3;
  if (u_gas < 1000) u_gas = 1000;
  if (u_gas > 2000) u_gas = 2000;

  u_1 = u_gas;
  u_2 = u_gas;
  u_3 = u_gas;
  u_4 = u_gas;

  /* Duty cycle (Pulsbreite) im Timer-Compare Register setzen */
  //OCR2B = u_1 >> 3; //D3 - Vorne Links
  OCR1A = u_2 >> 3; //D9 - Vorne Rechts
  //OCR1B = u_3 >> 3; //D10 - Hinten Rechts
  //OCR2A = u_4 >> 3; //D11 - Hinten Links

  /* Schleifenzeit-Kontrolle */
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

  /* Ausgabe Seriell Schnittstelle */
  debugSerial();
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

  /************** Channel 3 = u_gas (throttle) **************/
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



// *************** AUSGABE SERIELLE SCHNITTSTELLE   *************************** //
void debugSerial() {
  idx = 1;
  if (serialCounter == 1) {
    Serial.print(u_gas);
    Serial.print("\t");
    Serial.print(rxInputCh1);
    Serial.print("\t");
    Serial.print(rxInputCh2);
    Serial.print("\t");
    Serial.print(rxInputCh3);
    Serial.print("\t");
    Serial.print(rxInputCh4);
    Serial.print("\t");
    Serial.print(rxInputCh5);
    Serial.print("\t");
    Serial.println(rxInputCh6);
  }
  if (serialCounter == (4 + idx)) {
    serialCounter = 0;
  }
  serialCounter++;
}
