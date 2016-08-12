/*
 * 433MHz Handsender mit 3 Buttons
 * 
 * Copyright (c) 2016 Peter Müller <peter@crycode.de>
 * 
 * 
 *                 ATtiny85
 *                  +-\/-+
 * Ain0 (D 5) PB5  1|    |8  Vcc
 * Ain3 (D 3) PB3  2|    |7  PB2 (D 2)  Ain1
 * Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
 *            GND  4|    |5  PB0 (D 0) pwm0
 *                  +----+
 */

/*
 * Adresse des MC im Funknetz von RadioHead.
 */
#define rh_addr 2

/*
 * Adresse des Servers im Funknetz von RadioHead.
 */
#define rh_server_addr 1

/*
 * Bitrate in Bit/s vom RadioHead.
 */
#define rh_speed 2000

/*
 * RX-/TX-/PTT-Pin für RadioHead.
 * 433MHz Empfänger/Sender.
 * PTT ungenutzt, sollte ein freier Pin des MC sein.
 */
#define rh_rx_pin 6 // nicht vorhanden
#define rh_tx_pin 3
#define rh_ptt_pin 7 // nicht vorhanden

/*
 * Pins der Taster S1, S2 und S3
 */
#define taster_1 0
#define taster_2 1
#define taster_3 2

/*
 * LED-Pin
 */
#define led 4

/*
 * Zeit die die LED beim Senden leuchten soll in Millisekunden
 */
#define led_time 250

/* 
 * Maximale RadioHead Nachrichtenlänge
 */
#define RH_ASK_MAX_MESSAGE_LEN 2

/*
 *******************************************
 */

// Sleep-Funktionen des MC
#include <avr/sleep.h>

// RadioHead
#include <RH_ASK.h>
#include <RHDatagram.h>

RH_ASK rh_driver(rh_speed, rh_rx_pin, rh_tx_pin, rh_ptt_pin);
RHDatagram rh_manager(rh_driver, rh_addr);


// Buffer für die zu sendenden Nachrichten
uint8_t rh_buf[2] = { 0x01, 0x01 };
/*
 * rh_buf[0] => Länge der Nachricht
 * rh_buf[1] => Befehl (0x01 Taster 1, 0x02 Taster 2, 0x03 Taster 3)
 */

// Zustände der Taster
uint8_t t1;
uint8_t t2;
uint8_t t3;
uint8_t t1_last = HIGH;
uint8_t t2_last = HIGH;
uint8_t t3_last = HIGH;

/*
 * Porgrammstart
 */
void setup() {

  // I/Os einrichten
  pinMode(taster_1, INPUT);
  pinMode(taster_2, INPUT);
  pinMode(taster_3, INPUT);
  pinMode(led, OUTPUT);

  // Pullups aktivieren?
  digitalWrite(taster_1, HIGH);
  digitalWrite(taster_2, HIGH);
  digitalWrite(taster_3, HIGH);

  // RadioHead initialisieren
  if(!rh_manager.init()){
    // Init fehlgeschlagen... LED blinken und dann in endlosschleife wechseln
    for(uint8_t i=0; i<10; i++){
      digitalWrite(led, HIGH);
      delay(200);
      digitalWrite(led, LOW);
      delay(200);
    }
    for(;;){ }
  }

  // PCINT für die 3 Taster aktivieren
  PCMSK |= ((digitalPinToPCMSKbit(taster_1)) || (digitalPinToPCMSKbit(taster_2)) || (digitalPinToPCMSKbit(taster_3)));

  // PCIE im GIMSK Register aktivieren
  GIMSK |= (1<<5);
}

/*
 * Hauptschleife
 */
void loop() {

  // Prozessor schlafen legen, bis ein Interrupt kommt
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  //sei();
  sleep_cpu();
  sleep_disable();
  
  // aufwachen...

  // disable all interrupts
  //cli();

  // Taster abfragen...
  t1 = digitalRead(taster_1);
  t2 = digitalRead(taster_2);
  t3 = digitalRead(taster_3);

  // ... und auswerten
  if(t1 != t1_last){
    if(t1 == LOW){
      // T1 gedrückt
      senden(0x01);
    }
    // entprellen
    delay(50);

    t1_last = t1;
    
  }else if(t2 != t2_last){
    if(t2 == LOW){
      // T2 gedrückt
      senden(0x02);
    }
    // entprellen
    delay(50);

    t2_last = t2;
    
  }else if(t3 != t3_last){
    if(t3 == LOW){
      // T3 gedrückt
      senden(0x03);
    }
    // entprellen
    delay(50);

    t3_last = t3;
  }
}

/*
 * Senden eines Befehls an den Server.
 */
void senden(uint8_t cmd){
  // LED einschalten
  digitalWrite(led, HIGH);

  // Befehl in den Buffer schreiben
  rh_buf[1] = cmd;

  // Senden
  rh_manager.sendto(rh_buf, 2, rh_server_addr);

  // Kurz warten und LED wieder ausschalten
  delay(led_time);
  digitalWrite(led,LOW);
}

/*
 * ISR für den PinChangeInterrupt.
 */
ISR (PCINT0_vect){   
  // hier gibt es nichts zu tun... nur aufwachen
}
