
/* 
Arduino Speed Detector
=============================================================================================================

Hardware:
---------
Controller
1x Arduino UNO oder Pro Mini

Sensor
1x QRD1114 IR Reflective Object Sensor @5V with restistors to apply 1,7V max!
1x 220 Ohm current-limiting resistor
1x 10k pull-up resistor
Alternativ:
bipolarer Hallsensor TLE 4935L mit Neodymmagnete (5x2mm)

Display
1x 8x2 LCD 


The circuit:
------------

LCD:
	Syntax: LiquidCrystal(rs, rw, enable, d4, d5, d6, d7) 
	Example:
		LiquidCrystal lcd(8, 9, 4, 5, 6, 7); 
		* LCD RS pin to digital pin 8
		* LCD Enable pin to digital pin 9
		* LCD D4 pin to digital pin 4
		* LCD D5 pin to digital pin 5
		* LCD D6 pin to digital pin 6
		* LCD D7 pin to digital pin 7
		* LCD VSS pin to ground
		* LCD VCC pin to 5V
		* 10K variable resistor:
			* ends to +5V and ground
			* wiper to LCD VO pin (pin 3)

Sensor:
	connect sensor pin 1 and 3 to GND
	connect sensor pin 2 to Arduino Analog in (PIN A1)
	connect sensor pin 4 to +5V
	
Behavior:
---------
  Operation:
	IR sensor detects a marker at a rotating part and counts its revolution
	Displays the calculated speed of the rotation in rmp at the 8x2 LCD

  
Coding assistance:
------------------

http://forum.zerspanungsbude.net/viewtopic.php?t=1601
http://circuitdigest.com/microcontroller-projects/tachometer-using-arduino

LCD Hookup
http://learning.grobotronics.com/2013/10/controlling-lcd-displays-8x2-raystar/
https://www.arduino.cc/en/Tutorial/HelloWorld
http://linksprite.com/wiki/index.php5?title=16_X_2_LCD_Keypad_Shield_for_Arduino


QRD1114 Hookup
http://dm.risd.edu/pbadger/PhysComp/index.php?n=Devices.QRD1114
https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide

=============================================================================================================
*/

//===========================================================================
//=================DEFINITIONS===============================================
//===========================================================================

//Libraries to include
   
	#include <LiquidCrystal.h>                    // load LCD library

// == constants won't change. They're used here to set pin numbers:
	#define LCDdimmer 10                          // LCD Hintergrundbeleuchtung an Pin 10
	#define Eingang 2                             // Drehzahl-Eingang Pin2 / Interrupt 0	
	#define AVG 8                                 // runden über 8 Werte                                
	#define REFRESH 200                           // Display refresh alle 200ms
	#define HELLIGKEIT 200                        // LCD Hintergrundbeleuchtung
	#define SENSORS 1                             // Anzahl Sensoren / Impulse pro U

// == other definitions
	LiquidCrystal lcd(8, 9, 4, 5, 6, 7);      // Display Anschlußpins definieren

// == variables will change:
	uint32_t last=0;                              // Zählerwert beim letzten Interrupt
	uint16_t rra[AVG];                            // Array für rolling Average
	uint8_t ptr=0;                                // Array Pointer
	volatile int32_t prep=0;                      // Avg prep
	volatile uint8_t cnt=0;                       // Avg count
	volatile uint16_t drehzahl=0;                 // selbstredend
	volatile uint16_t timeout=10;                 // dynamic timeout
	volatile uint16_t wdog=10;                    // um Stillstand zu erkennen
	char buf[18];                                 // Pufferstring für sprintf
	

//===========================================================================
//=================SETUP - code to run once==================================
//===========================================================================

void setup() 
{
  pinMode(Eingang, INPUT);                    // Eingangspin auf Eingang stellen
//  digitalWrite(Eingang, HIGH);                // und Pullup-Widerstand einschalten bei HALL
  pinMode(LCDdimmer, OUTPUT);                 // Display-Dimmer-Pin auf Ausgang
  analogWrite(LCDdimmer, HELLIGKEIT);         // Helligkeit einstellen (via PWM)
  lcd.begin(16, 2);                            // Display hat 2 Zeilen a 8 Zeichen
  lcd.home();                                 // cursor an den Anfang der 1. Zeile
  lcd.print("-=- Drehzahl -=-");                      // Überschrift ausgeben
  lcd.setCursor(0, 1);                        // cursor an den Anfang der 2. Zeile
  lcd.print("** ???? U/min **");                      // 2. Zeile ausgeben
  attachInterrupt(0, readmicros, FALLING);    // Interrupt 0 auf Routine readmicros setzen
}

//===========================================================================
//=================LOOP - main code to run repeatedly========================
//===========================================================================

void loop() 
{                                             // Hauptprogramm
  if (wdog <= timeout) {                      // dynamic timeout
    wdog++;
  } else {
    cli();                                    // kein Impuls erhalten
    drehzahl = 0;                             // alle Werte auf 0
    cnt = 0;
    prep = 0;
    last = 0;
    sei();
  }
  drehzahl = myround(drehzahl);               // runden auf 5 / 10 / 50 / 100 - optional, wen nicht benötigt, zeile auskommentieren
  sprintf(buf, "%5u ", drehzahl);             // als 5stellig formatierte Zahl in den Puffer 
  lcd.setCursor(2, 1);                        // cursor an Position bringen
  lcd.print(buf);                             // Puffer ausgeben
  delay(REFRESH);                             // etwas warten
}

void readmicros() 
{                                             // Interrupt-Routine
  uint32_t m, dur;
  uint16_t rpm;
  m = micros();                               // Microsekundenzähler auslesen
  dur = m - last;                             // Differenz zum letzten Durchlauf berechnen
  if (dur > 5000L) {                          // plausibel? Drehzahl bis max 12000 U/min
    if (last > 0) {                           // ersten Messzyklus ignorieren
      rpm = (60000000L / SENSORS) / dur;      // Drehzahl ausrechnen
      if (cnt < AVG) {                        // |
        cnt++;                                // |
      } else {                                // |
        prep -= rra[ptr];                     // |
      }                                       // | rolling Average
      rra[ptr++] = rpm;                       // |
      prep += rpm;                            // |
      ptr %= AVG;                             // |
      drehzahl = prep / cnt;                  // |
    }
    last = m;                                 // letzten Wert merken
    timeout = dur / (REFRESH * 500L);         // timeout nach 2 fehlenden Impulsen
  }
  wdog = 0;                                   // timeout watchdog
}

uint16_t myround(uint16_t value) 
{                                             // Gewichtete Rundung
  uint16_t rto;
  if (value > 10000) {                        // Rundungswert bestimmen  
    rto = 100;
  } else if (value > 3000) {
    rto = 50;
  } else if (value > 500) {
    rto = 10;
  } else if (value > 100) {
    rto = 5;
  } else {
    return (value);
  }
  return (_myround(value, rto));
}

uint16_t _myround(uint16_t value, uint8_t roundto) 
{                                             // Rundung Hilfsroutine
  value += (roundto >> 1);                    // halben roundto Wert addieren
  value /= roundto;                           // integer division  
  value *= roundto;                           // integer multiplikation
  return (value);
}
  

//===========================================================================
//================= E N D   of   S K E T C H ================================
//===========================================================================

