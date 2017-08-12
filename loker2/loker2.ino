/* @file EventSerialKeypad.pde
  || @version 1.0
  || @author Alexander Brevig
  || @contact alexanderbrevig@gmail.com
  ||
  || @description
  || | Demonstrates using the KeypadEvent.
  || #
*/
#include <Keypad2.h>
#include <LiquidCrystal.h>
#include <avr/sleep.h>
//#include <MsTimer2.h>
//#include <TimerOne.h>
//#include "LowPower.h"
#include <avr/wdt.h>
#include <dumpmon.h>        // Include necessary header.

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'},

};  // end of keys

byte rowPins[ROWS] = { 5, 6, 7, 8 };
byte colPins[COLS] = { 9, 10, 11 , 12};
//of the keypad

// number of items in an array for pin re-configuration between sleep and keypad routines
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

byte ledPin = 13;
boolean ledPin_state;
unsigned long timeBegin;

// we don't want to do anything except wake
EMPTY_INTERRUPT (PCINT0_vect)
EMPTY_INTERRUPT (PCINT1_vect)
EMPTY_INTERRUPT (PCINT2_vect)

/*
  // функция обработки прерывания SPI
  ISR (TIMER2_COMPA_vect)
  {
  wakeUp();
  }

  ISR (TIMER2_COMPB_vect)
  {
  wakeUp();
  }

  ISR (TIMER2_OVF_vect)
  {
  wakeUp();
  }
*/


volatile byte bCount = 0; // use volatile for shared variables

void setup() {
  wdt_disable(); // бесполезная строка до которой не доходит выполнение при bootloop
  Serial.begin(115200);
  Serial.println("Setup..");

  Serial.println("Wait 5 sec..");
  delay(5000); // Задержка, чтобы было время перепрошить устройство в случае bootloop
  //wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
  Serial.println("Watchdog enabled.");


  pinMode(ledPin, OUTPUT);              // Sets the digital pin as output.
  digitalWrite(ledPin, HIGH);           // Turn the LED on.
  ledPin_state = digitalRead(ledPin);   // Store initial LED state. HIGH when LED is on.
  keypad.addEventListener(keypadEvent); // Add an event listener for this keypad

  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
  lcd.setCursor(0, 1);

  // pin change interrupt masks (see above list)
  PCMSK2 |= _BV (PCINT21);   // pin 5  - PORTD mask 0x20
  PCMSK2 |= _BV (PCINT22);   // pin 6  - PORTD mask 0x40
  PCMSK2 |= _BV (PCINT23);   // pin 7  - PORTD mask 0x80
  PCMSK0 |= _BV (PCINT0);    // pin 8  - PORTB mask 0x01

  //SetTimer1 ();

  timeBegin = millis();
  //prints time since program started
  Serial.println(timeBegin);
  // MsTimer2::start();

  // setup_watchdog(7);
  //dumpmonSetup(115200);

}


void SetTimer1 ()
{

  //MsTimer2::set(1000, TimerInterupt); // 1500ms period
  //MsTimer2::stop();
  //  Timer1.initialize(5000000);
  // Timer1.stop();
  // Timer1.attachInterrupt(TimerInterupt); // blinkLED to run every 0.15 seconds

}


void TimerInterupt ()
{

  // bCount++;
  //if ( bCount == 3 )
  {
    Serial.println ("set timer ...");
    //Timer1.stop();
    // goToSleep ();
    //reboot() ;
  }

}


//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;
  Serial.println(ww);

  MCUSR &= ~(1 << WDRF);
  // запуск таймера
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // установка периода срабатывания сторожевого таймера
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);

}
//****************************************************************
// Обработка прерывания сторожевого таймера / выполняется при истечении установленного для него периода
ISR(WDT_vect)
{
  bCount++;
}

void wdt_set ()
{
  // clear various "reset" flags
  MCUSR = 0;
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog

}

void reboot()
{
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}


void reconfigurePins ()
{
  byte i;

  // go back to all pins as per the keypad library

  for (i = 0; i < NUMITEMS (colPins); i++)
  {
    pinMode (colPins [i], OUTPUT);
    digitalWrite (colPins [i], HIGH);
  }  // end of for each column

  for (i = 0; i < NUMITEMS (rowPins); i++)
  {
    pinMode (rowPins [i], INPUT);
    digitalWrite (rowPins [i], HIGH);
  }   // end of for each row

}  // end of reconfigurePins

void wakeUp()
{

 
}


void goToSleep ()
{
  //  noInterrupts();
  Serial.println ("begin to sleep ...");
  digitalWrite(ledPin, LOW);

  byte i;

  // set up to detect a keypress
  for (i = 0; i < NUMITEMS (colPins); i++)
  {
    pinMode (colPins [i], OUTPUT);
    digitalWrite (colPins [i], LOW);   // columns low
  }  // end of for each column

  for (i = 0; i < NUMITEMS (rowPins); i++)
  {
    pinMode (rowPins [i], INPUT_PULLUP);
  }  // end of for each row

  // now check no pins pressed (otherwise we wake on a key release)
  for (i = 0; i < NUMITEMS (rowPins); i++)
  {
    if (digitalRead (rowPins [i]) == LOW)
    {
      reconfigurePins ();
      return;
    } // end of a pin pressed
  }  // end of for each row


  Serial.println ("Going to sleep ...");
  Serial.print ("PINB = ");
  Serial.println (PINB, HEX);
  Serial.print ("PIND = ");
  Serial.println (PIND, HEX);

  // overcome any debounce delays built into the keypad library
  delay (50);

  //interrupts();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  //set_sleep_mode (SLEEP_MODE_PWR_SAVE);
  // set_sleep_mode (SLEEP_MODE_IDLE);

  sleep_enable();

  byte old_ADCSRA = ADCSRA;
  // disable ADC to save power
  ADCSRA = 0;

  PRR = 0xFF;  // turn off various modules

  PCIFR  |= _BV (PCIF0) | _BV (PCIF1) | _BV (PCIF2);   // clear any outstanding interrupts
  PCICR  |= _BV (PCIE0) | _BV (PCIE1) | _BV (PCIE2);   // enable pin change interrupts

  // turn off brown-out enable in software
  MCUCR = _BV (BODS) | _BV (BODSE);
  MCUCR = _BV (BODS);
  sleep_cpu ();

  // cancel sleep as a precaution
  sleep_disable();
  PCICR = 0;  // cancel pin change interrupts
  PRR = 0;    // enable modules again
  ADCSRA = old_ADCSRA; // re-enable ADC conversion


  Serial.println ("Awake!");
  Serial.print ("PINB = ");
  Serial.println (PINB, HEX);
  Serial.print ("PIND = ");
  Serial.println (PIND, HEX);

  // put keypad pins back how they are expected to be
  reconfigurePins ();
  digitalWrite(ledPin, HIGH);


}  // end of goToSleep

void testTimeToSleep ()
{
  unsigned long timeNow;
  unsigned long timeDelta;

  timeNow = millis();
  timeDelta = (timeNow - timeBegin) / 1000;

  if (timeDelta > 15)
  {
    Serial.println("I want sleep ...");
    goToSleep ();
  }
  else
  {
    //wdt_reset();  // pat the dog
  }

}

void loop() {
  char key = keypad.getKey();

  if (key) {
    Serial.println(key);
    lcd.print((char) key);
    timeBegin = millis();
  }

  testTimeToSleep ();
  //dumpmonLoop();          // Вызов функции цикла библиотеки DumpMon
}

// Taking care of some special events.
void keypadEvent(KeypadEvent key) {

  switch (keypad.getState()) {
    case PRESSED:
      if (key == '#') {
        digitalWrite(ledPin, !digitalRead(ledPin));
        ledPin_state = digitalRead(ledPin);        // Remember LED state, lit or unlit.

      }
      // break;
      if (key == 'C') {
        lcd.setCursor(0, 1);
      }
      if (key == 'D') {
        lcd.setCursor(1, 1);
        //goToSleep();
      }
      break;


    case RELEASED:
      if (key == '*') {
        digitalWrite(ledPin, ledPin_state);   // Restore LED state from before it started blinking.
        goToSleep();
      }
      break;

    case HOLD:
      if (key == '*') {
        // goToSleep();
      }
      break;
  }
}
