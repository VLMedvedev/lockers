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
#include <TimerOne.h>
//#include "LowPower.h"


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
byte colPins[COLS] = { 9, 10, 11 ,12};  
//of the keypad


// number of items in an array for pin re-configuration between sleep and keypad routines
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
byte ledPin = 13; 

boolean blink = false;
boolean ledPin_state;

/*
// we don't want to do anything except wake
EMPTY_INTERRUPT (PCINT0_vect)
EMPTY_INTERRUPT (PCINT1_vect)
EMPTY_INTERRUPT (PCINT2_vect)
*/

// функция обработки прерывания SPI
ISR (PCINT0_vect)
{
  wakeUp();
}

ISR (PCINT1_vect)
{
  wakeUp();
}

ISR (PCINT2_vect)
{
  wakeUp();
}


volatile byte bCount = 0; // use volatile for shared variables

void setup(){
    Serial.begin(115200);
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

   SetTimer1 ();

 // MsTimer2::start();
}


void SetTimer1 ()
{
 
  //MsTimer2::set(5000, TimerInterupt); // 1500ms period
  //MsTimer2::start();
   Timer1.initialize(5000000);
   Timer1.stop();
  Timer1.attachInterrupt(TimerInterupt); // blinkLED to run every 0.15 seconds
  
}


void TimerInterupt ()
{

   bCount++;
  if ( bCount == 3 ) 
  {
    Serial.println ("set timer ...");
    //Timer1.stop();
    //goToSleep ();
  }

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
  /*
    // Just a handler for the pin interrupt.
  Serial.println ("Awake!");
  Serial.print ("PINB = ");
  Serial.println (PINB, HEX);
  Serial.print ("PIND = ");
  Serial.println (PIND, HEX);
  
  // put keypad pins back how they are expected to be
  reconfigurePins ();
              digitalWrite(ledPin,HIGH);
     */           
}
  

void goToSleep ()
  {
  //  noInterrupts();
    Serial.println ("begin to sleep ...");
    digitalWrite(ledPin,LOW);
      
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


 // ATmega328P, ATmega168
 // LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF,TIMER2_OFF);
//LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF,TIMER2_OFF);

  
  // at this point, pressing a key should connect the high in the row to the 
  // to the low in the column and trigger a pin change

  //interrupts(); 
  //set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  set_sleep_mode (SLEEP_MODE_PWR_SAVE);  
  
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
              digitalWrite(ledPin,HIGH);
      
    
  }  // end of goToSleep

void loop(){
    char key = keypad.getKey();

    if (key) {
        Serial.println(key);
        lcd.print((char) key);
    }
    if (blink){
        digitalWrite(ledPin,!digitalRead(ledPin));    // Change the ledPin from Hi2Lo or Lo2Hi.
        delay(100);
    }
}

// Taking care of some special events.
void keypadEvent(KeypadEvent key){

   //MsTimer2::stop();
  // MsTimer2::start();
bCount=0;
  //Timer1.restart();
  Timer1.stop();
  Timer1.start();
   
    switch (keypad.getState()){
    case PRESSED:
        if (key == '#') {
            digitalWrite(ledPin,!digitalRead(ledPin));
            ledPin_state = digitalRead(ledPin);        // Remember LED state, lit or unlit.
              
        }
       // break;
        if (key == 'C') {
            lcd.setCursor(0, 1);  
        }
          if (key == 'D') {
            lcd.setCursor(1, 1); 
            goToSleep(); 
        }
        break;
        

    case RELEASED:
        if (key == 'A') {
            digitalWrite(ledPin,ledPin_state);    // Restore LED state from before it started blinking.
            blink = false;
             goToSleep();
        }
        break;

    case HOLD:
        if (key == '*') {
            blink = true;    // Blink the LED when holding the * key.
           // goToSleep();
        }
        break;
    }
}
