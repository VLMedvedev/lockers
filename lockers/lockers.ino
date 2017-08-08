#include <Keypad2.h>
#include <avr/sleep.h>

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

// number of items in an array for pin re-configuration between sleep and keypad routines
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

const byte ledPin = 13;

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// we don't want to do anything except wake
EMPTY_INTERRUPT (PCINT0_vect)
EMPTY_INTERRUPT (PCINT1_vect)
EMPTY_INTERRUPT (PCINT2_vect)

void setup()
   {
   pinMode (ledPin, OUTPUT);

   // pin change interrupt masks (see above list)
   PCMSK2 |= _BV (PCINT21);   // pin 5  - PORTD mask 0x20
   PCMSK2 |= _BV (PCINT22);   // pin 6  - PORTD mask 0x40
   PCMSK2 |= _BV (PCINT23);   // pin 7  - PORTD mask 0x80
   PCMSK0 |= _BV (PCINT0);    // pin 8  - PORTB mask 0x01
   
   Serial.begin (115200);
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

void goToSleep ()
  {
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

  /*
  Serial.println ("Going to sleep ...");
  Serial.print ("PINB = ");
  Serial.println (PINB, HEX);
  Serial.print ("PIND = ");
  Serial.println (PIND, HEX);
  */
  // overcome any debounce delays built into the keypad library
  delay (50);
  
  // at this point, pressing a key should connect the high in the row to the 
  // to the low in the column and trigger a pin change
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
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
/*
  Serial.println ("Awake!");
  Serial.print ("PINB = ");
  Serial.println (PINB, HEX);
  Serial.print ("PIND = ");
  Serial.println (PIND, HEX);
  */
  // put keypad pins back how they are expected to be
  reconfigurePins ();
    
  }  // end of goToSleep

void loop()
{
   byte key = keypad.getKey();

   if (!key) 
      {
       goToSleep ();
       delay(10);
       return;
      }

   acknowledgeKeypress(key);
}

void acknowledgeKeypress(byte key) 
  {
   Serial.print ("Got key: ");
   Serial.println ((char) key);
  }
