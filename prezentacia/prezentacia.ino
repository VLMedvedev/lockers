#include <Keypad2.h>
//#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <OneWire.h>

#include <AltSoftSerial.h>
AltSoftSerial BarcodeScaner; // RX-8, TX-9

const byte MAX_INDEX = 32;
const byte LEN_ID_PACKET = 14;
const byte LEN_SERIAL_NUMBER = 8;
const byte LEN_STATUS_UNIT = 1;
const byte LEN_PASSWD = 6;
const int ADDRESS_SERIAL_NUMBER_ARRAY = 0;
const int ADDRESS_ID_PACKET_ARRAY = MAX_INDEX * LEN_SERIAL_NUMBER;
const int ADDRESS_STATUS_UNIT_ARRAY = ADDRESS_ID_PACKET_ARRAY + MAX_INDEX * LEN_ID_PACKET;
const int ADDRESS_PASSWD_ARRAY = ADDRESS_STATUS_UNIT_ARRAY + MAX_INDEX * LEN_STATUS_UNIT;
const int ADDRESS_COUNT_BOX = 1022;

byte count_box;

//SoftwareSerial BarcodeScaner(A0, A1); // RX, TX
char strBarcode[LEN_ID_PACKET];
String barcode = "";
String buff = "";

#define DS2413_ONEWIRE_PIN  (4)

#define DS2413_FAMILY_ID    0x3A
#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
#define DS2413_ACK_SUCCESS  0xAA
#define DS2413_ACK_ERROR    0xFF

OneWire oneWire(DS2413_ONEWIRE_PIN);

byte address[LEN_SERIAL_NUMBER];

byte address1[LEN_SERIAL_NUMBER] = { 0x3A, 0xA6, 0xB6, 0x06, 0x00, 0x00, 0x00, 0xAB };
byte address2[LEN_SERIAL_NUMBER] = { 0x3A, 0xC5, 0xB9, 0x06, 0x00, 0x00, 0x00, 0x5E };


/*
  0x3A,0xA6,0xB6,0x06,0x00,0x00,0x00,0xAB
  0x3A,0xC5,0xB9,0x06,0x00,0x00,0x00,0x5E
*/

#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

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

// Use pin 2 as wake up pin
const int wakeUpPin = 2;
byte ledPin = 13;
boolean ledPin_state;
String password = "      ";

volatile unsigned long timeBegin;

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


/********************************************************/
void printBytes(byte* addr, byte count, bool newline = 0)
{
  for (byte i = 0; i < count; i++)
  {
    Serial.print(addr[i] >> 4, HEX);
    Serial.print(addr[i] & 0x0f, HEX);
    Serial.print(" ");
  }
  if (newline)
  {
    Serial.println();
  }
}

void array_to_array (byte* array_out, byte* array_in, byte len)
{
  for (byte i = 0; i <= len; i++)
  {
    array_out[i] = array_in[i];
  }
}
/*******************************************************************/

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Wait 5 sec..");
  delay(500);
  //Serial.println("Tests ..");
  byte count = 1;

  // set the data rate for the SoftwareSerial port
  BarcodeScaner.begin(9600);
  //Serial.println("Tests ended");

  pinMode(ledPin, OUTPUT);              // Sets the digital pin as output.
  digitalWrite(ledPin, HIGH);           // Turn the LED on.
  ledPin_state = digitalRead(ledPin);   // Store initial LED state. HIGH when LED is on.
  keypad.addEventListener(keypadEvent); // Add an event listener for this keypad

  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Put password !");
  lcd.setCursor(0, 1);
  lcd.print(password);

  // pin change interrupt masks (see above list)
  PCMSK2 |= _BV (PCINT21);   // pin 5  - PORTD mask 0x20
  PCMSK2 |= _BV (PCINT22);   // pin 6  - PORTD mask 0x40
  PCMSK2 |= _BV (PCINT23);   // pin 7  - PORTD mask 0x80
  PCMSK0 |= _BV (PCINT0);    // pin 8  - PORTB mask 0x01

  // Configure wake up pin as input.
  // This will consumes few uA of current.
  pinMode(wakeUpPin, INPUT_PULLUP);
  // Allow wake up pin to trigger interrupt on low.
  attachInterrupt(0, wakeUp, LOW);

  timeBegin = millis();
  //prints time since program started
  //Serial.println(timeBegin);

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


  // Configure wake up pin as input.
  // This will consumes few uA of current.
  pinMode(wakeUpPin, INPUT_PULLUP);
  // Allow wake up pin to trigger interrupt on low.
  attachInterrupt(0, wakeUp, LOW);

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


void loop()
{
  // put your main code here, to run repeatedly:
  //Serial.println("Arbaiten !!!");
  //delay(3000);
  char c;
  char endstr = 0xD;
  char retstr = 0xA;


  if (BarcodeScaner.available()) {
    c = BarcodeScaner.read();
    //Serial.print(c);   Serial.print(" ");   Serial.println(c, HEX);
    if (c == endstr || c == retstr)
    {
      /*
        Serial.println(buff.length());
        Serial.println(buff);
        Serial.println(c, HEX);
      */
      if (buff.length() >= 2)
      {
        barcode = buff;
        buff = "";
        buff.trim();
        //Serial.print(" bc - ");
        Serial.print(barcode);
        compare_barcode (barcode);

      }
    }
    else
    {
      buff = String (buff + c);
    }


  }

  char key = keypad.getKey();

  if (key) {
    Serial.println(key);
    set_password(key);
    timeBegin = millis();
  }

  testTimeToSleep ();

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
        password = "      ";
        lcd.print(password);

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


void set_password(char key)
{
  if (key == 'C') {
    return;
  }
  for (int i = 0; i <= 5; i++)
  {
    password.setCharAt(i, password.charAt(i + 1));
  }
  password.setCharAt(5, key);
  lcd.setCursor(0, 1);
  //  lcd.print((char) key);
  lcd.print(password);
  compare_password (password);

}

void compare_password (String passwd)
{
  String passwd1 = "123456";
  String passwd2 = "007007";
  if (passwd == passwd1)
  {
    open_box_addr(address1);
  }
  if (passwd == passwd2)
  {
    open_box_addr(address2);
  }
}

void compare_barcode (String barcod)
{
  String bc1 = "RB298973247SG";
  String bc2 = "RM128973247SG";
  if (barcod == bc1)
  {
    open_box_addr(address1);
  }
  if (barcod == bc2)
  {
    open_box_addr(address2);
  }

}

void open_box_addr(byte* addr)
{
  for (byte i = 0; i < 7; i++)
  {
    address[i] = addr[i];
  }
  bool ok = false;
  ok = write(0x3);
  if (!ok) Serial.println(F("Wire failed"));
  delay(1000);
  ok = write(0x0);
  if (!ok) Serial.println(F("Wire failed"));
  delay(1000);
}


void reboot()
{
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void wakeUp()
{
  timeBegin = millis();
  Serial.println ("external wake up ...");

  reboot();
}


/***************************************1-Wire***************************/
byte read(void)
{
  bool ok = false;
  byte results;

  oneWire.reset();
  oneWire.select(address);
  oneWire.write(DS2413_ACCESS_READ);

  results = oneWire.read();                 /* Get the register results   */
  ok = (!results & 0x0F) == (results >> 4); /* Compare nibbles            */
  results &= 0x0F;                          /* Clear inverted values      */

  oneWire.reset();

  // return ok ? results : -1;
  return results;
}

bool write(byte state)
{
  byte ack = 0;

  /* Top six bits must '1' */
  state |= 0xFC;

  oneWire.reset();
  oneWire.select(address);
  oneWire.write(DS2413_ACCESS_WRITE);
  oneWire.write(state);
  oneWire.write(~state);                    /* Invert data and resend     */
  ack = oneWire.read();                     /* 0xAA=success, 0xFF=failure */
  if (ack == DS2413_ACK_SUCCESS)
  {
    oneWire.read();                          /* Read the status byte      */
  }
  oneWire.reset();

  return (ack == DS2413_ACK_SUCCESS ? true : false);
}


