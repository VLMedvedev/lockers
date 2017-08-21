
#include <Keypad2.h>
//#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
//#include <dumpmon.h>        // Include necessary header.
#include <OneWire.h>
#include <EEPROM.h>

#include <AltSoftSerial.h>
AltSoftSerial BarcodeScaner; // RX-8, TX-9

//SoftwareSerial BarcodeScaner(A0, A1); // RX, TX
char strBarcode[16];

#define DS2413_ONEWIRE_PIN  (4)

#define DS2413_FAMILY_ID    0x3A
#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
#define DS2413_ACK_SUCCESS  0xAA
#define DS2413_ACK_ERROR    0xFF

OneWire oneWire(DS2413_ONEWIRE_PIN);
uint8_t address[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
/*
  typedef struct
  {
  String SerialNumberUnit = "3A1234567";
  String ID_paket = "LF202929536CN";
  byte StatusUnit;
  }  record_type;
*/

struct record_type
{
  char SerialNumberUnit[7];
  char ID_paket[14];
  byte StatusUnit;
}  ;

String PassWord[32];

int len_eeAddress = 22;

//const byte secret[16] = "01234567890ABCDEF";
const uint8_t secretkey[] = {55, 129, 2, 45, 4, 115, 6, 7, 211, 9, 10, 11, 192, 13, 14, 15};

char passwd[7] = "      " ;
String pass_wd;

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

void setup() {
  wdt_disable(); // бесполезная строка до которой не доходит выполнение при bootloop
  Serial.begin(115200);
  Serial.println("Wait 5 sec..");
  delay(1000); // Задержка, чтобы было время перепрошить устройство в случае bootloop
  //wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
  //Serial.println("Watchdog enabled.");


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

  // setup_watchdog(7);
  //dumpmonSetup(115200);

  //len_eeAddress += sizeof(record); //Move address to the next byte after  'record'.
  for (byte i = 0; i <= 5; i++)
  {
    test_1_wire();
  }
  //put_box_table ();
  //greate_passwd_table ();

  // set the data rate for the SoftwareSerial port
  //BarcodeScaner.begin(9600);

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
  timeBegin = millis();
  Serial.println ("external wake up ...");

  reboot();
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

void loop() {

  if (BarcodeScaner.available()) {
    char c = BarcodeScaner.read();
    Serial.write(c);
    lcd.setCursor(0, 1);
    lcd.print(c);
    
  }


  char key = keypad.getKey();

  if (key) {
    Serial.println(key);
    set_password(key);
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

/********************************************************/
void printBytes(uint8_t* addr, uint8_t count, bool newline = 0)
{
  for (uint8_t i = 0; i < count; i++)
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

String calk_password(String data)
{
  uint8_t j = 0; // counter summ byte
  uint8_t i = 0; //counter main
  uint8_t k = 0; // counter pass
  uint8_t p = 0; // pointer
  boolean f_and = false; // and or
  boolean f_rol = false; // rol ror
  uint8_t data_byte;
  uint32_t pass_num = 0;
  String pass = "";

  p = secretkey[0] % 16;
  if (secretkey[0] > 128)
  {
    f_rol = true;
  }
  if (secretkey[1] > 128)
  {
    f_and = true;
  }

  // Serial.println(p, HEX);
  //   Serial.print(" ");

  while ( i < 16 )
  {
    //delay(100);

    data_byte = data.charAt(p);
    /*
      Serial.print(data_byte, HEX);
      Serial.print(" key ");
      Serial.print(key[i], HEX);
      Serial.print(" ");
      Serial.print(data_byte, HEX);
      Serial.print("  ");
      Serial.print(data_byte, HEX);
      Serial.print("  ");
      Serial.print(" i - ");
      Serial.print(i, HEX);
      Serial.print(" p - ");
      Serial.println(p, HEX);
    */
    if (f_and)
    {
      data_byte = data_byte | secretkey[i];
    }
    else
    {
      data_byte = data_byte & secretkey[i];
    }
    data_byte = data_byte ^ secretkey[i];

    pass_num = pass_num + data_byte;

    if (!f_rol)
    {
      //    Serial.print(" ror - ");
      p++;
      if (p >= 16)
      {
        p = 0;
      }
    }
    else
    {
      //    Serial.print(" rol - ");
      p--;
      //   Serial.print(" p - ");
      //  Serial.println(p, HEX);
      if (p == 255)
      {
        p = 15;
      }
      //   Serial.print(" p - ");
      //   Serial.println(p, HEX);
    }

    i++;
    j++;

    //   Serial.print(pass_num);
    //  Serial.print(" ");

    if ( j >= 3 )
    {
      String num = String( pass_num % 9);
      //   Serial.println(num);
      pass = pass + num;
      //pass.setCharAt(k , num);

      k++;
      pass_num = 0;
      j = 0;
    }
  }
  String num = String(char(48 + pass_num % 9));
  //    Serial.println(num);
  pass = pass + num;
  // pass.setCharAt(k , num);

  Serial.println(pass);
  return pass;
}

void set_str (char* strrez, char* strset, byte len)
{
  for (byte i = 0; i <= len; i++)
  {
    strrez[i] = strset[i];
  }
}

void put_box_table ()
{
  /*
    record_type record;

    int eeAddress = 256;
    set_str(record.SerialNumberUnit, "123456",7);
    set_str(record.ID_paket, "RB298973247SG", 14)  ;
    record.StatusUnit = 0x43;

    Serial.println(" structur ");
    Serial.println(eeAddress);
    //Serial.println(record.SerialNumberUnit );
    Serial.println(record.ID_paket );
    Serial.println(record.StatusUnit);
    // Serial.println(record.PassWord);

    //One simple call, with the address first and the object second.
    EEPROM.put(eeAddress, record);
    delay(350);

    eeAddress = 256;
    eeAddress = eeAddress + len_eeAddress;
    Serial.println(len_eeAddress);
    Serial.println(eeAddress);
    // record_type record = { "3A2397", "LF202929536CN", 0x43  };
    set_str(record.SerialNumberUnit, "3A2397", 7);
    set_str(record.ID_paket, "LF202929536CN", 14) ;
    record.StatusUnit = 0x43;

    EEPROM.put(eeAddress, record);
    delay(350);

    eeAddress = 256;
    eeAddress = eeAddress + (2 * len_eeAddress);
    Serial.println(len_eeAddress);
    Serial.println(eeAddress);
    set_str(record.SerialNumberUnit, "3A4697", 7);
    set_str(record.ID_paket, "MF102229526RN", 14);
    record.StatusUnit = 0x43;

    EEPROM.put(eeAddress, record);
    delay(350);

  */

  get_tabl_unit (1);
  get_tabl_unit (0);
  get_tabl_unit (2);

}

record_type get_tabl_unit (byte ID)
{
  record_type record;
  int eeAddress = 256;
  eeAddress = eeAddress + (ID * len_eeAddress);
  Serial.print(" read - len_eeAddress ");
  Serial.println(len_eeAddress);
  Serial.println(eeAddress);
  EEPROM.get(eeAddress, record);
  delay(150);
  Serial.println("Read custom object from EEPROM: - 1 ");
  Serial.println(record.SerialNumberUnit);
  Serial.println(record.ID_paket);
  Serial.println(record.StatusUnit);
  delay(150);
  return record;
}

void greate_passwd_table ()
{
  for (byte i = 0; i <= 10; i++)
  {
    record_type record = get_tabl_unit (i);
    String ID_paket = record.ID_paket;
    ID_paket.trim();
    Serial.print(" ID_paket ");
    Serial.println(ID_paket);
    if (record.StatusUnit >= 12 )
    {
      String pass = calk_password (ID_paket) ;
      Serial.print(" pass ");
      Serial.println(pass);
      PassWord[i] = pass;
    }
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

}

/***************************************1-Wire***************************/


byte read(void)
{
  bool ok = false;
  uint8_t results;

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

bool write(uint8_t state)
{
  uint8_t ack = 0;

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

void test_1_wire()
{

  Serial.println(F("Looking for a DS2413 on the bus"));

  /* Try to find a device on the bus */
  //oneWire.reset_search();
  //delay(250);
  if (!oneWire.search(address))
  {
    // printBytes(address, 8);
    Serial.println(F("No device found on the bus!"));
    oneWire.reset_search();
    delay(500);
    return;
  }

  /* Check the CRC in the device address */
  if (OneWire::crc8(address, 7) != address[7])
  {
    Serial.println(F("Invalid CRC!"));
    while (1);
  }

  /* Make sure we have a DS2413 */
  if (address[0] != DS2413_FAMILY_ID)
  {
    printBytes(address, 8);
    Serial.println(F(" is not a DS2413!"));
    while (1);
  }

  Serial.print(F("Found a DS2413: "));
  printBytes(address, 8);
  Serial.println(F(""));

  test_wire1() ;
}

void test_wire1()
{
  /* Read */

  uint8_t state = read();
  if (state == -1)
    Serial.println(F("Failed reading the DS2413"));
  else
    Serial.println(state, BIN);

  Serial.println();

  /* Write */
  /*
    bool ok = false;
    ok = write(0x3);
    if (!ok) Serial.println(F("Wire failed"));
    delay(1000);
    ok = write(0x0);
    if (!ok) Serial.println(F("Wire failed"));
    delay(1000);
  */
}

