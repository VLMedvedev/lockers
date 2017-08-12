/************************************************************************************************************/
/*
  "Arduino_deep_sleep" is an example of the proper and most advanced way to
  put ANY Arduino boards to sleep.
  
  Puts any arduino in to deep sleep for 30 sec. Wakes it up and turns on LED
  connected to the pin 0 and puts arduino back to sleep.
  
  Average ATtiny85 consumption in deep sleep mode is about 3 microamps
  at 3.3 volt.

  written by enjoyneering79
*/
/************************************************************************************************************/
#include <avr/sleep.h>     //AVR MCU power management.
#include <avr/power.h>     //AVR MCU peripheries (Analog comparator, ADC, USI, Timers/Counters etc) management.
#include <avr/wdt.h>       //AVR MCU watchdog timer management.
#include <avr/io.h>        //AVR MCU IO ports management.
#include <avr/interrupt.h> //AVR MCU interrupt flags management.

#include <Keypad2.h>
#include <LiquidCrystal.h>
/************************************************************************************************************/
/*
    Constants

    Enables interrupts (instead of MCU reset), when watchdog is timed out.
    Used for wake-up MCU from power-down/sleep.
*/
/************************************************************************************************************/
#define WDTCR |= _BV(WDIE)


/************************************************************************************************************/
/*
    Variables
*/
/************************************************************************************************************/
volatile byte watchdog_counter = 0;
volatile boolean f_wdt = false;

byte ledPin = 13;
boolean blink = false;
boolean ledPin_state;
unsigned long timeBegin;

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

/************************************************************************************************************/
/*
    ISR(WDT_vect)
    
    Watchdog Interrupt Service (automatically executed when watchdog is timed out)
*/
/************************************************************************************************************/
ISR(WDT_vect)
{
  watchdog_counter++;
  f_wdt = false;
}

/************************************************************************************************************/
/*
    Setup()

    Main setup
*/
/************************************************************************************************************/
void setup()
{
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


  timeBegin = millis();
  //prints time since program started
  Serial.println(timeBegin);

  //setup_watchdog1(8);
  
 //  setup_watchdog(WDTO_8S); //approximately 8 sec. of sleep  
}


void reboot()
{
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}


/************************************************************************************************************/

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
  f_wdt = false;
}

void testTimeToSleep ()
{
  unsigned long timeNow;
  unsigned long timeDelta;

  timeNow = millis();
  timeDelta = (timeNow - timeBegin) / 1000;

  if (timeDelta > 5)
  {
    Serial.println("I want sleep ...");
    //goToSleep ();
  }
  else
  {
    //wdt_reset();  // pat the dog
  }

}


// Taking care of some special events.
void keypadEvent(KeypadEvent key) {
  watchdog_counter = 0;  
  f_wdt = false;

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
        goToSleep();
      }
      break;


    case RELEASED:
      if (key == 'A') {
        digitalWrite(ledPin, ledPin_state);   // Restore LED state from before it started blinking.
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

/************************************************************************************************************/
/*
    loop()

    Main loop
*/
/************************************************************************************************************/
void loop()
{
  while (watchdog_counter < 4) //wait for watchdog counter reched the limit (WDTO_8S * 4 = 32sec.)
  {
    go_sleep();
  }
  
  watchdog_counter = 0;     //reset watchdog_counter
  power_all_enable();       //enable all peripheries (timer0, timer1, Universal Serial Interface, ADC)
  /*
  power_adc_enable();       //enable ADC
  power_timer0_enable();    //enable Timer0
  power_timer1_enable();    //enable Timer1
  power_usi_enable();       //enable the Universal Serial Interface module
  */
  delay(5);                 //to settle down the ADC and peripheries

 
   wdt_disable();
    timeBegin = millis();
    
    Serial.println ("Awake!");
    Serial.print ("PINB = ");
    Serial.println (PINB, HEX);
    Serial.print ("PIND = ");
    Serial.println (PIND, HEX);

    // put keypad pins back how they are expected to be
    reconfigurePins ();
    delay(50);
    
    digitalWrite(ledPin, HIGH);

 char key = keypad.getKey();

  if (key) {
    Serial.println(key);
    lcd.print((char) key);
    timeBegin = millis();
    watchdog_counter = 0;  
    f_wdt = false;
  }
  if (blink) {
    digitalWrite(ledPin, !digitalRead(ledPin));   // Change the ledPin from Hi2Lo or Lo2Hi.
    delay(100);
  }

  //testTimeToSleep ();
  
}


/************************************************************************************************************/
/*
    go_sleep()

    Puts MCU into the sleep state

    NOTE: There are 6 different sleeps modes:
          * SLEEP_MODE_IDLE..........The least power savings state. CPU stopped but Analog
                                     comparator, ADC, USI, Timer/Counter, Watchdog (if enabled),
                                     & the interrupt system continues operating. (by default in "sleep.h")
          * SLEEP_MODE_ADC...........ADC Noise Reduction. CPU stopped but the ADC, the external
                                     interrupts, & the Watchdog (if enabled) continue operating.
          * SLEEP_MODE_PWR_SAVE......Supported by Atiny25, Atiny45, Atiny85.
          * SLEEP_MODE_EXT_STANDBY...Not supported by Atiny25, Atiny45, Atiny85.
          * SLEEP_MODE_STANDBY.......Not supported by Atiny25, Atiny45, Atiny85.
          * SLEEP_MODE_PWR_DOWN......The most power savings state. All oscillators are stopped, only an
                                     External Reset, Watchdog Reset, Brown-out Reset, USI start condition
                                     interupt & external level interrupt on INT0 or a pin change interrupt
                                     can wake up the MCU.      
*/
/************************************************************************************************************/
void go_sleep()
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
  Serial.println (watchdog_counter);
  

  // overcome any debounce delays built into the keypad library
  delay (50);
  
  power_all_disable();                 //disable all peripheries (timer0, timer1, Universal Serial Interface, ADC)
  /*              
  power_adc_disable();                 //disable ADC
  power_timer0_disable();              //disable Timer0
  power_timer1_disable();              //disable Timer2
  power_usi_disable();                 //disable the Universal Serial Interface module.
  */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set the sleep type
  sleep_mode();                        /*system stops & sleeps here (automatically sets the SE (Sleep Enable) bit
                                         (so the sleep is possible), goes to sleep, wakes-up from sleep after an
                                         interrupt (if interrupts are enabled) or WDT timed out (if enabled) and
                                         clears the SE (Sleep Enable) bit afterwards).
                                         the sketch will continue from this point after interrupt or WDT timed out
                                       */
}


/************************************************************************************************************/
/*
    setup_watchdog()
    
    Sleeps intervals: WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS,
                      WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S
    
    NOTE: The MCU watchdog runs from internal 128kHz clock and continues to work
          during the deepest sleep modes to provide a wake up source.
*/
/************************************************************************************************************/
void setup_watchdog(byte sleep_time)
{
  wdt_enable(sleep_time);
}


