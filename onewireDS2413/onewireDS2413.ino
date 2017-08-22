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

struct onewire_address_type
{
  uint8_t SerialNumberUnit[8];
}  ;

String PassWord[32];

int len_eeAddress = 22;

//const byte secret[16] = "01234567890ABCDEF";


#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Wait 5 sec..");

  Serial.println("Tests ..");

  onewire_address_type onewire_address[24];
  uint8_t count = test_1_wire(onewire_address);
  Serial.print( count ); Serial.println(" "); 
  for (int i = 0; i < count+1; i++)
  {   
    printBytes(onewire_address[i].SerialNumberUnit, 8);
    //Serial.print( i ); 
    Serial.println(" "); 
 
  }

  //put_box_table ();
  //greate_passwd_table ();

  // set the data rate for the SoftwareSerial port
  //BarcodeScaner.begin(9600);
  Serial.println("Tests ended");

}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Arbaiten !!!");
  delay(3000);
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

uint8_t test_1_wire(onewire_address_type* onewire_address)
{

  uint8_t i = 0;
  uint8_t t = 0;

  Serial.println(F("Looking for a DS2413 on the bus"));

  /* Try to find a device on the bus */
  oneWire.reset_search();
  delay(500);

  while (oneWire.search(address))
  {
    /* Check the CRC in the device address */
    if (OneWire::crc8(address, 7) != address[7])
    {
      Serial.println(F("Invalid CRC!"));
      if (t >= 3)
      {
        i++;
      }
      else
      {
        i = 0;
        oneWire.reset_search();
      }
      t++;
     }
    else
    {
      for (int j=0; j <=7; j++)
      {
        onewire_address[i].SerialNumberUnit[j] = address[j];
      }
  
      i++;
    }
    delay(500);
  }

  /* Make sure we have a DS2413 */
  /*
    if (address[0] != DS2413_FAMILY_ID)
    {
     printBytes(address, 8);
     Serial.println(F(" is not a DS2413!"));
     //while (1);
    }

  */
  //test_wire1() ;
  
  return i-1;
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

